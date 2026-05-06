# bus_broker/machine/can.py

"""
Fake machine.CAN class.
Connects to the shared memory bus broker.
Matches the MicroPython machine.CAN API.

This is the most important file in the machine module -
it is what makes ECU code talk to the virtual bus.
"""

import time
import threading
from typing import Callable

from ..transport.shm_writer import SHMBusReader, SHMBusWriter, make_slot
from ..protocols.can_protocol import CANProtocol
from ..protocols.can_fd_protocol import CANFDProtocol
from ..core.signal import SignalConverter, DifferentialSignal
from ..core.frames import CANFrame, Protocol


class CAN:
    """
    Simulated CAN controller connected to shared memory bus.

    Usage (identical to real MicroPython machine.CAN):
        can = CAN(0, baudrate=500_000)
        can.setfilter(0, CAN.MASK16, 0, (0x100, 0x7FF))
        if can.any():
            arb_id, rtr, fdf, data = can.recv()
        can.send(bytes([0x01, 0x02]), 0x110)
    """

    # Filter modes - match MicroPython constants
    MASK16 = 0
    MASK32 = 1
    LIST16 = 2
    LIST32 = 3

    # Bus state
    STOPPED  = 0
    ERROR_ACTIVE  = 1
    ERROR_PASSIVE = 2
    BUS_OFF  = 3

    def __init__(
        self,
        bus      : int,
        mode     : int = ERROR_ACTIVE,
        baudrate : int = 500_000,
        prescaler: int = 1,
        sjw      : int = 1,
        bs1      : int = 6,
        bs2      : int = 8,
        auto_restart: bool = False
    ):
        self._bus_id    = bus
        self._baudrate  = baudrate
        self._filters   : list[tuple[int,int]] = []
        self._rx_queue  : list[CANFrame] = []
        self._rx_lock   = threading.Lock()
        self._reader    = None
        self._writer    = None
        self._running   = True
        self._state     = self.ERROR_ACTIVE

        # Callbacks (MicroPython supports recv callbacks)
        self._recv_callback: Callable | None = None

        # Protocol handler
        self._protocol  = CANProtocol(baudrate)
        self._conv      = SignalConverter()

        self._connect()

        # Background polling thread
        self._poll_thread = threading.Thread(
            target = self._poll_loop,
            daemon = True,
            name   = f"CAN{bus}-poll"
        )
        self._poll_thread.start()

        print(f"[CAN] Bus {bus} initialised at {baudrate:,} bit/s")

    def _connect(self):
        """Connect to shared memory bus. Retries if not ready."""
        deadline = time.time() + 10.0   # 10 second timeout
        while time.time() < deadline:
            try:
                self._reader = SHMBusReader()
                self._writer = SHMBusWriter()
                return
            except RuntimeError:
                time.sleep(0.1)
        print(
            f"[CAN] Warning: could not connect to bus broker. "
            f"Running in offline mode."
        )

    # ── Filter API ────────────────────────────────────────────────────────────

    def setfilter(
        self,
        bank  : int,
        mode  : int,
        fifo  : int,
        params: tuple,
        rtr   : bool = False
    ):
        """
        Set receive filter.

        MASK16: params = (id, mask)
            Accept frame if (arb_id & mask) == (id & mask)

        LIST16: params = (id1, id2, ...)
            Accept frames with exactly these IDs
        """
        if mode == self.MASK16:
            if len(params) >= 2:
                id_, mask = params[0], params[1]
                self._filters.append((id_, mask))
                print(
                    f"[CAN] Filter bank={bank}: "
                    f"MASK16 id=0x{id_:03X} mask=0x{mask:03X}"
                )
        elif mode == self.LIST16:
            for id_ in params:
                self._filters.append((id_, 0x7FF))
                print(f"[CAN] Filter bank={bank}: LIST16 id=0x{id_:03X}")

    def clearfilter(self):
        """Remove all filters - accept all frames."""
        self._filters.clear()
        print("[CAN] All filters cleared")

    def _passes_filter(self, arb_id: int) -> bool:
        """Return True if this frame ID passes any configured filter."""
        if not self._filters:
            return True
        for id_, mask in self._filters:
            if (arb_id & mask) == (id_ & mask):
                return True
        return False

    # ── Receive API ───────────────────────────────────────────────────────────

    def any(self) -> bool:
        """Return True if at least one frame is waiting."""
        with self._rx_lock:
            return len(self._rx_queue) > 0

    def recv(
        self,
        fifo   : int = 0,
        timeout: int = 5000
    ) -> tuple:
        """
        Receive a CAN frame.

        Returns:
            (arbitration_id, rtr, fdf, data)
            - arbitration_id : int
            - rtr            : bool  (remote frame)
            - fdf            : bool  (CAN FD frame)
            - data           : bytes

        Raises:
            OSError if timeout expires with no frame.
        """
        deadline = time.time() + timeout / 1000.0

        while time.time() < deadline:
            with self._rx_lock:
                if self._rx_queue:
                    frame = self._rx_queue.pop(0)
                    return (
                        frame.arbitration_id,
                        frame.is_remote,
                        frame.protocol == Protocol.CAN_FD,
                        frame.data
                    )
            time.sleep(0.001)

        raise OSError("CAN recv timeout")

    def recv_cb(self, callback: Callable):
        """
        Register a callback to be called when a frame arrives.
        callback receives (arb_id, rtr, fdf, data).
        This is an extension beyond MicroPython API for convenience.
        """
        self._recv_callback = callback

    # ── Transmit API ──────────────────────────────────────────────────────────

    def send(
        self,
        data   : bytes | bytearray,
        id     : int,
        timeout: int  = 5000,
        rtr    : bool = False
    ):
        """
        Send a CAN frame.

        data : bytes to send (max 8 for CAN, 64 for CAN FD)
        id   : arbitration ID
        """
        if self._writer is None:
            print(f"[CAN] Cannot send - not connected to bus")
            return

        data = bytes(data)

        frame = CANFrame(
            arbitration_id = id,
            dlc            = len(data),
            data           = data,
            protocol       = Protocol.CAN,
            is_remote      = rtr
        )

        signal = self._protocol.encode(frame)
        slot   = make_slot(signal, frame)
        ok     = self._writer.write(slot)

        if not ok:
            print(f"[CAN] Warning: bus full, frame 0x{id:03X} dropped")

    # ── Status API ────────────────────────────────────────────────────────────

    def state(self) -> int:
        """Return bus state. Matches MicroPython CAN.state()"""
        return self._state

    def info(self) -> list:
        """
        Return bus info list.
        Matches MicroPython CAN.info() format:
        [tx_errors, rx_errors, 0, 0, tx_queue, rx_queue]
        """
        with self._rx_lock:
            rx_waiting = len(self._rx_queue)
        return [0, 0, 0, 0, 0, rx_waiting]

    def restart(self):
        """Restart the CAN controller. No-op in simulation."""
        self._state = self.ERROR_ACTIVE
        print(f"[CAN] Bus {self._bus_id} restarted")

    def deinit(self):
        """Shut down the CAN controller."""
        self._running = False
        if self._reader:
            self._reader.close()
        if self._writer:
            self._writer.close()
        print(f"[CAN] Bus {self._bus_id} deinitialised")

    # ── Background poll ───────────────────────────────────────────────────────

    def _poll_loop(self):
        """
        Background thread that continuously reads from shared memory
        and puts matching frames into the receive queue.
        """
        while self._running:
            if self._reader is None:
                time.sleep(0.01)
                continue

            slot = self._reader.read()
            if slot is None:
                time.sleep(0.0001)   # 100us poll interval
                continue

            try:
                frame = self._decode_slot(slot)
                if frame and self._passes_filter(frame.arbitration_id):
                    with self._rx_lock:
                        self._rx_queue.append(frame)
                    if self._recv_callback:
                        self._recv_callback(
                            frame.arbitration_id,
                            frame.is_remote,
                            frame.protocol == Protocol.CAN_FD,
                            frame.data
                        )
            except Exception as e:
                print(f"[CAN] Decode error: {e}")

    def _decode_slot(self, slot) -> CANFrame | None:
        """Decode a BusFrameSlot back into a CANFrame."""
        try:
            bit_count = slot.bit_count
            # Guard against mock objects or invalid values
            if not isinstance(bit_count, int) or bit_count <= 0:
                return None

            num_bytes = (bit_count + 7) // 8
            if num_bytes > 256:
                return None

            canh = bytes(slot.canh_bytes[:num_bytes])
            canl = bytes(slot.canl_bytes[:num_bytes])

            signal = DifferentialSignal(
                canh_bytes = canh,
                canl_bytes = canl,
                bit_count  = bit_count
            )

            if slot.protocol == 0:
                return self._protocol.decode(signal)
            return None

        except Exception:
            return None

    def __repr__(self) -> str:
        return (
            f"CAN(bus={self._bus_id}, "
            f"baudrate={self._baudrate}, "
            f"filters={len(self._filters)})"
        )