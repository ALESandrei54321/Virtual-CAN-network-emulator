# bus_broker/transport/shm_writer.py

import ctypes
import os
import time
from pathlib import Path

# ── Mirror the C structs exactly ──────────────────────────────────────────────

MAX_SIGNAL_BYTES = 256
RING_SIZE        = 4096

class BusFrameSlot(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("timestamp_ns",    ctypes.c_uint64),
        ("arbitration_id",  ctypes.c_uint32),
        ("bit_count",       ctypes.c_uint16),
        ("protocol",        ctypes.c_uint8),
        ("is_extended",     ctypes.c_uint8),
        ("is_remote",       ctypes.c_uint8),
        ("brs_index",       ctypes.c_uint16),   # bit where data-rate begins (0=CAN)
        ("_pad",            ctypes.c_uint8 * 1),
        ("canh_bytes",      ctypes.c_uint8 * MAX_SIGNAL_BYTES),
        ("canl_bytes",      ctypes.c_uint8 * MAX_SIGNAL_BYTES),
    ]


def _load_library() -> ctypes.CDLL:
    lib_path = Path(__file__).parent / "libringbuffer.so"
    if not lib_path.exists():
        raise FileNotFoundError(
            f"libringbuffer.so not found at {lib_path}. "
            f"Run 'make' in bus_broker/transport/ first."
        )
    lib = ctypes.CDLL(str(lib_path))

    lib.shm_create.restype        = ctypes.c_void_p
    lib.shm_create.argtypes       = []

    lib.shm_reset.restype         = ctypes.c_void_p
    lib.shm_reset.argtypes        = []

    lib.shm_open_existing.restype  = ctypes.c_void_p
    lib.shm_open_existing.argtypes = []

    lib.shm_close.restype         = None
    lib.shm_close.argtypes        = [ctypes.c_void_p]

    lib.shm_unlink_bus.restype    = None
    lib.shm_unlink_bus.argtypes   = []

    lib.shm_write.restype         = ctypes.c_int
    lib.shm_write.argtypes        = [
        ctypes.c_void_p,
        ctypes.POINTER(BusFrameSlot)
    ]

    lib.shm_read.restype          = ctypes.c_int
    lib.shm_read.argtypes         = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_uint64),
        ctypes.POINTER(BusFrameSlot)
    ]

    lib.shm_available.restype     = ctypes.c_uint64
    lib.shm_available.argtypes    = [ctypes.c_void_p, ctypes.c_uint64]

    return lib


_lib = _load_library()


# ── Public Python API ─────────────────────────────────────────────────────────

class SHMBusWriter:
    """
    Used by the Bus Broker to write encoded frames into shared memory.
    Does NOT unlink shm on close - shm persists for readers.
    """

    def __init__(self):
        self._bus = _lib.shm_create()
        if not self._bus:
            raise RuntimeError("Failed to create shared memory bus")

    def write(self, slot: BusFrameSlot) -> bool:
        result = _lib.shm_write(self._bus, ctypes.byref(slot))
        return result == 0

    def close(self):
        if self._bus:
            # Only unmap, do not unlink.
            # shm object stays in /dev/shm for readers.
            _lib.shm_close(self._bus)
            self._bus = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


class SHMBusReader:
    """
    Used by each ECU process to read frames from shared memory.
    Each reader maintains its own position in the ring buffer.
    Starts at the current write index so it only sees new frames,
    not frames that were written before it connected.
    """

    def __init__(self):
        self._bus = _lib.shm_open_existing()
        if not self._bus:
            raise RuntimeError(
                "Failed to open shared memory bus. "
                "Is the broker running?"
            )
        # Start at current write index - skip all past frames
        current_write = self._get_write_index()
        self._consumer_index = ctypes.c_uint64(current_write)

    def _get_write_index(self) -> int:
        """Read the current write index directly from shared memory."""
        # write_index is the first 8 bytes of the SharedBus struct
        # (first field of the first cache-line-aligned atomic)
        buf = (ctypes.c_uint64 * 1)()
        ctypes.memmove(buf, self._bus, 8)
        return buf[0]

    def read(self) -> BusFrameSlot | None:
        slot   = BusFrameSlot()
        result = _lib.shm_read(
            self._bus,
            ctypes.byref(self._consumer_index),
            ctypes.byref(slot)
        )
        return slot if result == 0 else None

    def available(self) -> int:
        return _lib.shm_available(self._bus, self._consumer_index.value)

    def close(self):
        if self._bus:
            _lib.shm_close(self._bus)
            self._bus = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def make_slot(
    signal,
    frame,
    timestamp_ns: int | None = None,
    brs_index: int = 0
) -> BusFrameSlot:
    slot                = BusFrameSlot()
    slot.timestamp_ns   = timestamp_ns or time.time_ns()
    slot.arbitration_id = frame.arbitration_id
    slot.bit_count      = signal.bit_count
    slot.protocol       = int(frame.protocol)
    slot.is_extended    = int(frame.is_extended)
    slot.is_remote      = int(frame.is_remote)
    slot.brs_index      = brs_index

    canh = signal.canh_bytes
    canl = signal.canl_bytes
    ctypes.memmove(slot.canh_bytes, canh, len(canh))
    ctypes.memmove(slot.canl_bytes, canl, len(canl))

    return slot