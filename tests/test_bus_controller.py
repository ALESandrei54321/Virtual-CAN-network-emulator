# tests/test_bus_controller.py

import asyncio
import pytest
import pytest_asyncio

from bus_broker.core.frames import CANFrame, Protocol
from bus_broker.core.bus_controller import BusController, BusStats, PendingFrame
from bus_broker.protocols.can_protocol import CANProtocol
from bus_broker.protocols.can_fd_protocol import CANFDProtocol
from bus_broker.transport.shm_writer import SHMBusWriter, SHMBusReader


# ── Fixtures ──────────────────────────────────────────────────────────────────

@pytest.fixture
def protocol():
    return CANProtocol()

@pytest.fixture
def writer():
    w = SHMBusWriter()
    yield w
    w.close()

@pytest.fixture
def reader(writer):
    r = SHMBusReader()
    yield r
    r.close()

@pytest.fixture
def controller(protocol, writer):
    return BusController(protocol, writer)

@pytest.fixture
def std_frame():
    return CANFrame(
        arbitration_id = 0x123,
        dlc            = 4,
        data           = bytes([0xDE, 0xAD, 0xBE, 0xEF])
    )

@pytest.fixture
def low_priority_frame():
    return CANFrame(arbitration_id=0x7FF, dlc=0, data=b"")

@pytest.fixture
def high_priority_frame():
    return CANFrame(arbitration_id=0x001, dlc=0, data=b"")


# ── BusStats ──────────────────────────────────────────────────────────────────

def test_stats_initial_values():
    s = BusStats()
    assert s.frames_sent        == 0
    assert s.frames_dropped     == 0
    assert s.arbitration_events == 0
    assert s.avg_latency_ns     == 0.0

def test_stats_avg_latency():
    s = BusStats()
    s.frames_sent = 2
    s.total_latency_ns = 1000
    assert s.avg_latency_ns == 500.0

def test_stats_record_latency():
    s = BusStats()
    s.record_latency(100)
    s.record_latency(200)
    assert s.min_latency_ns == 100
    assert s.max_latency_ns == 200
    assert s.total_latency_ns == 300

def test_stats_repr():
    s = BusStats()
    r = repr(s)
    assert "sent=" in r
    assert "dropped=" in r


# ── PendingFrame ──────────────────────────────────────────────────────────────

def test_pending_frame_priority(protocol, std_frame):
    p = PendingFrame(frame=std_frame)
    assert p.priority(protocol) == std_frame.arbitration_id

def test_pending_frame_initial_retries(std_frame):
    p = PendingFrame(frame=std_frame)
    assert p.retries == 0

def test_pending_frame_has_timestamp(std_frame):
    p = PendingFrame(frame=std_frame)
    assert p.submitted_ns > 0


# ── Submit ────────────────────────────────────────────────────────────────────

@pytest.mark.asyncio
async def test_submit_valid_frame(controller, std_frame):
    await controller.submit(std_frame)
    assert len(controller._queue) == 1

@pytest.mark.asyncio
async def test_submit_wrong_protocol_raises(controller):
    fd_frame = CANFrame(
        arbitration_id = 0x200,
        dlc            = 9,
        data           = bytes(12),
        protocol       = Protocol.CAN_FD
    )
    with pytest.raises(ValueError, match="rejected"):
        await controller.submit(fd_frame)

@pytest.mark.asyncio
async def test_submit_multiple_frames(controller, std_frame, low_priority_frame):
    await controller.submit(std_frame)
    await controller.submit(low_priority_frame)
    assert len(controller._queue) == 2


# ── Transmit ──────────────────────────────────────────────────────────────────

@pytest.mark.asyncio
async def test_single_frame_transmitted(controller, reader, std_frame):
    await controller.submit(std_frame)
    await controller._tick()
    assert reader.available() == 1

@pytest.mark.asyncio
async def test_transmitted_frame_has_correct_id(controller, reader, std_frame):
    await controller.submit(std_frame)
    await controller._tick()
    slot = reader.read()
    assert slot.arbitration_id == std_frame.arbitration_id

@pytest.mark.asyncio
async def test_no_frame_no_transmit(controller, reader):
    await controller._tick()
    assert reader.available() == 0

@pytest.mark.asyncio
async def test_stats_updated_after_transmit(controller, std_frame):
    await controller.submit(std_frame)
    await controller._tick()
    assert controller.stats.frames_sent == 1

@pytest.mark.asyncio
async def test_latency_recorded_after_transmit(controller, std_frame):
    await controller.submit(std_frame)
    await controller._tick()
    assert controller.stats.total_latency_ns > 0


# ── Arbitration ───────────────────────────────────────────────────────────────

@pytest.mark.asyncio
async def test_lower_id_wins_arbitration(
    controller, reader, high_priority_frame, low_priority_frame
):
    await controller.submit(low_priority_frame)
    await controller.submit(high_priority_frame)
    await controller._tick()

    slot = reader.read()
    assert slot.arbitration_id == high_priority_frame.arbitration_id

@pytest.mark.asyncio
async def test_loser_requeued_after_arbitration(
    controller, high_priority_frame, low_priority_frame
):
    await controller.submit(low_priority_frame)
    await controller.submit(high_priority_frame)
    await controller._tick()

    # Loser should be back in the queue
    assert len(controller._queue) == 1
    assert controller._queue[0].frame.arbitration_id == \
           low_priority_frame.arbitration_id

@pytest.mark.asyncio
async def test_loser_retry_count_incremented(
    controller, high_priority_frame, low_priority_frame
):
    await controller.submit(low_priority_frame)
    await controller.submit(high_priority_frame)
    await controller._tick()

    assert controller._queue[0].retries == 1

@pytest.mark.asyncio
async def test_arbitration_event_counted(
    controller, high_priority_frame, low_priority_frame
):
    await controller.submit(low_priority_frame)
    await controller.submit(high_priority_frame)
    await controller._tick()

    assert controller.stats.arbitration_events == 1

@pytest.mark.asyncio
async def test_arbitration_not_counted_for_single_frame(
    controller, std_frame
):
    await controller.submit(std_frame)
    await controller._tick()
    assert controller.stats.arbitration_events == 0

@pytest.mark.asyncio
async def test_loser_transmitted_on_next_tick(
    controller, reader, high_priority_frame, low_priority_frame
):
    await controller.submit(low_priority_frame)
    await controller.submit(high_priority_frame)

    await controller._tick()   # high priority wins
    await controller._tick()   # low priority gets through

    assert reader.available() == 2


# ── Callback ──────────────────────────────────────────────────────────────────

@pytest.mark.asyncio
async def test_callback_called_on_transmit(protocol, writer, std_frame):
    received = []
    controller = BusController(
        protocol,
        writer,
        on_frame_sent=lambda f: received.append(f)
    )
    await controller.submit(std_frame)
    await controller._tick()
    assert len(received) == 1
    assert received[0].arbitration_id == std_frame.arbitration_id

@pytest.mark.asyncio
async def test_callback_not_called_when_no_frame(protocol, writer):
    received = []
    controller = BusController(
        protocol,
        writer,
        on_frame_sent=lambda f: received.append(f)
    )
    await controller._tick()
    assert len(received) == 0


# ── Run / stop ────────────────────────────────────────────────────────────────

@pytest.mark.asyncio
async def test_run_and_stop(controller, std_frame):
    await controller.submit(std_frame)

    # Run for a very short time then stop
    async def stop_soon():
        await asyncio.sleep(0.01)
        controller.stop()

    await asyncio.gather(
        controller.run(),
        stop_soon()
    )

    assert controller.stats.frames_sent >= 1

@pytest.mark.asyncio
async def test_context_manager(protocol, writer, std_frame):
    async with BusController(protocol, writer) as ctrl:
        await ctrl.submit(std_frame)
        await ctrl._tick()
    assert ctrl.stats.frames_sent == 1
