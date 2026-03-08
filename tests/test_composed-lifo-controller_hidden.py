"""
Extended cocotb test coverage for the composed LIFO controller.

Tests:
  1. Reset behavior
  2. Status flags during fill
  3. Multiple fill/drain cycles
  4. Packet counter increments
  5. Mid-operation reset
  6. Simultaneous push/pop (write-through)
  7. Parity error injection
"""
from __future__ import annotations

import os
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer, FallingEdge
from cocotb_tools.runner import get_runner


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

async def reset_dut(dut, duration_ns=15):
    """Assert active-low reset for *duration_ns*, then release."""
    dut.rst_n.value = 0
    await Timer(duration_ns, unit="ns")
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)


async def wait_cycles(dut, n):
    """Wait for *n* rising clock edges."""
    for _ in range(n):
        await RisingEdge(dut.clk)


async def collect_drain_data(dut, count, timeout=500):
    """Capture *count* bytes during the LIFO drain phase.

    Returns list[int] of popped data values.
    """
    captured = []
    pop_prev = 0
    cycles = 0

    while len(captured) < count:
        if pop_prev == 1:
            captured.append(int(dut.lifo_data_out.value))
        pop_prev = int(dut.u_lifo.pop.value)
        await RisingEdge(dut.clk)
        cycles += 1
        assert cycles < timeout, f"Timeout after {cycles} cycles, captured {len(captured)}/{count}"

    return captured


# ---------------------------------------------------------------------------
# Test 1 — Reset behavior
# ---------------------------------------------------------------------------

@cocotb.test()
async def test_reset_behavior(dut):
    """After reset: empty=1, full=0, burst_ready=1, parity_err=0, data_out=0."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    # Wait one extra cycle for synchronous outputs to settle
    await RisingEdge(dut.clk)

    assert dut.lifo_empty.value == 1,       "LIFO should be empty after reset"
    assert dut.lifo_full.value == 0,        "LIFO should not be full after reset"
    assert dut.lifo_burst_ready.value == 1, "burst_ready should be high (16 slots free >= 4)"
    assert dut.parity_err.value == 0,       "parity_err should be low after reset"
    assert int(dut.lifo_data_out.value) == 0, "data_out should be 0 after reset"

    dut._log.info("PASS: Reset behavior verified")


# ---------------------------------------------------------------------------
# Test 2 — Status flags during fill
# ---------------------------------------------------------------------------

@cocotb.test()
async def test_status_flags_during_fill(dut):
    """Track empty, full, and burst_ready as the Producer pushes data."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    saw_empty_deassert = False
    saw_burst_ready_deassert = False
    saw_full_assert = False

    for cycle in range(500):
        await RisingEdge(dut.clk)

        sp = int(dut.u_lifo.sp.value)
        empty = int(dut.lifo_empty.value)
        full = int(dut.lifo_full.value)
        burst_ready = int(dut.lifo_burst_ready.value)

        # Track transitions
        if sp > 0 and empty == 0:
            saw_empty_deassert = True

        if sp > 12 and burst_ready == 0:
            saw_burst_ready_deassert = True

        if sp == 16 and full == 1:
            saw_full_assert = True

        # Once full has been seen, no need to continue
        if saw_full_assert:
            break

    assert saw_empty_deassert,       "empty should deassert after the first push"
    assert saw_burst_ready_deassert, "burst_ready should deassert when sp > DEPTH - BURST_SIZE"
    assert saw_full_assert,          "full should assert when sp reaches DEPTH"

    dut._log.info("PASS: Status flags verified during fill phase")


# ---------------------------------------------------------------------------
# Test 3 — Multiple fill/drain cycles
# ---------------------------------------------------------------------------

@cocotb.test()
async def test_multiple_fill_drain_cycles(dut):
    """Run through 2 complete fill→drain cycles and verify packet data."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    PACKETS_PER_BURST = 4  # DEPTH(16) / BURST_SIZE(4)
    BYTES_PER_BURST = 16   # 4 packets × 4 bytes

    for cycle_idx in range(2):
        dut._log.info(f"--- Fill/Drain Cycle {cycle_idx} ---")

        # Build expected sequence for this cycle
        # pkt_cnt starts at cycle_idx * PACKETS_PER_BURST
        base_pkt = cycle_idx * PACKETS_PER_BURST
        expected = []
        for pkt in reversed(range(PACKETS_PER_BURST)):
            pkt_id = base_pkt + pkt
            # LIFO reversal: Footer(0x55), LSB, MSB, Header(0xAA)
            expected.extend([0x55, pkt_id & 0xFF, (pkt_id >> 8) & 0xFF, 0xAA])

        captured = await collect_drain_data(dut, BYTES_PER_BURST)

        dut._log.info(f"Expected: {[hex(x) for x in expected]}")
        dut._log.info(f"Captured: {[hex(x) for x in captured]}")
        assert captured == expected, f"Cycle {cycle_idx}: data mismatch"

    dut._log.info("PASS: Two fill/drain cycles verified")


# ---------------------------------------------------------------------------
# Test 4 — Packet counter increments
# ---------------------------------------------------------------------------

@cocotb.test()
async def test_packet_counter_increments(dut):
    """Monitor pkt_cnt inside the Producer FSM; should reach 4 after first burst."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    assert int(dut.u_producer.pkt_cnt.value) == 0, "pkt_cnt should start at 0"

    prev_cnt = 0
    increments = []

    for cycle in range(500):
        await RisingEdge(dut.clk)
        cnt = int(dut.u_producer.pkt_cnt.value)

        if cnt != prev_cnt:
            increments.append(cnt)
            dut._log.info(f"pkt_cnt incremented to {cnt} at cycle {cycle}")
            prev_cnt = cnt

        # Stop after the first burst completes (4 packets pushed)
        if cnt >= 4:
            break

    assert increments == [1, 2, 3, 4], f"Expected [1,2,3,4], got {increments}"

    dut._log.info("PASS: Packet counter increments verified")


# ---------------------------------------------------------------------------
# Test 5 — Mid-operation reset
# ---------------------------------------------------------------------------

@cocotb.test()
async def test_mid_operation_reset(dut):
    """Assert reset mid-burst and verify clean recovery."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    # Let the FSM push a few bytes (wait ~6 cycles into the first burst)
    await wait_cycles(dut, 6)

    sp_before_reset = int(dut.u_lifo.sp.value)
    dut._log.info(f"SP before mid-operation reset: {sp_before_reset}")
    assert sp_before_reset > 0, "Should have pushed some data before resetting"

    # Assert reset mid-operation
    dut.rst_n.value = 0
    await wait_cycles(dut, 3)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)

    # After reset, LIFO state should be clean
    await RisingEdge(dut.clk)
    assert dut.lifo_empty.value == 1,  "LIFO should be empty after mid-op reset"
    assert dut.parity_err.value == 0,  "parity_err should be 0 after mid-op reset"
    assert int(dut.u_lifo.sp.value) == 0, "Stack pointer should be 0 after reset"

    # Verify the FSM recovers — it should start a new fill cycle
    # Wait enough cycles for at least one push
    saw_push = False
    for _ in range(20):
        await RisingEdge(dut.clk)
        if int(dut.u_lifo.push.value) == 1:
            saw_push = True
            break

    assert saw_push, "FSM should resume pushing after reset recovery"

    dut._log.info("PASS: Mid-operation reset recovery verified")


# ---------------------------------------------------------------------------
# Test 6 — Simultaneous push/pop (write-through path)
# ---------------------------------------------------------------------------

@cocotb.test()
async def test_simultaneous_push_pop(dut):
    """Exercise the LIFO write-through path by forcing push+pop simultaneously."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    # Let the FSM push some data so the LIFO isn't empty
    # Wait for the first burst to complete (sp should reach at least 4)
    for _ in range(30):
        await RisingEdge(dut.clk)
        if int(dut.u_lifo.sp.value) >= 4:
            break

    sp_before = int(dut.u_lifo.sp.value)
    dut._log.info(f"SP before simultaneous push/pop: {sp_before}")
    assert sp_before > 0, "LIFO must have data for simultaneous push/pop test"

    # Force simultaneous push + pop with known data
    test_byte = 0xBE
    dut.u_lifo.push.value = 1
    dut.u_lifo.pop.value = 1
    dut.u_lifo.data_in.value = test_byte

    await RisingEdge(dut.clk)

    # Release forced signals
    dut.u_lifo.push.value = 0
    dut.u_lifo.pop.value = 0

    # After simultaneous push+pop, sp should remain unchanged
    await RisingEdge(dut.clk)
    sp_after = int(dut.u_lifo.sp.value)

    # Write-through: data_out should be the new data we pushed
    data_after = int(dut.lifo_data_out.value)

    dut._log.info(f"SP after: {sp_after} (expected: {sp_before})")
    dut._log.info(f"data_out after: {hex(data_after)} (expected: {hex(test_byte)})")

    assert sp_after == sp_before, \
        f"SP changed during simultaneous push/pop: {sp_before} -> {sp_after}"
    assert data_after == test_byte, \
        f"Write-through failed: expected {hex(test_byte)}, got {hex(data_after)}"
    assert dut.parity_err.value == 0, "parity_err should be 0 for write-through"

    dut._log.info("PASS: Simultaneous push/pop write-through verified")


# ---------------------------------------------------------------------------
# Test 7 — Parity error injection
# ---------------------------------------------------------------------------

@cocotb.test()
async def test_parity_error_injection(dut):
    """Corrupt a stored word and verify parity_err asserts on read."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    # Let the FSM fill some data
    for _ in range(30):
        await RisingEdge(dut.clk)
        if int(dut.u_lifo.sp.value) >= 4:
            break

    sp = int(dut.u_lifo.sp.value)
    dut._log.info(f"SP after partial fill: {sp}")
    assert sp >= 1, "Need at least one entry to corrupt"

    # Read the current top-of-stack word (data + parity bit)
    corrupt_idx = sp - 1
    original_word = int(dut.u_lifo.mem[corrupt_idx].value)
    dut._log.info(f"Original mem[{corrupt_idx}] = {hex(original_word)} (9 bits: parity + data)")

    # Flip the parity bit (MSB of the 9-bit word) to create a mismatch
    corrupted_word = original_word ^ (1 << 8)  # flip bit 8 (the parity bit)
    dut._log.info(f"Corrupted mem[{corrupt_idx}] = {hex(corrupted_word)}")
    dut.u_lifo.mem[corrupt_idx].value = corrupted_word

    # Now force a pop to read the corrupted entry
    # We need to ensure a pop happens on the corrupted index
    # Stop the FSM's normal push by holding it, then force a pop
    dut.u_lifo.push.value = 0
    dut.u_lifo.pop.value = 1

    await RisingEdge(dut.clk)  # Pop is registered
    dut.u_lifo.pop.value = 0

    await RisingEdge(dut.clk)  # parity_err should be available now

    parity_val = int(dut.parity_err.value)
    dut._log.info(f"parity_err = {parity_val} (expected: 1)")

    assert parity_val == 1, "parity_err should assert when reading a corrupted word"

    dut._log.info("PASS: Parity error injection detected correctly")


# ---------------------------------------------------------------------------
# Pytest runner
# ---------------------------------------------------------------------------

def test_coverage_runner():
    """Pytest entry point — builds the design and runs all cocotb tests in this file."""
    sim = os.getenv("SIM", "icarus")

    proj_path = Path(__file__).resolve().parent.parent

    sources = [
        proj_path / "golden" / "top_composed.sv",
        proj_path / "golden" / "lifo.sv",
        proj_path / "golden" / "producer_fsm.sv",
    ]

    runner = get_runner(sim)
    runner.build(
        sources=sources,
        hdl_toplevel="top_composed_lifo",
        always=True,
    )
    runner.test(
        hdl_toplevel="top_composed_lifo",
        test_module="test_coverage",
    )
