"""
Extended cocotb test coverage for the composed LIFO controller with:
  - CRC-8/MAXIM footer verification
  - Credit-based flow control
  - Packet counter wrapping at 1000

Tests:
  1. Reset behavior (including credit signals)
  2. Credit issuance and tracking
  3. CRC-8 footer verification (single burst)
  4. Multiple fill/drain cycles with CRC validation
  5. Packet counter wrap at 1000
  6. Mid-operation reset recovery
  7. Simultaneous push/pop (write-through)
  8. Parity error injection
"""
from __future__ import annotations

import os
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer, FallingEdge
from cocotb_tools.runner import get_runner


# ---------------------------------------------------------------------------
# CRC-8/MAXIM reference implementation (polynomial 0x31, init 0x00)
# ---------------------------------------------------------------------------

def crc8_maxim(data_bytes: list[int], init: int = 0x00) -> int:
    """Compute CRC-8/MAXIM over a sequence of bytes."""
    crc = init
    for byte in data_bytes:
        for i in range(7, -1, -1):
            bit = (byte >> i) & 1
            if (crc >> 7) ^ bit:
                crc = ((crc << 1) & 0xFF) ^ 0x31
            else:
                crc = (crc << 1) & 0xFF
    return crc


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


def build_expected_packet(pkt_id: int) -> list[int]:
    """Build a 4-byte packet: [Header, PayloadHi, PayloadLo, CRC-8]."""
    hdr = 0xAA
    phi = (pkt_id >> 8) & 0xFF
    plo = pkt_id & 0xFF
    crc = crc8_maxim([hdr, phi, plo])
    return [hdr, phi, plo, crc]


# ---------------------------------------------------------------------------
# Test 1 — Reset behavior
# ---------------------------------------------------------------------------

@cocotb.test()
async def test_reset_behavior(dut):
    """After reset: empty=1, full=0, parity_err=0, data_out=0, credit signals init."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    await RisingEdge(dut.clk)

    assert dut.lifo_empty.value == 1,       "LIFO should be empty after reset"
    assert dut.lifo_full.value == 0,        "LIFO should not be full after reset"
    assert dut.parity_err.value == 0,       "parity_err should be low after reset"
    assert int(dut.lifo_data_out.value) == 0, "data_out should be 0 after reset"

    # After reset, the LIFO immediately issues BURST_SIZE credits
    # (combinational output), so the producer latches them on the first rising edge.
    # By the time we check (1 extra cycle after reset), credit_balance == BURST_SIZE.
    assert int(dut.u_producer.credit_balance.value) == 4, \
        "credit_balance should be BURST_SIZE (4) after reset + 1 cycle"

    dut._log.info("PASS: Reset behavior verified")


# ---------------------------------------------------------------------------
# Test 2 — Credit issuance and tracking
# ---------------------------------------------------------------------------

@cocotb.test()
async def test_credit_issuance(dut):
    """Verify the LIFO issues credits correctly and the producer tracks them."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    saw_credit_valid = False
    credit_grants = []
    push_count = 0

    for cycle in range(100):
        await RisingEdge(dut.clk)

        cv = int(dut.credit_valid.value)
        ca = int(dut.credit_amount.value)
        push = int(dut.u_lifo.push.value)

        if cv == 1:
            saw_credit_valid = True
            credit_grants.append(ca)
            dut._log.info(f"Cycle {cycle}: credit_valid pulse, amount={ca}")

        if push == 1:
            push_count += 1

        # After enough data to track a few grants, stop
        if push_count >= 16:
            break

    assert saw_credit_valid, "credit_valid should have pulsed at least once"
    assert all(g == 4 for g in credit_grants), \
        f"All credit grants should be BURST_SIZE=4, got {credit_grants}"

    # The total observed credits may be less than push_count because the first
    # grant happens at reset release (before our loop starts).  Verify that
    # at least 3 grants of BURST_SIZE were observed (out of 4 expected).
    total_credits = sum(credit_grants)
    assert len(credit_grants) >= 3, \
        f"Expected at least 3 credit grants, got {len(credit_grants)}"
    assert total_credits == len(credit_grants) * 4, \
        f"Each grant should be BURST_SIZE=4, total {total_credits} from {len(credit_grants)} grants"

    dut._log.info(f"PASS: Credit issuance verified ({len(credit_grants)} grants, {push_count} pushes)")


# ---------------------------------------------------------------------------
# Test 3 — CRC-8 footer verification (single burst)
# ---------------------------------------------------------------------------

@cocotb.test()
async def test_crc8_footer_single_burst(dut):
    """Capture a single 4-byte packet and verify the CRC-8 footer."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    # Capture push data for the first packet
    pushed_bytes = []
    for cycle in range(50):
        await RisingEdge(dut.clk)

        if int(dut.u_lifo.push.value) == 1:
            pushed_bytes.append(int(dut.u_producer.data_out.value))

        if len(pushed_bytes) >= 4:
            break

    assert len(pushed_bytes) >= 4, f"Expected at least 4 pushed bytes, got {len(pushed_bytes)}"

    # First packet: header=0xAA, payload_hi, payload_lo, crc
    pkt = pushed_bytes[:4]
    dut._log.info(f"Captured packet: {[hex(b) for b in pkt]}")

    assert pkt[0] == 0xAA, f"Header should be 0xAA, got {hex(pkt[0])}"

    # Verify CRC: compute over first 3 bytes, should match 4th
    expected_crc = crc8_maxim(pkt[:3])
    assert pkt[3] == expected_crc, \
        f"CRC mismatch: expected {hex(expected_crc)}, got {hex(pkt[3])}"

    dut._log.info(f"PASS: CRC-8 footer verified (CRC={hex(expected_crc)})")


# ---------------------------------------------------------------------------
# Test 4 — Multiple fill/drain cycles with CRC validation
# ---------------------------------------------------------------------------

@cocotb.test()
async def test_multiple_fill_drain_cycles(dut):
    """Run through 2 complete fill→drain cycles and verify packet data with CRC."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    PACKETS_PER_BURST = 4   # DEPTH(16) / BURST_SIZE(4)
    BYTES_PER_BURST = 16    # 4 packets × 4 bytes

    for cycle_idx in range(2):
        dut._log.info(f"--- Fill/Drain Cycle {cycle_idx} ---")

        base_pkt = cycle_idx * PACKETS_PER_BURST
        expected = []
        for pkt in reversed(range(PACKETS_PER_BURST)):
            pkt_id = base_pkt + pkt
            pkt_bytes = build_expected_packet(pkt_id)
            # LIFO reversal: Footer(CRC), LSB, MSB, Header(0xAA)
            expected.extend(reversed(pkt_bytes))

        captured = await collect_drain_data(dut, BYTES_PER_BURST)

        dut._log.info(f"Expected: {[hex(x) for x in expected]}")
        dut._log.info(f"Captured: {[hex(x) for x in captured]}")
        assert captured == expected, f"Cycle {cycle_idx}: data mismatch"

    dut._log.info("PASS: Two fill/drain cycles with CRC verified")


# ---------------------------------------------------------------------------
# Test 5 — Packet counter wrap at 1000
# ---------------------------------------------------------------------------

@cocotb.test()
async def test_packet_counter_wrap(dut):
    """Verify the packet counter wraps from 999 → 0."""
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

        # Stop after first burst (4 packets)
        if cnt >= 4:
            break

    assert increments == [1, 2, 3, 4], f"Expected [1,2,3,4], got {increments}"

    # Now force the counter near the wrap point to test wrapping
    # Set pkt_cnt to 998 and let 2 more packets push through
    dut.u_producer.pkt_cnt.value = 998
    await RisingEdge(dut.clk)

    # Wait for a full fill/drain cycle to observe wrap
    prev_cnt = 998
    wrap_observed = False

    for cycle in range(500):
        await RisingEdge(dut.clk)
        cnt = int(dut.u_producer.pkt_cnt.value)

        if cnt != prev_cnt:
            dut._log.info(f"pkt_cnt changed: {prev_cnt} -> {cnt}")
            if prev_cnt == 999 and cnt == 0:
                wrap_observed = True
            prev_cnt = cnt

        if wrap_observed:
            break

    assert wrap_observed, "pkt_cnt should wrap from 999 to 0"

    dut._log.info("PASS: Packet counter wrap at 1000 verified")


# ---------------------------------------------------------------------------
# Test 6 — Mid-operation reset
# ---------------------------------------------------------------------------

@cocotb.test()
async def test_mid_operation_reset(dut):
    """Assert reset mid-burst and verify clean recovery."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    # Let the FSM push a few bytes
    await wait_cycles(dut, 6)

    sp_before_reset = int(dut.u_lifo.sp.value)
    dut._log.info(f"SP before mid-operation reset: {sp_before_reset}")
    assert sp_before_reset > 0, "Should have pushed some data before resetting"

    # Assert reset mid-operation
    dut.rst_n.value = 0
    await wait_cycles(dut, 3)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)

    # After reset, all state should be clean
    await RisingEdge(dut.clk)
    assert dut.lifo_empty.value == 1,  "LIFO should be empty after mid-op reset"
    assert dut.parity_err.value == 0,  "parity_err should be 0 after mid-op reset"
    assert int(dut.u_lifo.sp.value) == 0, "Stack pointer should be 0 after reset"
    # After reset, LIFO immediately issues credits (combinational),
    # so credit_balance == BURST_SIZE by the time we check.
    assert int(dut.u_producer.credit_balance.value) == 4, \
        "credit_balance should be BURST_SIZE (4) after reset recovery"

    # Verify the FSM recovers — it should start a new fill cycle
    saw_push = False
    for _ in range(30):
        await RisingEdge(dut.clk)
        if int(dut.u_lifo.push.value) == 1:
            saw_push = True
            break

    assert saw_push, "FSM should resume pushing after reset recovery"

    dut._log.info("PASS: Mid-operation reset recovery verified")


# ---------------------------------------------------------------------------
# Test 7 — Simultaneous push/pop (write-through path)
# ---------------------------------------------------------------------------

@cocotb.test()
async def test_simultaneous_push_pop(dut):
    """Exercise the LIFO write-through path by forcing push+pop simultaneously."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    # Let the FSM push some data so the LIFO isn't empty
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
# Test 8 — Parity error injection
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
    corrupted_word = original_word ^ (1 << 8)
    dut._log.info(f"Corrupted mem[{corrupt_idx}] = {hex(corrupted_word)}")
    dut.u_lifo.mem[corrupt_idx].value = corrupted_word

    # Force a pop to read the corrupted entry
    dut.u_lifo.push.value = 0
    dut.u_lifo.pop.value = 1

    await RisingEdge(dut.clk)
    dut.u_lifo.pop.value = 0

    await RisingEdge(dut.clk)

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
        test_module="test_composed_lifo_controller_hidden",
    )
