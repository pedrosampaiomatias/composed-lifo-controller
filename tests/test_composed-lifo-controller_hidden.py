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

    Watches the producer's pop output (via the LIFO's pop port)
    and captures data_out one cycle after each pop assertion.

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


async def wait_for_push_activity(dut, timeout=30):
    """Wait until we see push=1 on the LIFO, or timeout."""
    for _ in range(timeout):
        await RisingEdge(dut.clk)
        if int(dut.u_lifo.push.value) == 1:
            return True
    return False


async def count_pushes_until_drain(dut, timeout=100):
    """Count pushes until drain phase starts. Returns (push_count, timed_out)."""
    push_count = 0
    for _ in range(timeout):
        await RisingEdge(dut.clk)
        if int(dut.u_lifo.push.value) == 1:
            push_count += 1
        elif int(dut.u_lifo.pop.value) == 1:
            return push_count, False
    return push_count, True


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

    # Check status outputs (registered — valid after first edge)
    assert dut.lifo_empty.value == 1,       "LIFO should be empty after reset"
    assert dut.lifo_full.value == 0,        "LIFO should not be full after reset"
    assert dut.parity_err.value == 0,       "parity_err should be low after reset"
    assert int(dut.lifo_data_out.value) == 0, "data_out should be 0 after reset"

    # Verify the system actually starts pushing after reset,
    # which proves credits were issued and consumed.
    saw_push = await wait_for_push_activity(dut, timeout=10)
    assert saw_push, "Producer should start pushing shortly after reset (credits available)"

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
    """Verify the packet counter wraps from 999 → 0 by observing pushed payload bytes."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    # Strategy: observe pushed bytes to extract packet counter values.
    # Each packet is [0xAA, counter_hi, counter_lo, CRC].
    # We track the counter values and verify wrap at 1000.
    
    PACKETS_PER_BURST = 4   # DEPTH(16) / BURST_SIZE(4) = 4 packets per fill
    BYTES_PER_PACKET = 4
    TARGET_PACKETS = 1005   # Enough to pass the 999→0 wrap point
    
    # We'll observe packets by capturing push data
    packet_counters = []
    current_packet = []
    
    # Maximum cycles: ~1005 packets * 4 bytes + drain overhead
    # Each fill/drain cycle: 16 push cycles + ~16 drain cycles = 32
    # 1005/4 = ~252 fill/drain cycles * 32 = ~8064 cycles
    MAX_CYCLES = 12000

    for cycle in range(MAX_CYCLES):
        await RisingEdge(dut.clk)

        if int(dut.u_lifo.push.value) == 1:
            current_packet.append(int(dut.u_producer.data_out.value))
            
            if len(current_packet) == BYTES_PER_PACKET:
                # Extract counter from payload bytes
                counter_val = (current_packet[1] << 8) | current_packet[2]
                packet_counters.append(counter_val)
                current_packet = []

        if len(packet_counters) >= TARGET_PACKETS:
            break

    assert len(packet_counters) >= TARGET_PACKETS, \
        f"Only captured {len(packet_counters)} packets, needed {TARGET_PACKETS}"

    # Verify first few counters
    assert packet_counters[0] == 0, f"First counter should be 0, got {packet_counters[0]}"
    for i in range(1, min(10, len(packet_counters))):
        assert packet_counters[i] == i, \
            f"Counter[{i}] should be {i}, got {packet_counters[i]}"

    # Find the wrap point: should go 998 → 999 → 0
    wrap_found = False
    for i in range(1, len(packet_counters)):
        if packet_counters[i - 1] == 999 and packet_counters[i] == 0:
            wrap_found = True
            dut._log.info(f"Wrap observed at packet index {i}: 999 → 0")
            break

    assert wrap_found, \
        f"Counter should wrap from 999→0. Last 5 counters: {packet_counters[-5:]}"

    # Verify the counter after wrap continues from 0
    wrap_idx = packet_counters.index(0, 999)  # Find the 0 after position 999
    for j in range(min(5, len(packet_counters) - wrap_idx)):
        assert packet_counters[wrap_idx + j] == j, \
            f"Post-wrap counter[{j}] should be {j}, got {packet_counters[wrap_idx + j]}"

    dut._log.info(f"PASS: Counter wrap verified over {len(packet_counters)} packets")


# ---------------------------------------------------------------------------
# Test 6 — Mid-operation reset
# ---------------------------------------------------------------------------

@cocotb.test()
async def test_mid_operation_reset(dut):
    """Assert reset mid-burst and verify clean recovery."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    # Let the FSM push a few bytes (wait for at least one push)
    saw_push = await wait_for_push_activity(dut, timeout=30)
    assert saw_push, "Should have seen push activity before mid-operation reset"

    # Wait a couple more cycles to get into the middle of a packet
    await wait_cycles(dut, 2)

    # Assert reset mid-operation
    dut.rst_n.value = 0
    await wait_cycles(dut, 3)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)

    # After reset, all state should be clean
    await RisingEdge(dut.clk)
    assert dut.lifo_empty.value == 1,  "LIFO should be empty after mid-op reset"
    assert dut.parity_err.value == 0,  "parity_err should be 0 after mid-op reset"

    # Verify the FSM recovers — it should start a new fill cycle
    # (which proves credits were re-issued after reset)
    saw_push = await wait_for_push_activity(dut, timeout=30)
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

    # Let the FSM push some data so the LIFO isn't empty.
    # Wait until LIFO is no longer empty (observed via top-level lifo_empty).
    for _ in range(30):
        await RisingEdge(dut.clk)
        if int(dut.lifo_empty.value) == 0:
            break

    assert int(dut.lifo_empty.value) == 0, "LIFO must have data for simultaneous push/pop test"

    # Count how many items are in the LIFO by tracking pushes
    # (we know LIFO wasn't empty, so at least 1 item)
    was_not_empty = True

    # Force simultaneous push + pop with known data via sub-module ports.
    # These port names (push, pop, data_in) are specified in the prompt interface.
    test_byte = 0xBE
    dut.u_lifo.push.value = 1
    dut.u_lifo.pop.value = 1
    dut.u_lifo.data_in.value = test_byte

    await RisingEdge(dut.clk)

    # Release forced signals
    dut.u_lifo.push.value = 0
    dut.u_lifo.pop.value = 0

    # After simultaneous push+pop:
    # - The LIFO should still not be empty (write-through doesn't change SP)
    # - data_out should reflect the write-through data
    await RisingEdge(dut.clk)

    # Write-through: data_out should be the new data we pushed
    data_after = int(dut.lifo_data_out.value)

    dut._log.info(f"data_out after: {hex(data_after)} (expected: {hex(test_byte)})")
    dut._log.info(f"lifo_empty: {int(dut.lifo_empty.value)} (expected: 0)")

    assert int(dut.lifo_empty.value) == 0, \
        "LIFO should still not be empty after write-through (SP unchanged)"
    assert data_after == test_byte, \
        f"Write-through failed: expected {hex(test_byte)}, got {hex(data_after)}"
    assert dut.parity_err.value == 0, "parity_err should be 0 for write-through"

    dut._log.info("PASS: Simultaneous push/pop write-through verified")


# ---------------------------------------------------------------------------
# Pytest runner
# ---------------------------------------------------------------------------

def test_coverage_runner():
    """Pytest entry point — builds the design and runs all cocotb tests in this file."""
    sim = os.getenv("SIM", "icarus")

    proj_path = Path(__file__).resolve().parent.parent

    sources = [
        proj_path / "sources" / "top_composed.sv",
        proj_path / "sources" / "lifo.sv",
        proj_path / "sources" / "producer_fsm.sv",
    ]

    runner = get_runner(sim)
    runner.build(
        sources=sources,
        hdl_toplevel="top_composed_lifo",
        always=True,
    )
    runner.test(
        hdl_toplevel="top_composed_lifo",
        test_module="test_composed-lifo-controller_hidden",
    )
