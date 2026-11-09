# Review notes: eth/v1 interrupt storm fix (drop this commit before opening the PR)

Branch `eth-v1-clear-abnormal-irq` = `origin/main` (ceaab81) + one fix commit.
The same fix on the free-wheel pinned rev is `freewheel/eth-v1-clear-abnormal-irq-5f0e963` (85d08a0);
free-wheel `main` consumes that one through a Cargo `[patch]`.

## Symptom

CH32V307, `eth::v1` with the internal 10M PHY. Under a UDP flood (about 10 kHz of small datagrams,
sometimes after sustained 5 kHz) the device stops answering ARP and never recovers until reset. Halting the core
shows it alive, with every PC sample inside `ETH` / `__qingke_rt_ETH` (mcause = IRQ 77) and only the
higher-priority timer interrupt preempting it. Thread-mode code (the executor, embassy-net) never runs.

## Root cause

- `Ethernet::new` enables in `DMAIER`: `tie`, `rie`, `nise`, **`aise`**, **`rbuie`**.
- `on_interrupt()` write-1-clears only `TS`, `RS` and `NIS` in `DMASR`.
- When the RX ring fills (the stack cannot drain it as fast as frames arrive), the DMA sets **RBUS**
  (receive buffer unavailable) and the **AIS** summary. Nothing ever clears them, so the interrupt line
  stays asserted and `ETH` re-enters forever.

## The change (src/eth/v1/mod.rs, +14/-8)

1. `on_interrupt()` reads `DMASR` once and write-1-clears every W1C status bit that is set:
   TS0 TPSS1 TBUS2 TJTS3 ROS4 TUS5 RS6 RBUS7 RPSS8 RWTS9 ETS10 FBES13 ERS14 AIS15 NIS16
   (mask `0x1E7FF`). The read-only state fields (PLS, EBS, TPS, RPS and so on) are not written. It
   still wakes the waker and keeps the two dummy reads.
2. `new()` no longer enables `aise` and `rbuie`. While the ring is full, RBUS re-sets on every dropped
   frame, so RBUIE would cost one interrupt per flood packet and do no useful work. RX already resumes
   without it: `pop_packet` issues a receive poll demand (`DMARPDR`) on every descriptor recycle, and
   `available()` poll-demands when `DMASR.RPS` reads "suspended". TX is unchanged: `transmit()` writes
   `DMATPDR` on every send.

Bit positions: CH32V307 RM V2.5, Ethernet DMA status register (section 27.1.8.4.6, pp. 543-546), and
`ETHERNET_DMA` / `DMASR` in `ch32-data/CH32V307xx.svd`. The layout matches the STM32F107's.

## Points a reviewer may raise

- **Disabling AISE also silences fatal-bus-error (FBES) interrupts.** If upstream prefers to keep AISE
  for error visibility, the minimal alternative is to keep `aise` and drop only `rbuie`. The W1C clear
  in (1) alone already stops the storm. The cost is one interrupt per dropped frame during a flood.
  Either way (1) is the essential part.
- **Clearing bits that also need handling elsewhere:** none of the bits cleared here is consumed by
  other code in the driver; they were simply never cleared.
- **Other ETH variants:** `v10m` already clears all of `EIR` (`write_value(flags)`); `h4` / `v4x7`
  were not checked as part of this change.

## Evidence

- free-wheel firmware (embassy-net 0.9, smoltcp 0.13), internal 10M PHY, 10 Mb/s full duplex:
  - stock driver: hung twice, once under a 10 kHz flood and once after sustained 5 kHz;
  - patched: 3 x 10 s of 10 kHz junk UDP plus a 5 kHz telemetry TX stream; the device answered
    ping within 0.01 s after each flood;
  - an instrumented copy of the driver counted about 2 ETH interrupts per flood packet (bounded),
    and about 97 % of flood datagrams were delivered to the socket.
- `examples/ch32v307` `eth` builds on this branch.

## Suggested PR

Title: `fix(eth/v1): clear all DMASR status bits; avoid RBU/AIS interrupt storm under RX flood`

Body: the root cause and change sections above, plus the evidence, trimmed.
