# RC Snubber Network Specification

> **Applies to:** Final spot welder power stage schematic (PCB layout release)
> 
> **Network location:** Between **PACK_POS** and **PACK_DRAIN**
> 
> **Schematic refs:** **C19, C17, C20, C18** with corresponding **R8, R3, R9, R7**

---

## 1) Hardware Overview

The weld bus includes a **4× parallel RC snubber network** to reduce high-frequency ringing caused by fast current transitions in the low-inductance, high-current MOSFET stage.

Each branch is:

- 1× polypropylene film capacitor (0.56 µF, 400 V)
- 1× series power resistor (2.2 Ω, 6 W thick film)

Total network: **4 identical RC branches in parallel**, connected from PACK_POS to PACK_DRAIN.

---

## 2) Component Specification (Per Branch)

### 2.1 Capacitor

- **Manufacturer:** KEMET
- **Part number:** **F461BY564J400A**
- **Technology:** Polypropylene film (pulse-capable)
- **Capacitance:** 0.56 µF
- **Voltage rating:** 400 VDC
- **Tolerance:** ±5%

### 2.2 Resistor

- **Manufacturer:** TE Connectivity
- **Part:** 2.2 Ω, 6 W thick film power resistor (project-approved TE part)
- **Resistance:** 2.2 Ω
- **Power rating:** 6 W
- **Function:** Damps oscillation and controls capacitor discharge current

---

## 3) Electrical Characteristics

Using per-branch nominal values:

- \(R = 2.2\ \Omega\)
- \(C = 0.56\ \mu F\)

### 3.1 Time constant

\[
\tau = R \cdot C = 2.2 \times 0.56\times10^{-6} = 1.232\times10^{-6}\ s \approx \mathbf{1.23\ \mu s}
\]

### 3.2 Corner frequency

\[
f_c = \frac{1}{2\pi RC} = \frac{1}{2\pi\cdot2.2\cdot0.56\times10^{-6}} \approx \mathbf{129\ kHz}
\]

This targets damping in the ringing band while minimally affecting low-frequency weld/charge behavior.

---

## 4) Performance and Safety Calculations

The calculations below use a representative bus level of **19.5 V**.

### 4.1 Energy absorbed per switching event (per branch)

\[
E_{branch} = \frac{1}{2}CV^2 = 0.5\cdot0.56\times10^{-6}\cdot(19.5)^2
\approx 1.0647\times10^{-4}\ J = \mathbf{0.106\ mJ}
\]

Total for four branches:

\[
E_{total} = 4\cdot E_{branch} \approx \mathbf{0.426\ mJ/event}
\]

### 4.2 Peak branch current and initial resistor pulse power

\[
I_{pk} = \frac{V}{R} = \frac{19.5}{2.2} \approx \mathbf{8.86\ A}
\]

\[
P_{0} = \frac{V^2}{R} = \frac{(19.5)^2}{2.2} \approx \mathbf{173\ W}
\]

This is a short exponential pulse (time constant 1.23 µs), not continuous dissipation.

### 4.3 Average branch dissipation vs repetition rate

\[
P_{avg,branch} = E_{branch}\cdot f
\]

- At **10 Hz**: \(P_{avg}\approx 1.06\ mW\)
- At **100 Hz**: \(P_{avg}\approx 10.6\ mW\)

Both are far below 6 W resistor continuous rating.

### 4.4 Safety margin snapshot

- **Capacitor voltage margin** at 19.5 V:
  - \(400/19.5 \approx 20.5\times\)
- **Resistor average power margin** at 100 Hz:
  - \(6 W / 0.0106 W \approx 566\times\)

Result: substantial margin for normal operating envelope.

---

## 5) Placement and Layout Guidance

1. Place each RC branch physically close to the switching node return path between PACK_POS and PACK_DRAIN.
2. Keep loop area minimal (short, wide copper).
3. Maintain consistent branch geometry for balanced damping.
4. Use thermal copper around resistor pads for repeated pulse operation.
5. Keep snubber return away from sensitive analog routing (INA226 / ADC front-end).

---

## 6) Schematic Reference Summary

| Channel | Capacitor | Resistor | Connection |
|---|---|---|---|
| Snubber #1 | C19 | R8 | PACK_POS ↔ PACK_DRAIN |
| Snubber #2 | C17 | R3 | PACK_POS ↔ PACK_DRAIN |
| Snubber #3 | C20 | R9 | PACK_POS ↔ PACK_DRAIN |
| Snubber #4 | C18 | R7 | PACK_POS ↔ PACK_DRAIN |

---

## 7) BOM (Snubber Network)

| Item | Qty | Manufacturer | Part Number | Key Spec | Mouser Link |
|---|---:|---|---|---|---|
| RC Snubber Capacitor | 4 | KEMET | **F461BY564J400A** | 0.56 µF, 400 V, PP film, ±5% | https://www.mouser.com/ProductDetail/KEMET/F461BY564J400A |
| RC Snubber Resistor | 4 | TE Connectivity | **2.2 Ω 6 W Thick Film (project-approved TE part)** | 2.2 Ω, 6 W, pulse damping | https://www.mouser.com/c/passive-components/resistors/?q=TE%20Connectivity%202.2%20ohm%206W%20thick%20film |

> Procurement note: keep resistor technology as thick-film/pulse-rated as specified, and validate final stocking SKU against TE series availability before order placement.

---

## 8) Release Status

- ✅ Snubber topology fixed (4× RC)
- ✅ Electrical targets computed and documented
- ✅ Placement constraints defined for PCB layout
- ✅ BOM entries captured for procurement workflow

This specification is ready for PCB placement/routing execution.
