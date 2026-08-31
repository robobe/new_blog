---
title: Power
tags:
    - capacitor
---

# Why an FPV ESC needs an input capacitor

An FPV ESC rapidly switches large motor currents. The battery cannot respond
instantly because the battery leads have inductance. A capacitor across the ESC
battery input acts as a nearby energy reservoir:

- It supplies short current pulses to the ESC.
- It absorbs regenerative current from the motors.
- It reduces voltage spikes caused by battery-wire inductance.
- It reduces electrical noise reaching the flight controller and video system.
- It helps protect the ESC MOSFETs from excessive input voltage.

```text
Battery ───── long inductive wires ───── ESC
                                      + │
                                   capacitor
                                      - │
```

Mount the capacitor directly across the ESC's `BAT+` and `BAT−` pads. Long
capacitor leads add inductance and reduce its effectiveness.

!!! warning "Capacitor polarity"
    An aluminum electrolytic capacitor is polarized. Connect its positive lead
    to `BAT+` and its striped negative side to `BAT−`. Reversed polarity or a
    voltage above its rating can cause the capacitor to vent or fail.

!!! info "Battery connection and inrush current"
    When the battery is connected, an empty input capacitor briefly draws a
    large charging current. This can create a connector spark and stress the
    connector or power switch. See [Inrush Current in FPV Power
    Systems](../power_system/inrush_current/index.md) for calculations and
    mitigation methods.

## Why voltage spikes happen

Battery wires behave like a small inductor. An inductor resists rapid current
changes:

\[
V_L = L\frac{dI}{dt}
\]

If the ESC rapidly changes tens of amperes, even a small wire inductance can
generate a large voltage spike.

The capacitor accepts or supplies this current locally:

\[
I = C\frac{dV}{dt}
\]

Rearranging gives a useful first sizing equation:

\[
C \geq \frac{\Delta I \times \Delta t}{\Delta V}
\]

Where:

- \(C\) is the required capacitance.
- \(\Delta I\) is the current change.
- \(\Delta t\) is the time the capacitor must support that current.
- \(\Delta V\) is the acceptable capacitor voltage change.

## Example capacitance calculation

Assume:

- Current changes by 50 A.
- The capacitor must support it for 20 µs.
- The acceptable capacitor voltage change is 2 V.

\[
C \geq \frac{50 \times 20 \times 10^{-6}}{2}
\]

\[
C \geq 500\ \mu F
\]

The next common size would be approximately:

```text
680 µF or 1000 µF
```

This is only a starting estimate. The ESC's current waveform, switching
frequency, wiring, motor braking, ESR, and existing onboard capacitance all
affect the real requirement.

## ESR can matter more than capacitance

A real capacitor has equivalent series resistance, or ESR. A sudden current
produces an immediate voltage step:

\[
\Delta V_{ESR} = \Delta I \times ESR
\]

For a 50 A current change and a 20 mΩ capacitor:

\[
\Delta V_{ESR} = 50 \times 0.020 = 1\ V
\]

If the ESR were 100 mΩ:

\[
\Delta V_{ESR} = 50 \times 0.100 = 5\ V
\]

Therefore, a physically large capacitor with poor ESR may perform worse than a
smaller low-ESR capacitor. Increasing capacitance alone cannot remove the ESR
voltage step. See [TI's capacitor transient analysis](https://www.ti.com/lit/ug/tidu186/tidu186.pdf).

Look for:

- Low ESR
- High ripple-current rating
- 105 °C temperature rating
- Short, thick leads
- A reputable capacitor series

Ripple current heats a capacitor according to its ESR, and excess internal
temperature substantially reduces capacitor life. See [TDK's electrolytic
capacitor guidance](https://www.tdk-electronics.tdk.com/download/185386/31a5416e653dd6e4e428b8208d65cc2e/pdf-generaltechnicalinformation.pdf).

## Voltage rating

Use the battery's maximum fully charged voltage, not its nominal voltage:

\[
V_{battery,max} = cell\ count \times 4.2\ V
\]

| Battery | Fully charged | Practical capacitor rating |
| --- | ---: | ---: |
| 3S | 12.6 V | 25 V |
| 4S | 16.8 V | 25 V or 35 V |
| 6S | 25.2 V | 35 V minimum; 50 V gives more margin |
| 8S | 33.6 V | 50 V |
| 12S | 50.4 V | 63 V or higher |

The voltage rating must also tolerate spikes. Do not operate an electrolytic
capacitor above its rating.

## Practical FPV starting points

When the ESC manufacturer does not specify a value, these are reasonable
experimental starting points, not universal calculations:

| Setup | Starting capacitor |
| --- | --- |
| Small 3S–4S build | 470–680 µF, 25–35 V |
| Typical 4S build | 680–1000 µF, 35 V |
| Typical 6S build | 680–1000 µF, 35–50 V |
| High-current or long battery leads | 1000–2200 µF, suitable voltage |

Two capacitors in parallel can be useful:

\[
C_{total} = C_1 + C_2
\]

Parallel capacitors also reduce combined ESR and share ripple current, assuming
similar parts and a good physical layout.

## Engineering selection procedure

1. Start with the ESC manufacturer's recommended capacitor.
2. Calculate the fully charged battery voltage.
3. Select a low-ESR, 105 °C capacitor with adequate voltage margin.
4. Estimate capacitance using:

   \[
   C \geq \frac{\Delta I\Delta t}{\Delta V}
   \]

5. Check the maximum acceptable ESR using:

   \[
   ESR \leq \frac{\Delta V_{ESR}}{\Delta I}
   \]

6. Check that the capacitor's ripple-current rating is sufficient.
7. Install it directly at the ESC battery pads.
8. Measure the input voltage with an oscilloscope using a short ground spring.

A long oscilloscope ground lead can create a false voltage spike, so measurement
technique matters.

For most ordinary 4S–6S FPV builds, a genuine **680–1000 µF low-ESR, 35–50 V,
105 °C capacitor mounted directly on the ESC** is a sensible starting point.
Exact sizing ultimately requires measuring the real ESC input waveform.
