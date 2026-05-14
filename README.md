# Bio-Inspired Ornithopter — Flapping Wing UAV

> *A mechanically flapping, radio-controlled aircraft inspired by bird flight — designed from scratch using CAD, custom gear mechanisms, and an intelligent dual-controller avionics stack.*

---

## Project Overview

This project is the complete design and build of a **radio-controlled ornithopter** — an aircraft that flies by flapping its wings, mimicking the biomechanics of birds rather than relying on fixed wings or rotors.

Unlike conventional UAVs, an ornithopter must solve a fundamentally harder problem: **converting continuous rotary motor torque into a periodic, asymmetric flapping stroke** that generates both lift and thrust simultaneously. Every subsystem — mechanical, electronic, and software — was designed around this constraint.


---

## What is an Ornithopter?

An **ornithopter** is a type of aircraft that generates lift and thrust entirely through the flapping motion of its wings — no propeller, no rotor. The concept dates back to Leonardo da Vinci, but building a working RC ornithopter requires solving several non-trivial engineering challenges:

| Challenge | Solution Used |
|---|---|
| Rotary → Reciprocating motion | Crank-rocker four-bar linkage |
| High wing-beat torque | 1:3 cluster gear reduction |
| Wing symmetry at speed | Dual symmetric gear train |
| Attitude stability at climb | Intelligent gimbal tail with pitch/roll servos |
| Torque coupling (power vs attitude) | Automatic roll bias derived from pitch |

---

## Mechanical Design

### Flapping Mechanism — Crank Rocker Linkage

The core of the ornithopter is a **crank-rocker four-bar mechanism** that converts the continuous rotation of a BLDC motor into a reciprocating (flapping) wing stroke. The mechanism was designed to replicate the kinematics of bird wing beats — a faster downstroke than upstroke — for aerodynamic efficiency.

**Key features:**
- Symmetric dual gear train ensures both wings beat in phase
- 1:3 cluster gear reduction lowers the wing-beat frequency and multiplies torque
- Carbon fiber rods transmit the stroke from crank pins to wing leading edges
- Wing attachment points (triangular connectors) at the rod tips allow the membrane to flex naturally

### CAD Design

The complete airframe and mechanism were designed in **Autodesk Fusion 360**. 

<p align="center">
<a href="https://github.com/sneha19122004/Development-of-Ornithopter-Mechanism/blob/main/CAD-model/front-view.png">
<img src="https://github.com/sneha19122004/Development-of-Ornithopter-Mechanism/blob/main/CAD-model/front-view.png" alt="3D CAD Model of Ornithopter" width="700"/>
</a>
</p>
3D CAD Model of Ornithopter
<p align="center">
<a href="https://github.com/sneha19122004/Development-of-Ornithopter-Mechanism/blob/main/CAD-model/gear-mechanism.png">
<img src="https://github.com/sneha19122004/Development-of-Ornithopter-Mechanism/blob/main/CAD-model/gear-mechanism.png" alt="1:3 reduction cluster, crank pins, rocker arms, and the secondary small gear driving the timing synchronization" width="300"/>
</a>
</p>
1:3 reduction cluster, crank pins, rocker arms, and the secondary small gear driving the timing synchronization
<p align="center">
<a href="https://github.com/sneha19122004/Development-of-Ornithopter-Mechanism/blob/main/CAD-model/orthographic-view.png">
<img src="https://github.com/sneha19122004/Development-of-Ornithopter-Mechanism/blob/main/CAD-model/orthographic-view.png" width="700"/>
</a>
</p>
<p align="center">
<a href="https://github.com/sneha19122004/Development-of-Ornithopter-Mechanism/blob/main/CAD-model/orthographic-back-view.png">
<img src="https://github.com/sneha19122004/Development-of-Ornithopter-Mechanism/blob/main/CAD-model/orthographic-back-view.png" width="700"/>
</a>
</p>


The ring-frame fuselage body houses the motor, gear train, and electronics bay while keeping the structure lightweight.

---

## Electronics & Avionics Architecture

The ornithopter uses a **dual-brain controller architecture** — a design choice that separates flight stabilization from gimbal/tail control, keeping each system focused on its task.

```
RadioMaster Controller
        │
        ▼
  RadioMaster SB
   Nano Receiver
        │
        ▼
┌───────────────────────────────────┐
│        DUAL BRAIN CONTROLLERS     │
│                                   │
│  Dreamfly F722 FC ◄──────► ESP32  │
│   (flight & motor ctrl)  (gimbal) │
└───────────────────────────────────┘
        │                   │
        ▼                   ▼
  BLDC Motor          Pitch Servo
  30A ESC             Roll Servo
  1:3 Gear Reduction  (tail gimbal)
  Crank-Rocker Flap

POWER: 3S LiPo → Power Distribution Board → ESC + FC + ESP32
```

### Component List

| Component | Role |
|---|---|
| RadioMaster Controller | Pilot transmitter (ELRS) |
| RadioMaster SB Nano Receiver | ELRS RC link receiver |
| Dreamfly F722 FC | Flight controller — motor output, stabilisation loop |
| ESP32 WROOM | Gimbal/tail controller — reads MSP from FC, drives servos |
| Standard 30A ESC | Drives the BLDC motor |
| BLDC Motor | Primary power plant — drives the flapping mechanism |
| 3S LiPo Battery | Main power source |
| Power Distribution Board | Distributes power to all subsystems |
| 2× Standard Servos | Pitch and roll of the tail gimbal |

### Why Two Controllers?

The **Dreamfly F722** handles motor ESC output and the main RC link. But controlling a tail gimbal with automatic torque compensation logic required a dedicated microcontroller. The **ESP32** reads RC channel data from the F722 via **MSP (MultiWii Serial Protocol)** over UART, then runs its own servo control loop independently. This keeps latency low on both paths.

Block Diagram
<p align="center">
<a href="https://github.com/sneha19122004/Development-of-Ornithopter-Mechanism/blob/main/electronics/block-diagram.png">
<img src="https://github.com/sneha19122004/Development-of-Ornithopter-Mechanism/blob/main/electronics/block-diagram.png" width="500"/>
</a>
</p>

Wiring Diagram
<p align="center">
<a href="https://github.com/sneha19122004/Development-of-Ornithopter-Mechanism/blob/main/electronics/wiring-diagram.png">
<img src="https://github.com/sneha19122004/Development-of-Ornithopter-Mechanism/blob/main/electronics/wiring-diagram.png" width="500"/>
</a>
</p>
---

## Firmware — Intelligent Gimbal Controller

> **File:** `gimbal_controller/main.cpp`  
> **Platform:** ESP32 WROOM — PlatformIO / Arduino Framework  
> **Language:** C++

### What It Does

The tail gimbal on an ornithopter serves the same function as a tail on a bird — controlling pitch attitude and counteracting propulsive torque. This firmware implements **fully automatic pitch and roll control** derived from a single input: throttle.

**No manual servo sticks needed.** The pilot only controls throttle; the ESP32 figures out what the tail should do.

### Control Logic

```
Throttle (CH3)
     │
     ▼
 throttleToPitchDeg()          ← 3-zone soft ramp
     │
     ▼
 targetPitchDeg ──► Slew-Rate Limiter ──► pitchServo.write()
     │                   (0.12°/ms)
     │
     └──► currentPitchDeg (actual hardware position)
               │
               ▼
         pitchToRollBiasDeg()  ← automatic torque compensation
               │
               ▼
         targetRollDeg ──► Slew-Rate Limiter ──► rollServo.write()
                               (0.20°/ms)
```

### Throttle → Pitch Mapping (3-Zone Ramp)

| Throttle Range | PWM (µs) | Pitch Servo |
|---|---|---|
| 0% – 30% | 1000–1300 µs | HOME (90°) — stationary |
| 30% – 40% | 1300–1400 µs | Smooth ramp: 90° → 120° |
| 40% – 100% | 1400–2000 µs | PITCH_MAX (120°) — held |

The narrow ramp band (only 100 µs wide) means the tail snaps to climb attitude quickly once the pilot crosses 30% throttle — but the **slew-rate limiter** (0.12°/ms) prevents any mechanical shock to the gear train regardless of how fast the pilot moves the stick.

### Automatic Roll Bias

At high throttle, the flapping mechanism generates propulsive torque that tends to roll the aircraft. The firmware automatically applies a small roll correction that scales linearly with pitch deflection:

```
pitch at HOME (90°)  →  0° roll bias   (no correction at idle)
pitch at MAX  (120°) →  +5° roll bias  (tunable: ROLL_BIAS_AT_MAX_PITCH)
```

### Dynamic Roll Guard (Safety)

As the tail pitches up, the physical clearance between the tail and fuselage shrinks. The firmware tracks this in real time and **reduces the maximum allowed roll travel** as pitch increases, preventing the servo from driving the mechanism into a bind:

```
pitch at HOME  →  ±30° roll authority
pitch at MAX   →   ±8° roll authority
```

### Failsafe Watchdog

If the MSP link from the F722 goes silent for more than **500 ms**, the firmware detects the loss and smoothly drives both servos back to HOME (90°) via the slew limiter — placing the aircraft in a best-glide attitude rather than locking up.

### Firmware Highlights

```cpp
// Three-zone throttle ramp — zero discontinuity at boundaries
float throttleToPitchDeg(uint16_t thrUs);

// Automatic roll compensation scaled to actual pitch position  
float pitchToRollBiasDeg(float actualPitchDeg);

// Dynamic roll authority limit — shrinks as pitch extends
float dynamicRollMax(float actualPitchDeg);

// MSP state-machine parser — byte-by-byte, CRC-verified
void mspFeedByte(uint8_t b);
```

---

## Repository Structure

```
ornithopter/
│
├── README.md                          ← You are here
│
├── firmware/
│   └── gimbal_controller/
│       └── main.cpp                   ← ESP32 gimbal firmware (full source)
│
├── cad/
│   ├── screenshots/
│   │   ├── gear_train_closeup.png     ← 1:3 cluster gear + crank-rocker detail
│   │   ├── full_spread_config.png     ← Wing extended / full flap position
│   │   └── folded_config.png          ← Wing stowed / compact configuration
│   └── [SolidWorks source files]      ← .SLDPRT / .SLDASM
│
├── electronics/
│   ├── system_block_diagram.png       ← Full avionics architecture
│   └── wiring_diagram.png             ← Physical wiring: FC, ESC, ESP32, receiver
│
└── media/
    └── [video iterations]             ← Build and test videos (see below)
```

---

## Video Iterations

The project went through multiple physical build and test iterations. Videos documenting each stage are included in the `media/` folder:
First Iteration
<p align="center">
<a href="https://github.com/sneha19122004/Development-of-Ornithopter-Mechanism/blob/main/media/first-iteration.mp4">
<img src="https://img.shields.io/badge/%20Watch%20Demo-Physical%20Robot-green?style=for-the-badge" alt="Watch First Iteration"/>
</a>
</p>

Second Iteration
<p align="center">
<a href="https://github.com/sneha19122004/Development-of-Ornithopter-Mechanism/blob/main/media/second-iteration.mp4">
<img src="https://img.shields.io/badge/%20Watch%20Demo-Physical%20Robot-green?style=for-the-badge" alt="Watch Second Iteration"/>
</a>
</p>

---

## Build Notes & Lessons Learned

**Gear train tolerances matter enormously.** Even small backlash in the 1:3 cluster causes the wing phase to drift mid-flight, so the gears were designed with tight mesh clearance and the crank pins were press-fit rather than bolted.

**The 3-zone throttle ramp was essential.** Early firmware used a simple linear map from throttle → pitch, which caused the tail to twitch at low throttle when the pilot's hand wasn't perfectly still. The deadband below 30% eliminated this entirely.

**Roll bias is airframe-specific.** The `ROLL_BIAS_AT_MAX_PITCH` constant will be different for every ornithopter depending on motor position, wing area, and CG. Start at 0.0 and tune upward in 1° increments.

**MSP is the right bridge between the F722 and ESP32.** It's compact, CRC-verified, and the F722 outputs it natively. Bit-banging a custom UART protocol would have been fragile; MSP just works.

---

## Getting Started (Replicate This Build)

### Firmware Upload

1. Install [PlatformIO](https://platformio.org/) (VS Code extension recommended)
2. Open `firmware/gimbal_controller/` as a PlatformIO project
3. Connect ESP32 via USB
4. Build and upload:
   ```bash
   pio run --target upload
   ```
5. Open Serial Monitor at 115200 baud to see live telemetry

### Key Tuning Parameters

```cpp
// In main.cpp — adjust these for your airframe:

#define PITCH_MAX              120   // Tail pitch at full throttle (degrees)
#define ROLL_BIAS_AT_MAX_PITCH 5.0f  // Roll correction at max pitch — tune per airframe
#define SLEW_PITCH_DEG_PER_MS  0.12f // Pitch servo speed limit
#define SLEW_ROLL_DEG_PER_MS   0.20f // Roll servo speed limit
#define MSP_TIMEOUT_MS         500   // Failsafe trigger time (ms)
```

### Wiring (ESP32 ↔ F722)

| ESP32 Pin | Connection |
|---|---|
| GPIO 16 (RX2) | F722 MSP TX out |
| GPIO 17 (TX2) | F722 MSP RX in (optional telemetry) |
| GPIO 18 | Pitch servo signal |
| GPIO 19 | Roll servo signal |
| GND | Common ground with F722 |

---

## About This Project

This ornithopter was an engineering project covering:

- **Mechanism design** — four-bar linkage kinematics, gear train sizing, CAD in SolidWorks
- **Avionics integration** — dual-controller architecture, MSP serial protocol
- **Embedded firmware** — real-time servo control, slew limiting, failsafe logic on ESP32
- **Iterative testing** — multiple physical build-test-revise cycles documented on video

The project sits at the intersection of biomimetic robotics, embedded systems, and mechanical design — and demonstrates that flapping-wing flight, while mechanically demanding, is entirely achievable with off-the-shelf RC components and custom firmware.

---

*Designed & built by [Sneha Alphonso Francis](https://github.com/sneha19122004)*
