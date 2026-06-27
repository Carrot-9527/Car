# 🚗 Remote Car Control System over 5G NR

> A TCP-based remote car control platform featuring **5G NR wireless communication** (srsRAN), **packet loss simulation**, **base station correction**, and **local no-delay correction** to ensure trajectory tracking accuracy in weak network environments.

---

## 📸 Project Screenshots

### System in Real World
![Car with USRP and 5G Base Station](docs/images/car_usrp.png) 

### Base Station Main Control Interface (Normal Operation)
![Base Station Interface](docs/images/Base_Station_Interface.jpg)

### Packet Loss + Base Station Correction Process
![Base Station Correction](docs/images/Base_Station_Correction.jpg)

### Packet Loss + Local Correction Process
![Local Correction Process](docs/images/Local_Correction_Process.jpg)

### Base Station Correction Terminal Logs
![UE Base Station correction Logs](docs/images/UE_Base_Station_Correction_Logs.jpg)

### Local Correction Runtime Logs
![UE Local Correction Logs](docs/images/UE_Local_Correction_Logs.jpg)

### Local Correction Configuration Interface
![Local Correction Setup](docs/images/Local_Correction_Setup.jpg)

### Local Correction Disabled (Initial Setup)
![Local Disabled](docs/images/Local_Disabled_setup.jpg)

### Local Correction Status & Parameters
![Local Status](docs/images/Local_Status.jpg)

### Logs After Packet Loss at Startup
![Startup Loss Logs](docs/images/Startup_Loss_Logs.jpg)

---

## ✨ Key Features

- **🌐 TCP Remote Control**: Base station (Ubuntu GUI) communicates in real-time with the UE client (Ubuntu) via TCP
- **📡 5G NR Wireless Link**: The car-side UE connects to the base station through an open-source 5G NR protocol stack ([srsRAN](https://docs.srsran.com/projects/project/en/latest/tutorials/source/srsUE/source/index.html)) and USRP hardware
- **📉 Packet Loss Simulation**: Customizable TX/RX packet loss rates for testing in weak network environments; supports intelligent **queue task loss mode** and **targeted loss mode**
- **🎯 Dual-Mode Correction**:
  - **Standalone Correction (Base Station)**: The base station calculates compensation speeds and sends them to the UE
  - **Local Correction (UE Side)**: The UE switches immediately with no delay, forcing trajectory completion with higher accuracy
- **🔄 No-Delay Mode**: Upon detecting packet loss, the system switches to local compensation instantly without waiting for base station recovery
- **📐 Trajectory Queue Control**: Supports speed queue control and ideal trajectory generation (41-point interpolation, 0.20 m path)
- **🔧 CAN Bus Lower Computer**: The UE side controls the car chassis via dual-channel CAN bus

---

## 🏗️ System Architecture

```
┌─────────────────┐         5G NR Air Interface          ┌──────────────────┐
│  Base Station   │  ←──────(srsRAN gNB + USRP)──────→   │   UE Client      │
│  (Ubuntu  GUI)  │         Band 3 | 10 MHz | FDD        │ (Ubuntu / Python)│
│                 │                                      │                  │
│ • Trajectory Viz│                                      │ • CAN Init       │
│ • Packet Loss   │                                      │ • Local Engine   │
│   Simulator     │                                      │ • Forced Mode    │
│ • Speed Queue   │                                      │ • Downsample     │
│ • Correction    │                                      │   Tracking       │
│   Algorithm     │                                      │                  │
└─────────────────┘                                      └──────────────────┘
         │                                                        │
         │ TCP 10.45.0.1:5000 (over 5G TUN)                       │
         └────────────────────────────────────────────────────────┘
                                                                  │
                                                           ┌──────┴──────┐
                                                           │   CAN Bus   │
                                                           │ Channel 0/1 │
                                                           └──────┬──────┘
                                                                  │
                                                            ┌─────┴─────┐
                                                            │Car Chassis│
                                                            │(Motor/Enc)│
                                                            └───────────┘
```

---

## 🚀 Quick Start

### Launch

**Step 1: Start the 5G gNB & Core Network**
```bash
# Start Open5GS core network (AMF/SMF/UPF)
sudo systemctl start open5gs-*

# Start srsRAN gNB with USRP X300
sudo srsran_gnb -c gnb_config.yml
```

**Step 2: Start the Base Station (Server)**
```bash
cd carBS
python3 station_server.py
# The GUI will automatically listen on 0.0.0.0:5000
```

**Step 3: Start the UE Client (Car Side)**
```bash
cd carUE
sudo python3 ue_clientLX.py
# sudo privileges are required for CAN device access
```

---

## ⚙️ Configuration

### Base Station (Server GUI)

| Parameter | Description |
|-----------|-------------|
| **Packet Loss Rate** | Simulated packet loss probability (0% – 100%) |
| **Packet Loss Direction** | TX / RX / Both |
| **Control Mode** | Idle / Remote / CAN Control / Follow |
| **Speed Queue** | X, Y, Angular velocity + duration |
| **Trajectory Control** | Strength: correction intensity (0.0–1.0); Speed: base correction speed |

### UE Client (Local Correction Setup)

| Parameter | Description |
|-----------|-------------|
| **Enable local correction** | `y` to enable local correction; `n` for base-station-only control |
| **Operation mode** | No-delay mode: switch immediately → precise tracking |
| **Forced completion** | Whether to force trajectory completion (ignoring base station recovery) |
| **Base speed** | Local correction base speed (default: 200 mm/s) |
| **Downsampling** | Trajectory point downsampling ratio (default: 40%) |
| **Max correction time** | Maximum correction timeout (default: 120 s) |

---

## 📊 Three Operating Modes

### Mode 1: Ideal Environment (No Packet Loss)
- The base station sends the speed queue, and the UE executes it precisely
- Trajectory sync status: `Trajectory synced: Yes`

### Mode 2: Packet Loss + Base Station Correction
- When `TX` packet loss exceeds the threshold, the base station detects the loss
- The base station enters **Standalone Correction**, calculating compensation speeds (e.g., `x_speed=20, y_speed=-1`)
- The UE receives and executes the correction commands
- Real-time progress display: `Progress: 75%`

### Mode 3: Packet Loss + Local Correction (Recommended)
- The UE client enables **Local Correction**
- Once packet loss is detected (`Station reported packet loss!`), it switches immediately with no delay
- The local closed-loop system tracks the ideal trajectory without relying on real-time base station commands
- Even if the base station recovers, **Forced mode** can ignore the recovery and continue until the local trajectory is complete

---

## 📊 Trajectory Correction Performance Evaluation

To quantitatively validate the effectiveness of the proposed dual-mode correction strategy under wireless packet loss, we designed a controlled trajectory-tracking experiment. The ideal trajectory is a 41-point interpolated curve spanning approximately 0.20 m in the X–Y plane. Four scenarios were tested:

1. **Actual trajectory** — no packet loss, baseline reference;
2. **Packet-loss trajectory** — TX packet loss triggered at a task-group transition without any correction;
3. **Correction trajectory** — local no-delay correction enabled upon packet-loss detection;
4. **Delayed-correction trajectory** — correction activated after a noticeable delay (simulating base-station-only recovery).

### Trajectory Alignment Comparison

The figure below overlays the four measured trajectories against the ideal path after interpolation alignment.

![Trajectory Interpolation Alignment](docs/images/trajectory_alignment_comparison.png)

**Key observations from the trajectory plot:**

- **Actual trajectory** (solid blue) almost perfectly coincides with the **ideal trajectory** (solid black), confirming that the underlying motion-control loop is well calibrated.
- **Packet-loss trajectory** (dashed red) diverges dramatically after the loss point (~X = 0.15 m). The vehicle continues with stale commands, producing a large negative Y excursion and failing to return to the target curve.
- **Correction trajectory** (dashed yellow) deviates slightly at the onset of packet loss but rapidly converges back toward the ideal path, completing the remaining segment with only minor overshoot.
- **Delayed-correction trajectory** (dotted purple) shows intermediate behavior: because the correction command arrives later, the vehicle travels farther off-track before recovery begins, resulting in a persistent lateral offset through the second half of the curve.

### Quantitative Error Analysis

| Metric | Actual | Packet-Loss | Correction | Delayed-Correction |
|--------|--------|-------------|------------|-------------------|
| **MSE (vs Ideal)** | 0.0000 | 0.0051 | 0.0006 | 0.0017 |
| **Mean Absolute Error (m)** | 0.001 | 0.053 | 0.021 | 0.036 |
| **Max Absolute Error (m)** | 0.003 | 0.132 | 0.045 | 0.060 |
| **RMSE (m)** | 0.001 | 0.072 | 0.025 | 0.042 |

![MSE and Absolute Error Statistics](docs/images/mse_comparison.png)

**Interpretation of the error metrics:**

- **MSE reduction**: Compared with the uncorrected packet-loss case, the local no-delay correction achieves an **MSE reduction of roughly 88.2 %** (from 0.0051 to 0.0006). Even the delayed correction still reduces MSE by about 66.7 % (to 0.0017), underscoring that any timely intervention is better than none.
- **Maximum deviation**: The worst-case Y deviation drops from **0.132 m** (packet-loss) to **0.045 m** (correction) — a **65.9 % improvement**. The delayed correction limits the peak error to 0.060 m, which is still less than half the uncorrected value.
- **RMSE consistency**: The correction trajectory’s RMSE (0.025 m) is only slightly above the ideal baseline (0.001 m), whereas the delayed-correction RMSE (0.042 m) reflects the accumulated drift during the waiting period before recovery begins.

### Y-Axis Deviation Curves

The right-hand subplot in the trajectory figure plots the instantaneous Y deviation (ΔY = Y_measured − Y_ideal) versus X position. This view highlights the dynamic behavior:

- The **packet-loss curve** (red) plunges to nearly −0.13 m at X ≈ 0.45 m, indicating a severe undershoot because the vehicle never receives the steering commands needed for the second half of the curve.
- The **correction curve** (yellow) dips to about −0.05 m early on but then flattens back toward zero, demonstrating the local engine’s ability to regenerate the missing trajectory segment and cancel the error.
- The **delayed-correction curve** (purple) shows a smoother, prolonged negative drift that only gradually returns toward zero, confirming that every millisecond of delay translates directly into additional lateral error.
- The **actual trajectory** (blue) remains within ±0.005 m of the ideal path throughout, validating the control hardware’s intrinsic precision.

### Conclusion

The experimental results demonstrate that **local no-delay correction is essential for maintaining trajectory fidelity in 5G NR weak-signal environments**. While base-station-side standalone correction can eventually recover the vehicle, the round-trip latency over the air interface allows the error to grow significantly. By embedding a lightweight trajectory-regeneration loop directly on the UE (car side), the system achieves sub-5 cm maximum deviation even under severe packet loss, satisfying the precision requirements for indoor mobile-robot navigation and remote teleoperation tasks.

---

## 🗼 5G NR Wireless Communication Experiment

### Overview
This project uses the open-source **srsRAN** 5G NR protocol stack to establish the wireless link between the base station and the car-mounted UE. The UE side is equipped with a **USRP B210/X300** for RF transmission and reception.

- **Protocol Stack Reference**: [srsRAN Project — srsUE Source](https://docs.srsran.com/projects/project/en/latest/tutorials/source/srsUE/source/index.html)
- **gNB Hardware**: USRP X300 (192.168.10.2), master clock 184.32 MHz
- **UE Hardware**: USRP B210 (or X300)
- **Core Network**: Open5GS AMF (10.129.143.11:38412)

### Key Experiment Parameters

| Parameter | Value |
|-----------|-------|
| Band | 3 |
| DL ARFCN | 368500 |
| DL Frequency | 1842.50 MHz |
| UL Frequency | 1747.50 MHz |
| Channel Bandwidth | 10 MHz |
| SCS | 15 kHz |
| SSB Periodicity | 10 ms |
| PCI | 1 |
| PLMN | 00101 |
| TAC | 7 |
| UE IMSI | 001010123456780 |
| UE IP | 10.45.0.2 |
| C-RNTI | 0x4601 |
| Security | NEA0 (null encryption), 128-NIA2 (integrity) |

### Key Logs & Analysis

| File | Description | Size |
|------|-------------|------|
| [docs/5g/logs/ue.log](docs/5g/logs/ue.log) | UE side log: cell search, RRC setup, NAS authentication, PDU session establishment | ~120 KB |
| [docs/5g/logs/gnb.log](docs/5g/logs/gnb.log) | gNB side log: PRACH detection, scheduler metrics, F1/E1AP setup, UE context management | ~340 KB |

#### UE Log Key Events
- `09:05:38` — Cell Search: PCI=1, SNR=+14.7 dB, CFO=-61.7 Hz
- `09:05:38` — SIB1 acquired, CellID=4095
- `09:05:38` — PRACH transmitted, preamble=0, TA=6 (16.1 μs)
- `09:05:38` — RRC Setup Complete, C-RNTI=0x4601
- `09:05:38` — NAS Authentication Request/Response successful
- `09:05:38` — Security Mode Complete (NEA0, 128-NIA2)
- `09:05:38` — PDU Session Establishment successful, IP=10.45.0.2
- `09:05:38` — DRB1 established, UE enters Connected state

#### gNB Log Key Events
- `17:04:07` — gNB started, CU-CP/CU-UP/DU initialized
- `17:04:07` — N2 interface connected to AMF (10.129.143.11:38412)
- `17:04:07` — F1 Setup between CU-CP and DU completed
- `17:05:15` — PRACH detected: preamble=0, TA=3.12 μs, RSSI=+4.8 dB
- `17:05:15` — UE created: rnti=0x4601, SRB1/DRB1 configured
- `17:05:15` — Initial Context Setup complete, PDU session configured
- `17:05:16` — UE Reconfiguration complete, DRB1 active

---

## 📉 Packet Loss Simulation Mechanism

### Overview

The `PacketLossSimulator` is a flexible channel degradation simulator designed to emulate packet loss scenarios in communication systems. It supports multiple loss modes ranging from simple probabilistic dropping to intelligent task-group-based loss, enabling systematic testing of error recovery and correction strategies.

### Core Concepts

#### 1. Basic Loss Mode
At its foundation, the simulator operates on a configurable probability basis:

- **Loss Probability**: A value between `0.0` and `1.0` that determines the likelihood of dropping a packet.
- **Direction Control**: Three directional modes are supported:
  - `DIRECTION_TX_ONLY` — drops only outgoing (transmit) packets.
  - `DIRECTION_RX_ONLY` — drops only incoming (receive) packets.
  - `DIRECTION_BOTH` — drops packets in both directions.

In this mode, each packet is evaluated independently using a random draw against the configured probability.

#### 2. Queue Task Loss Mode
For systems that operate on sequential command queues (e.g., robot motion control), the simulator provides an intelligent **queue loss mode** that operates on **task groups** rather than individual packets.

**Task Group Analysis**: The simulator analyzes a speed command queue and automatically segments it into logical task groups based on velocity changes. A new task group is created when the speed in any direction changes by more than a threshold (absolute delta > 50 or relative change > 20%). Each group is characterized by:
- A unique `group_id`
- A descriptive label (e.g., "Forward 100 mm/s + Left 50 mm/s")
- Command index range and duration
- Dominant velocity components

**Loss at Transitions**: When queue loss mode is enabled, packet loss is evaluated **only at task group transitions** (i.e., when the system is about to switch from one motion task to another). This models realistic scenarios where channel degradation causes an entire upcoming maneuver to be lost, not just sporadic individual commands.

#### 3. Targeted Loss Mode
For reproducible testing, the simulator supports **targeted loss mode**, allowing you to specify exactly which task groups should be dropped:

- Provide a list of `task_group_ids` to mark as loss targets.
- Configure a dedicated `target_loss_probability` for these groups (independent of the base probability).
- Useful for regression testing specific failure scenarios.

#### 4. Manual Trigger Mode
In addition to automated probabilistic loss, the simulator supports **manual triggering**:

- Call `trigger_manual_loss_for_task_group(group_id)` to force the loss of a specific upcoming task group.
- The manual trigger takes **highest priority** over random and targeted modes.
- After firing, the trigger resets automatically.

#### 5. Correction & Recovery Integration
When a task group loss is detected at a transition point, the simulator can interact with the host system via an optional GUI callback:

- **User Decision**: A dialog prompts whether to start the correction routine immediately or continue with the current task group.
- **Early Correction Flag**: If the user chooses to correct immediately, `correction_started_early` is set to `True` and `queue_ended_by_loss` is set to `True`, signaling the main controller to abort the remaining queue and initiate recovery using the last known valid command.
- **Last Command Retention**: The simulator tracks the last successfully transmitted command (`last_command_data`) to serve as the baseline for recovery maneuvers.

#### 6. Loss History & Statistics
All loss events are recorded in a structured history log, including:

- Timestamp
- Task group ID and description
- Number of lost commands and total duration
- Loss reason and mode (`manual`, `target`, or `random`)
- Statistical aggregates: total packets, loss rates, average loss size, early correction flags

Statistics can be retrieved programmatically via `get_stats()` or as a formatted text summary via `get_stats_text()`.

### Usage Example

```python
from packet_loss_simulator import PacketLossSimulator

# Initialize with 10% base loss probability, TX-only
sim = PacketLossSimulator(loss_probability=0.1, direction=PacketLossSimulator.DIRECTION_TX_ONLY)
sim.enable()

# Analyze a command queue into task groups
speed_queue = [
    {'x': 100, 'y': 0, 'angular': 0, 'duration': 1.0},   # Group 0: Forward
    {'x': 100, 'y': 0, 'angular': 0, 'duration': 1.0},   # Group 0: Forward
    {'x': 0, 'y': 50, 'angular': 0, 'duration': 2.0},    # Group 1: Left
]
sim.analyze_task_groups(speed_queue)

# Enable queue loss mode for intelligent task-group dropping
sim.enable_queue_loss_mode()

# Optionally target specific groups for deterministic testing
sim.enable_target_loss_mode(task_group_ids=[1], probability=1.0)

# Or manually trigger a loss
sim.trigger_manual_loss_for_task_group(1)

# Simulate a transition between task groups
is_lost, start_correction, reason = sim.simulate_queue_loss(
    current_command=speed_queue[1],
    next_command=speed_queue[2],
    current_task_group=sim.task_groups[0],
    next_task_group=sim.task_groups[1],
    is_task_change=True,
    gui_callback=None  # Set to a Tkinter callback for interactive mode
)

if is_lost:
    print(f"Loss detected: {reason}")
    if start_correction:
        print("Initiating early correction...")
```

### API Summary

| Method | Description |
|--------|-------------|
| `set_loss_probability(p)` | Set base loss probability (`0.0`–`1.0`). |
| `set_direction(d)` | Set loss direction (`TX_ONLY`, `RX_ONLY`, `BOTH`). |
| `enable()` / `disable()` | Toggle the simulator on/off. |
| `analyze_task_groups(queue)` | Segment a speed queue into task groups. |
| `enable_queue_loss_mode()` | Enable intelligent task-group loss at transitions. |
| `enable_target_loss_mode(ids, p)` | Enable deterministic loss for specific task groups. |
| `trigger_manual_loss_for_task_group(id)` | Force loss of a specific upcoming group. |
| `simulate_queue_loss(...)` | Evaluate loss at a task transition point. |
| `simulate_tx_loss(data)` / `simulate_rx_loss(data)` | Legacy per-packet loss simulation. |
| `get_stats()` / `get_stats_text()` | Retrieve comprehensive statistics. |
| `get_loss_history_text(n)` | Retrieve the last `n` loss events. |
| `reset_stats()` | Clear all counters and history. |

---

## 📝 Key Log Interpretation

### Base Station Logs
```
[18:59:08] [PacketLoss] Dropped packet: 发送丢帧 (probability: 90.0%)
[18:59:09] [PacketLoss] Notify UE client to enter local correction mode
[18:59:09] Start correction, base speed: 100mm/s, strength: 0.7
[18:59:11] Standalone correction: X=20, Y=-1, progress=26%
[18:59:20] Standalone correction completed, reached target point
```

### UE Terminal Logs
```
🔴 Station reported packet loss!
✅ No-delay mode: switch to local correction immediately
[Local correction] Running loop (100mm/s) ...
✅ Station packet loss cleared
🔒 Forced mode: ignore station recovery and continue local correction until complete
[Local correction] Path completed (marked)
[Local correction] Stopped
```

---

## 📁 Project Structure

```
RemoteCarControl/
├── carBS/                          # Base station control GUI (Python/Tkinter)
│   ├── station_server.py
│   ├── packet_loss_simulator.py
│   └── ...
├── carUE/                          # UE client (car side, Python)
│   ├── ue_clientLX.py
│   ├── can_driver.py
│   └── ...
├── docs/
│   ├── images/                     # Project screenshots (car, GUI, etc.)
│   │   ├── Base_Station_Interface.png
│   │   ├── Base_Station_Correction.png
│   │   ├── Local_Correction_Process.png
│   │   ├── UE_Base_Station_Correction_Logs.jpg
│   │   ├── UE_Local_Correction_Logs.jpg
│   │   ├── Local_Correction_Setup.jpg
│   │   ├── Local_Disabled_setup.jpg
│   │   ├── Local_Status.jpg
│   │   ├── Startup_Loss_Logs.jpg
│   │   ├── car_usrp.jpg            # Car + USRP photo
│   │   ├── trajectory_alignment_comparison.png  # Trajectory experiment
│   │   └── mse_comparison.png      # MSE & error statistics
│   └── 5g/                         # 5G NR experiment records
│       └── logs/
│           ├── ue.log
│           └── gnb.log
├── README.md
└── .git/
```

---

## 📎 References

- [srsRAN Project Documentation](https://docs.srsran.com/)
- [srsUE Source Tutorial](https://docs.srsran.com/projects/project/en/latest/tutorials/source/srsUE/source/index.html)
- [Open5GS Open Source Core Network](https://open5gs.org/)
- 3GPP TS 38.331 — NR; Radio Resource Control (RRC) protocol specification

---
