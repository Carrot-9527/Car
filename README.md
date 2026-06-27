[README.md](https://github.com/user-attachments/files/29403961/README.md)
# 🚗 Remote Car Control System over 5G NR

> A TCP-based remote car control platform featuring **5G NR wireless communication** (srsRAN), **packet loss simulation**, **base station correction**, and **local correction** to ensure trajectory tracking accuracy in weak network environments.

---

## 📸 Project Overview

### System in Real World
| Car + USRP B210 | 5G gNB / Core Network |

| ![Car with USRP and 5G Base Station](docs/images/car_usrp.jpg) |

---

## ✨ Key Features

- **🌐 TCP Remote Control**: Base station (Ubuntu GUI) communicates in real-time with the UE client (Ubuntu) via TCP
- **📡 5G NR Wireless Link**: The car-side UE connects to the base station through an open-source 5G NR protocol stack ([srsRAN](https://docs.srsran.com/projects/project/en/latest/tutorials/source/srsUE/source/index.html)) and USRP hardware
- **📉 Packet Loss Simulation**: Customizable TX/RX packet loss rates for testing in weak network environments
- **🎯 Dual-Mode Correction**:
  - **Standalone Correction (Base Station)**: The base station calculates compensation speeds and sends them to the UE
  - **Local Correction (UE Side)**: The UE switches immediately with no delay, forcing trajectory completion with higher accuracy
- **🔄 No-Delay Mode**: Upon detecting packet loss, the system switches to local compensation instantly without waiting for base station recovery
- **📐 Trajectory Queue Control**: Supports speed queue control and ideal trajectory generation (41-point interpolation, 0.20m path)
- **🔧 CAN Bus Lower Computer**: The UE side controls the car chassis via dual-channel CAN bus

---

## 🏗️ System Architecture

```
┌─────────────────┐         5G NR Air Interface          ┌──────────────────┐
│  Base Station   │  ←──────(srsRAN gNB + USRP)──────→   │   UE Client      │
│  (Ubuntu  GUI)  │         Band 3 | 10 MHz | FDD          │ (Ubuntu / Python)│
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

> The UE obtains IP `10.45.0.2` via 5G PDU Session and connects to the base station TCP server at `10.45.0.1:5000`.

---

## ⚙️ Configuration

### Base Station (Server GUI)

| Parameter | Description |
|-----------|-------------|
| **Packet Loss Rate** | Simulated packet loss probability (0% - 100%) |
| **Packet Loss Direction** | TX / RX / Both |
| **Control Mode** | Idle / Remote / CAN Control / Follow |
| **Speed Queue** | X, Y, Angular velocity + duration |
| **Trajectory Control** | Strength: correction intensity (0.0-1.0); Speed: base correction speed |

### UE Client (Local Correction Setup)

| Parameter | Description |
|-----------|-------------|
| **Enable local correction** | `y` to enable local correction; `n` for base-station-only control |
| **Operation mode** | No-delay mode: switch immediately → precise tracking |
| **Forced completion** | Whether to force trajectory completion (ignoring base station recovery) |
| **Base speed** | Local correction base speed (default: 200 mm/s) |
| **Downsampling** | Trajectory point downsampling ratio (default: 40%) |
| **Max correction time** | Maximum correction timeout (default: 120s) |

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
│   │   ├── car_usrp.jpg      # ⬅️ Put your car+USRP photo here
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
