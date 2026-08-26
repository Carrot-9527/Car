```markdown
# Replication Manual: Remote Car Control System over 5G NR

---

## 1. System Overview

This system enables remote control of a car chassis over a 5G NR wireless link. The base station (BS) runs a GUI control panel that sends speed commands via TCP to the User Equipment (UE) mounted on the car. The UE translates these commands into CAN bus signals to drive the car's motors.

**Key capabilities:**
- TCP remote control over 5G NR (srsRAN + USRP)
- Programmable packet loss simulation (0–100%)
- Two correction modes when packet loss occurs:
  - **Base Station Correction**: BS calculates compensation and sends to UE
  - **Local Correction (No-Delay)**: UE switches instantly to local trajectory tracking

---

## 2. Prerequisites

### 2.1 Hardware

Known hardware components from the project:
- USRP (B210/X300)
- Car chassis (differential drive)
- CAN adapter (requires `libcontrolcan.so`)
- Band 3 (1800–1880 MHz) RF antennas

### 2.2 Software

| Component | Description |
|-----------|-------------|
| Ubuntu | 22.04 LTS (inferred from `/home/ubuntu22/` path) |
| Python | Main control scripts |
| srsRAN Project | 5G NR gNB and UE protocol stack |
| Open5GS | 5G core network |
| UHD | USRP hardware driver |
| CAN Driver (`libcontrolcan.so`) | USB-to-CAN adapter |
| Python Packages | Tkinter, ctypes, socket, etc. |

---

## 3. Hardware Setup

### 3.1 Connection Overview

The system consists of two main sides connected via the 5G NR air interface:

**Base Station Side:**
- BS PC running Ubuntu
- USRP
- Band 3 antenna

**UE Side (Car):**
- Car-mounted PC running Ubuntu
- USRP
- Band 3 antenna
- USB-to-CAN adapter connected to the car's CAN bus
- Car chassis with dual motors (differential drive)

---

## 4. Software Installation

### 4.1 CAN Driver Installation

This project uses `libcontrolcan.so` and `libcontrolcan.a`. The exact installation procedure depends on the CAN adapter vendor.

---

## 5. Network Configuration

### 5.1 5G NR Network Parameters

| Parameter | Value |
|-----------|-------|
| Band | 3 |
| DL ARFCN | 368500 |
| DL Frequency | 1842.50 MHz |
| UL Frequency | 1747.50 MHz |
| Bandwidth | 10 MHz |
| SCS | 15 kHz |
| SSB Periodicity | 10 ms |
| PCI | 1 |
| PLMN | 00101 |
| TAC | 7 |
| UE IP | 10.45.0.2 |
| BS IP | 10.45.0.1 |
| AMF IP | 10.129.143.11 |
| AMF Port | 38412 |

### 5.2 TCP Control Channel

- **Port**: `5000`
- **BS listens on**: `0.0.0.0:5000`
- **UE connects to**: `10.45.0.1:5000`

---

## 6. Code Acquisition & Directory Structure

### 6.1 Clone the Repository

```bash
git clone [repository URL]
cd RemoteCarControl
```

### 6.2 Directory Structure

```
RemoteCarControl/
├── carBS/
│   └── station_server.py          # Base station GUI server
├── carUE/
│   ├── ue_clientLX.py             # UE main client script
│   └── can_driver.py              # CAN bus interface
├── docs/
│   ├── images/                    # Project screenshots
│   └── 5g/logs/                   # Reference logs
│       ├── ue.log
│       └── gnb.log
├── README.md
└── .git/
```

---

## 7. Configuration Files

### 7.1 UE Client Configuration (`carUE/ue_clientLX.py`)

The following modifications are required in `ue_clientLX.py`:

**CAN Library Path:**
```python
self.canDLL = cdll.LoadLibrary('/home/ubuntu22/controlcan/libcontrolcan.so')
```
Change this to the actual path where `libcontrolcan.so` and `libcontrolcan.a` are located on your system.

**Base Station IP:**
```python
def __init__(self, station_host='10.45.0.1', station_port=5000, ...):
```
Change `station_host` to the actual IP address of the Base Station side.

---

## 8. Step-by-Step Execution

### 8.1 Start 5G Core Network (BS PC)

```bash
sudo systemctl start open5gs-*
```

### 8.2 Start srsRAN gNB (BS PC)

```bash
sudo srsran_gnb -c gnb_config.yml
```

**Expected log events:**
- gNB started, CU-CP/CU-UP/DU initialized
- N2 interface connected to AMF (10.129.143.11:38412)
- F1 Setup between CU-CP and DU completed

### 8.3 Start Base Station GUI Server (BS PC)

```bash
cd carBS
python3 station_server.py
```

The GUI will listen on `0.0.0.0:5000`.

### 8.4 Start srsUE (Car PC)

```bash
sudo srsue -c ue_config.yml
```

**Expected log events:**
- Cell Search: PCI=1, SNR=+14.7 dB, CFO=-61.7 Hz
- SIB1 acquired, CellID=4095
- PRACH transmitted, preamble=0, TA=6 (16.1 μs)
- RRC Setup Complete, C-RNTI=0x4601
- NAS Authentication Request/Response successful
- Security Mode Complete (NEA0, 128-NIA2)
- PDU Session Establishment successful, IP=10.45.0.2
- DRB1 established, UE enters Connected state

### 8.5 Start UE Client (Car PC)

```bash
cd carUE
sudo python3 ue_clientLX.py
```

> **Note**: `sudo` privileges are required for CAN device access.

The UE obtains IP `10.45.0.2` via 5G PDU Session and connects to the base station TCP server at `10.45.0.1:5000`.

### 8.6 Configuration Prompts

After starting `ue_clientLX.py`, the script will prompt for local correction settings:

| Parameter | Options | Description |
|-----------|---------|-------------|
| Enable local correction | `y` / `n` | Enable local correction or use base-station-only control |
| Operation mode | no-delay / forced | no-delay mode switches immediately; forced ignores BS recovery |
| Base speed | numeric | Local correction base speed |
| Downsampling ratio | percentage | Trajectory point downsampling ratio |
| Max correction time | seconds | Maximum correction timeout |

---

## 9. Operating Modes

### Mode 1: Ideal Environment (No Packet Loss)

- Base station sends speed queue; UE executes it precisely
- Trajectory sync status: `Trajectory synced: Yes`

---

### Mode 2: Packet Loss + Base Station Correction

- When `TX` packet loss exceeds threshold, base station detects the loss
- Base station enters **Standalone Correction**, calculating compensation speeds
- UE receives and executes correction commands
- Real-time progress display: `Progress: 75%`

**UE Terminal Logs:**
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

### Mode 3: Packet Loss + Local Correction (No-Delay)

- UE client enables **Local Correction**
- Once packet loss is detected, UE switches immediately with no delay
- Local closed-loop system tracks ideal trajectory without relying on real-time BS commands
- Even if BS recovers, **Forced mode** can ignore recovery and continue until local trajectory is complete

**BS Terminal Logs:**
```
[18:59:08] [PacketLoss] Dropped packet: 发送丢帧 (probability: 90.0%)
[18:59:09] [PacketLoss] Notify UE client to enter local correction mode
[18:59:09] Start correction, base speed: 100mm/s, strength: 0.7
[18:59:11] Standalone correction: X=20, Y=-1, progress=26%
[18:59:20] Standalone correction completed, reached target point
```

---

## 10. Task Switching Procedure

**Important**: Between different experimental tasks, you must restart the Python files and the car.

### 10.1 Stop Running Processes

1. Stop `ue_clientLX.py`: Press `Ctrl+C` in its terminal
2. Stop `station_server.py`: Press `Ctrl+C` in its terminal
3. Stop gNB: Press `Ctrl+C` in its terminal

### 10.2 Reset Car Hardware

1. Power off the car (disconnect battery or press emergency stop)
2. Wait **10 seconds** for CAN controller capacitors to discharge
3. Power on the car

### 10.3 Start New Task

Repeat all steps from [Section 8](#8-step-by-step-execution) in order.

---

## 11. Troubleshooting

### 11.1 CAN Library Path Error

**Error:** `libcontrolcan.so` not found.

**Solution:** Update the path in `ue_clientLX.py`:
```python
self.canDLL = cdll.LoadLibrary('/actual/path/to/libcontrolcan.so')
```

### 11.2 TCP Connection Refused

**Error:** `ConnectionRefusedError: [Errno 111] Connection refused`

**Solution:** Verify the BS IP address in `ue_clientLX.py` matches the actual BS IP.

---

## 12. References

- [srsRAN Project Documentation](https://docs.srsran.com/)
- [srsUE Source Tutorial](https://docs.srsran.com/projects/project/en/latest/tutorials/source/srsUE/source/index.html)
- [Open5GS Open Source Core Network](https://open5gs.org/)
- 3GPP TS 38.331 — NR; Radio Resource Control (RRC) protocol specification
