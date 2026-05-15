# Precision Olfactometer — MFC Delivery Control

A PyQt6 GUI for driving an **MKS 946** vacuum/flow controller and the **6 Mass Flow Controller (MFC) channels** connected to it. The application lets you set per-channel flow rates (SCCM), build vapor mixture recipes, zero channels, monitor live flow, record sessions to CSV, and diagnose hardware issues.

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Hardware Requirements](#2-hardware-requirements)
3. [Software Requirements](#3-software-requirements)
4. [Installation](#4-installation)
5. [Channel & Slot Mapping](#5-channel--slot-mapping)
6. [Launching the Application](#6-launching-the-application)
7. [Standard Operating Procedure (SOP)](#7-standard-operating-procedure-sop)
8. [GUI Reference](#8-gui-reference)
9. [Creating & Using Mixture Recipes](#9-creating--using-mixture-recipes)
10. [Recording Flow Data](#10-recording-flow-data)
11. [Zeroing Channels](#11-zeroing-channels)
12. [Diagnostics & Troubleshooting](#12-diagnostics--troubleshooting)
13. [Terminal-Only Mode](#13-terminal-only-mode)
14. [Configuration & Customization](#14-configuration--customization)
15. [Shutdown Procedure](#15-shutdown-procedure)
16. [FAQ](#16-faq)

---

## 1. System Overview

The script (`delivery.py`) opens a serial connection to an **MKS 946** controller and exposes its **6 MFC channels** through a dual-pane GUI:

- **Left pane:** real-time flow plot (SCCM vs. time, 30 s rolling window).
- **Right pane:** connection controls, per-channel set/readback widgets, recipe controls, zero/diagnostic tools, and recording controls.

By default the channels are named:

| Internal Key | Display Name | Typical Role         | Max (SCCM) |
| ------------ | ------------ | -------------------- | ---------- |
| Ch1          | Mixture      | Output / mixing line | 200        |
| Ch2          | Ch2          | Spare / user-defined | 200        |
| Ch3          | Dilution2    | Dilution for Vapor2  | 200        |
| Ch4          | Vapor2       | Vapor source 2       | 200        |
| Ch5          | Vapor1       | Vapor source 1       | 200        |
| Ch6          | Dilution1    | Dilution for Vapor1  | 200        |

These can be changed at the top of `delivery.py` — see [Section 14](#14-configuration--customization).

---

## 2. Hardware Requirements

- **MKS 946** multi-gas/vacuum controller.
- Up to **6 MFC channels**, populated across slots A/B/C (two channels per slot).
- Serial cable from the 946 to the PC (RS-232 or USB-to-Serial adapter).
- Default serial settings used by the script:
  - Baud rate: **9600**
  - Data bits: **8**, Parity: **None**, Stop bits: **1**
  - Flow control: **None**
  - Controller address: **253** (configurable via `DELIVERY_CONTROLLER_ADDRESS` env var)

> Confirm the 946's front-panel serial setup matches these values before connecting.

---

## 3. Software Requirements

- **Python 3.9+** (3.10/3.11 recommended)
- Operating systems supported: **Windows 10/11**, **Linux**, **macOS**
- Python packages:
  - `PyQt6`
  - `pyserial`
  - `pyqtgraph`
  - `numpy`
  - `scipy`

---

## 4. Installation

### 4.1 Get the files

Place `delivery.py` (and `icon.png` if you have one) in a working folder. Examples:

- Linux: `~/delivery-script-1.7/` or `~/Downloads/delivery-script-1.7/`
- Windows: `C:\Users\<you>\Downloads\delivery-script-1.7\`

### 4.2 Create a virtual environment (recommended)

**Windows (PowerShell):**

```powershell
cd C:\Users\<you>\Downloads\delivery-script-1.7
python -m venv .venv
.\.venv\Scripts\Activate.ps1
```

**Linux / macOS:**

```bash
cd ~/delivery-script-1.7
python3 -m venv .venv
source .venv/bin/activate
```

### 4.3 Install dependencies

```bash
pip install PyQt6 pyserial pyqtgraph numpy scipy
```

Or, if you create a `requirements.txt`:

```
PyQt6
pyserial
pyqtgraph
numpy
scipy
```

```bash
pip install -r requirements.txt
```

### 4.4 Linux-only: serial-port permissions

The script will attempt to add your user to the `dialout` group via `pkexec` on first run. After that, **log out and log back in** so the group membership takes effect.

To do it manually:

```bash
sudo usermod -a -G dialout $USER
# log out / log back in
```

---

## 5. Channel & Slot Mapping

The MKS 946 organizes channels into three I/O slots, two channels each:

| Slot | Channels    | Default Display Names    |
| ---- | ----------- | ------------------------ |
| A    | 1 (A1), 2 (A2) | Mixture, Ch2          |
| B    | 3 (B1), 4 (B2) | Dilution2, Vapor2     |
| C    | 5 (C1), 6 (C2) | Vapor1, Dilution1     |

> **Critical:** the *channel number* the software sends to the 946 must match the *slot* where the MFC board is physically installed. If your MFC sits in Slot B, you must use channel **3** or **4**, regardless of how it is labelled in the GUI.

---

## 6. Launching the Application

### 6.1 Quick start (Linux)

1. Open the file manager and navigate to the folder containing `delivery.py`.
2. **Right-click in an empty area inside the folder** → choose **"Open Terminal Here"** (or **"Open in Terminal"**, depending on your desktop environment).
   - If your file manager doesn't have that option, open a terminal manually and `cd` into the folder, e.g.:
     ```bash
     cd ~/Downloads/delivery-script-1.7
     ```
3. Run the script:

   ```bash
   python3 delivery.py
   ```

   The GUI window will open within a few seconds.

> **Note:** use `python3 delivery.py` — **not** `python3 -m delivery.py`. The `-m` flag is for installed modules; running a script file directly does not need it.

### 6.2 Terminal-only mode (continuous flow readout, no GUI)

```bash
python3 delivery.py -t
# or
python3 delivery.py --terminal
```

Press **Ctrl+C** to stop terminal mode.

### 6.3 Windows users

Open PowerShell or Command Prompt, `cd` into the folder, and run:

```powershell
python delivery.py
```

---

## 7. Standard Operating Procedure (SOP)

Follow these steps every time you run an experiment.

### Step 1 — Power up the hardware

1. Power on the MKS 946 controller and wait for it to finish booting.
2. Confirm all carrier-gas / vapor sources are open and at their working pressures.
3. Connect the serial cable between the 946 and your PC.

### Step 2 — Launch the software

**Linux:** open the folder containing `delivery.py` in your file manager, right-click in empty space, and choose **"Open Terminal Here"**. Then run:

```bash
python3 delivery.py
```

**Windows:** open PowerShell, `cd` into the folder, and run `python delivery.py`.

The main window opens with **Status: Disconnected** (red).

### Step 3 — Select the COM/serial port

1. The **Port** dropdown auto-populates with detected ports.
2. If your port is missing, click **Scan Ports** (Linux: requires admin password to read `dmesg`).
3. Pick the port connected to the 946 (e.g. `COM3` on Windows, `/dev/ttyS4` or `/dev/ttyUSB0` on Linux).

### Step 4 — Connect

Click **Connect**. On success:

- The status bar turns **green** ("Status: Connected").
- All per-channel **Set** buttons and SCCM spinboxes become enabled.
- The PV (process value) labels start showing live readings.
- The plot begins updating.
- The current setpoint of each channel is read back from the 946 and shown in its spinbox.

### Step 5 — (Optional) Verify MFC presence

Click **Identify MFC Channels** to probe each channel. The dialog reports which channels respond to native 946 MFC commands (`FR`, `QSP`, `QMD`).

> If a channel does not respond, see [Section 12](#12-diagnostics--troubleshooting).

### Step 6 — (Optional) Zero channels at no-flow

With all upstream valves **closed** (no actual flow):

1. Pick the channel from the **Zero Channel** dropdown.
2. Click **Zero**.
3. Repeat for each channel you want zeroed.

See [Section 11](#11-zeroing-channels) for details.

### Step 7 — Set flow rates

You have two ways to set flows:

- **Manual:** Type the desired SCCM into each channel's spinbox, then click its **Set** button. The button briefly turns green on success or red on failure.
- **Recipe:** Click **Create Mixture** to use the recipe dialog (see [Section 9](#9-creating--using-mixture-recipes)).

### Step 8 — Monitor

- Watch the live plot for stability.
- Confirm PV readings approach the setpoints.
- Use **Monitor Flow Response** for a step-response view of any single channel.

### Step 9 — Record (optional)

1. Set **recording duration** (10–3600 s) next to the **Start Recording** button.
2. Click **Start Recording**. CSV is written to `recordings/flow_recording_<timestamp>.csv`.
3. The recording auto-stops after the duration, or you can press **Stop Recording** manually.

### Step 10 — Shutdown

1. Stop any active recording.
2. Set all channels to **0 SCCM** (or close upstream sources, then zero again if needed).
3. Click **Disconnect**.
4. Close the window.
5. Power down the 946 if you are done for the day.

---

## 8. GUI Reference

### Top-right: Connection panel

| Control      | Purpose                                                                                  |
| ------------ | ---------------------------------------------------------------------------------------- |
| Port (combo) | Lists available serial ports. Refreshed on launch and after **Scan Ports**.              |
| Connect      | Opens the serial connection. Becomes **Disconnect** while connected.                     |
| Scan Ports   | Re-enumerates serial ports (Linux: uses `lspci`/`dmesg`, may prompt for admin password). |
| Status       | Red = Disconnected, Orange = Connecting, Green = Connected, Red = Permission Error.      |

### Per-channel rows (six rows)

Each channel row contains:

| Element        | Purpose                                                                  |
| -------------- | ------------------------------------------------------------------------ |
| Color square   | Matches the curve color in the plot.                                     |
| Channel name   | Display name from `CHANNEL_NAMES`.                                       |
| SCCM spinbox   | Desired setpoint. Step = 0.1 SCCM.                                       |
| PV label       | Live process value polled at ~10 Hz.                                     |
| **Set** button | Sends the setpoint to the 946. Green = success, red = failure.           |

### Recipe Controls

| Button            | Purpose                                                                                  |
| ----------------- | ---------------------------------------------------------------------------------------- |
| Create Mixture    | Opens the recipe dialog (total flow + vapor ratios → auto-calculated dilution channels). |
| Load Recipe       | Loads a saved recipe CSV and re-opens the dialog populated with those values.            |
| Save Recipe       | Saves the **currently displayed** spinbox values to a CSV.                               |

### Zero & Diagnostics

| Control                  | Purpose                                                                          |
| ------------------------ | -------------------------------------------------------------------------------- |
| Zero Channel + dropdown  | Sends a zero command (`QZn`) to the selected channel.                            |
| Identify MFC Channels    | Probes channels 1–6 for MFC responsiveness, with per-command details.            |
| Diagnose Channel         | Runs a deep diagnostic on one channel (mode, setpoint, valve, slot mapping).     |
| Monitor Flow Response    | Sets a new setpoint on one channel and plots its real-time response.             |

### Recording

| Control                 | Purpose                                                          |
| ----------------------- | ---------------------------------------------------------------- |
| Start/Stop Recording    | Toggles CSV logging.                                             |
| Recording duration      | Auto-stop time in seconds (10–3600).                             |
| Status label            | "Recording: Ns" (red) → "Recorded Ns" (green) when complete.     |

---

## 9. Creating & Using Mixture Recipes

A "recipe" defines a vapor/dilution mixture targeting a desired **total flow** and **vapor percentages**. The dialog automatically computes dilution flows so each vapor + dilution pair sums to the total flow.

### 9.1 Pairing rules (built into the script)

| Vapor            | Paired Dilution        | Sum Constraint              |
| ---------------- | ---------------------- | --------------------------- |
| Ch5 (Vapor1)     | Ch6 (Dilution1)        | Vapor1 + Dilution1 = Total  |
| Ch4 (Vapor2)     | Ch3 (Dilution2)        | Vapor2 + Dilution2 = Total  |

Ch1 (Mixture) and Ch2 are **manual override** fields — the dialog does not auto-compute them.

### 9.2 Creating a recipe

1. Click **Create Mixture**.
2. Enter:
   - **Total Flow (SCCM)** — applies to each vapor/dilution pair.
   - **Vapor1 Ratio (%)** and **Vapor2 Ratio (%)**.
   - Optional **Mixture (Ch1) Override** and **Ch2 Override**.
3. The **Computed Channel Values** table updates live. Cells turn **red** if a value is negative or **orange** if it exceeds the channel's max (200 SCCM by default).
4. The warning area shows **"All values within limits."** in green when the recipe is safe to apply.

### 9.3 Applying a recipe

Click **Apply to Hardware**. The dialog:

1. Validates ranges; refuses to apply if anything is out of range.
2. Pushes each computed value into the matching spinbox in the main window.
3. Sends `QSPn` (set-setpoint) to the 946 for each channel.
4. Reports success or per-channel errors.

### 9.4 Saving a recipe

Click **Save Recipe** in the dialog (or **Save Recipe** on the main window for the current spinbox state). A CSV is written with the format:

```csv
channel,name,sccm,percentage
_meta,total_flow,100.0,
_meta,vapor1_pct,50.0,
_meta,vapor2_pct,50.0,
Ch1,Mixture,0.0,
Ch2,Ch2,0.0,
Ch3,Dilution2,50.0,
Ch4,Vapor2,50.0,50.0
Ch5,Vapor1,50.0,50.0
Ch6,Dilution1,50.0,
```

### 9.5 Loading a recipe

Click **Load Recipe**, select the CSV, and the recipe dialog reopens populated with the saved values. Review the preview, then apply.

---

## 10. Recording Flow Data

### 10.1 Starting

1. Set **duration** (default 60 s).
2. Click **Start Recording**.
3. A CSV is created in `recordings/flow_recording_<MM-DD-YYYY-HHMMam/pm>.csv`.

### 10.2 What's recorded

One row per second:

```csv
Time(s),Ch1,Ch2,Ch3,Ch4,Ch5,Ch6
1,0.0,0.0,49.8,50.1,49.9,50.2
2,...
```

Values are the **last polled PV** for each channel at the moment the row is written.

### 10.3 Stopping

- Recording stops automatically when the duration elapses.
- Or click **Stop Recording** to stop early.
- A confirmation dialog shows the file path on completion.

---

## 11. Zeroing Channels

Zeroing tells the MFC that current flow = 0 SCCM. **Only zero with no actual flow through the device.**

### Procedure

1. Close the upstream gas/vapor source for that channel.
2. Wait ~10 s for any residual flow to settle.
3. Pick the channel in the **Zero Channel** dropdown.
4. Click **Zero**. Wait for the success message.
5. Re-open upstream and confirm PV ≈ 0 with no setpoint applied.

If "Zero failed (NAK response)" appears, the controller did not accept the command — check the slot/channel mapping ([Section 5](#5-channel--slot-mapping)) and try **Diagnose Channel**.

---

## 12. Diagnostics & Troubleshooting

### 12.1 Identify MFC Channels

Probes channels 1–6 with `FR` (flow read), `QSP` (setpoint query), and `QMD` (mode query). Reports:

- Which channels responded.
- The full raw response of each command per channel.

Use this to confirm the 946 *sees* an MFC where you expect.

### 12.2 Diagnose Channel

Runs a thorough check on one channel:

- Slot mapping reminder.
- Flow-control support flag.
- Device type (legacy `FC 1.23` is reported by default).
- Current setpoint.
- Current MFC mode (`SETPOINT`, `OPEN`, `CLOSE`, etc.).
- Latest reading + delta from setpoint (flags >5 SCCM and >10% deviation).
- Valve mode (`Normal`, `Close`, `Open`).

### 12.3 Monitor Flow Response

Lets you change a channel's setpoint and watch its step response on a live plot. Useful for tuning, leak checks, and confirming a channel is responsive.

### 12.4 Common problems

| Symptom                                           | Likely Cause                                                   | Fix                                                                                          |
| ------------------------------------------------- | -------------------------------------------------------------- | -------------------------------------------------------------------------------------------- |
| Status stuck on "Connecting…" or jumps to red     | Wrong COM port / cable / 946 powered off                       | Check cable, power, retry **Scan Ports**.                                                    |
| All PVs read `---`                                | No serial response                                             | Confirm baud (9600), 8N1, controller address (253). Try another port.                        |
| "Monitor Only" label on a channel                 | No MFC detected on that channel                                | Verify MFC physically installed in the matching slot; use the right channel number.          |
| Setting flow returns NAK                          | Channel/slot mismatch, or MFC not in setpoint mode             | Run **Diagnose Channel**; confirm `QMD` reports `SETPOINT`.                                  |
| PV stays far from setpoint                        | Flow restriction, leak, valve closed upstream, calibration off | Check plumbing; try **Zero** with no flow; verify max range (200 SCCM by default).           |
| Permission Error (Linux)                          | User not in `dialout` group                                    | `sudo usermod -a -G dialout $USER`, then log out/in.                                         |
| Recording file is empty / zeros                   | Recording started before connecting / before PV was valid      | Connect first, wait until PVs update, then start recording.                                  |

### 12.5 Enabling verbose serial debug

Verbose serial logging is **on by default** (`DELIVERY_DEBUG_SERIAL=1`). To turn it off:

```bash
# Windows PowerShell
$env:DELIVERY_DEBUG_SERIAL = "0"
python delivery.py

# Linux / macOS
DELIVERY_DEBUG_SERIAL=0 python delivery.py
```

To override the controller address (default 253):

```bash
DELIVERY_CONTROLLER_ADDRESS=001 python delivery.py
```

---

## 13. Terminal-Only Mode

For headless monitoring or quick checks without launching the GUI:

```bash
python delivery.py -t
```

The script prints a line like:

```
Mixture: 0.0 | Ch2: 0.0 | Dilution2: 50.1 | Vapor2: 49.9 | Vapor1: 50.0 | Dilution1: 50.0
```

every ~100 ms. Press **Ctrl+C** to exit.

---

## 14. Configuration & Customization

Edit the constants at the top of `delivery.py` to tailor the app:

```python
DEFAULT_PORT = 'COM3' if IS_WINDOWS else '/dev/ttyS4'

CHANNEL_NAMES = {
    'Ch1': 'Mixture',
    'Ch2': 'Ch2',
    'Ch3': 'Dilution2',
    'Ch4': 'Vapor2',
    'Ch5': 'Vapor1',
    'Ch6': 'Dilution1',
}

CHANNEL_MAX_SCCM = {
    'Ch1': 200, 'Ch2': 200, 'Ch3': 200,
    'Ch4': 200, 'Ch5': 200, 'Ch6': 200,
}

TIME_WINDOW = 30          # seconds visible on the live plot
MAX_DATA_POINTS = 1000    # plot history depth
LEGACY_FIRMWARE_MODE = True   # treat all channels as MFCs even if probe is silent
```

> **Do not change** the `Ch1`…`Ch6` keys — they are required by the rest of the code. Only the display values (right-hand strings) are safe to edit.

Environment variables:

| Variable                       | Default | Purpose                                          |
| ------------------------------ | ------- | ------------------------------------------------ |
| `DELIVERY_DEBUG_SERIAL`        | `1`     | Set to `0` to silence per-command serial logging. |
| `DELIVERY_CONTROLLER_ADDRESS`  | `253`   | MKS 946 RS-232 address.                          |

---

## 15. Shutdown Procedure

1. **Stop recording** if active.
2. **Set all channels to 0 SCCM** (or close upstream and zero them).
3. Click **Disconnect** — wait until the status returns to red.
4. **Close the window** (the script also stops the worker thread on `closeEvent`).
5. Power down the MKS 946 if leaving the bench.

---

## 16. FAQ

**Q: I see "Status: Connected" but PV labels stay at `---`.**
A: The serial connection is open but the 946 is not responding to `FR` queries. Check controller address (env var `DELIVERY_CONTROLLER_ADDRESS`), wiring, and that the 946 is in RS-232 (not Ethernet) mode.

**Q: Can I change the max SCCM per channel?**
A: Yes — edit `CHANNEL_MAX_SCCM` at the top of `delivery.py`. The recipe dialog uses these values for range checking.

**Q: Where do recordings live?**
A: In a `recordings/` folder created next to `delivery.py`. The folder is created automatically.

**Q: Does the script work without the 946 connected?**
A: The GUI launches, but **Connect** will fail. Recipes can still be designed and saved; they cannot be applied until you connect.

**Q: Can I use this with newer 946 firmware?**
A: The script is set to **legacy mode** (`LEGACY_FIRMWARE_MODE = True`) so it treats unresponsive `QMD` queries as success. If you have modern firmware that always answers `QMD`, you can leave this `True` — it does not break anything; it just relaxes the strictness.

**Q: How do I reset everything if it gets stuck?**
A: Click **Disconnect**, wait for the red status, close the window, then re-launch `python delivery.py`. If the serial port is still locked by the OS, unplug/replug the cable or reboot.

---

*Last updated: 2026 — for `delivery.py` v1.7. For questions, contact the SENSE lab maintainer.*
