# 2πSENSE X1000 Python API — Complete Guide

**Library:** `twopilabs-sense-x1000` **v0.7.12** (the version installed in `SAR-Algorithms\.venv`) plus its helper library `twopilabs-utils-scpi` v0.7.1  
**Based on:** the official documentation (<https://2pi-labs.gitlab.io/python/twopilabs-sense-x1000/> — Introduction, Controlling, Configuring, Examples, API reference), the docstrings/source code of the installed package, and the bundled example scripts. Where behaviour is not obvious from the docs it was checked in the source and, for several points, by running the library offline against a simulated device.  
**Legend:** ⚠ = quirk/bug in v0.7.12 (see §14) · 📄 = not covered by the official online API reference

## Contents
1. [What the library is](#1-what-the-library-is)
2. [Installation & requirements](#2-installation--requirements)
3. [Quick start (Python and MATLAB)](#3-quick-start)
4. [Core concepts](#4-core-concepts)
5. [Package map](#5-package-map)
6. [`SenseX1000` — entry point](#6-sensex1000--entry-point)
7. [`SenseX1000ScpiDevice` — the device object](#7-sensex1000scpidevice--the-device-object)
8. [Subsystem reference](#8-subsystem-reference) — `core`, `sense`, `trigger`, `initiate`, `calc`, `memory`, `system`, `control`
9. [`SenseX1000Base` — constants, enums, headers, data containers](#9-sensex1000base--constants-enums-headers-data-containers)
10. [Supporting library `twopilabs.utils.scpi`](#10-supporting-library-twopilabsutilsscpi)
11. [Recipes](#11-recipes)
12. [Bundled examples](#12-bundled-examples)
13. [Using the library from MATLAB](#13-using-the-library-from-matlab)
14. [Known quirks & pitfalls (v0.7.12)](#14-known-quirks--pitfalls-v0712)

---

## 1. What the library is

`twopilabs.sense.x1000` is the official Python control library for 2π-LABS **2πSENSE X1000** series FMCW radar sensors. It wraps the device's SCPI command set in a high-level API: every SCPI subsystem is an attribute of the device object and every command is a method:

```
device.<subsystem>.<command>(...)        e.g.  device.sense.frequency_start(126e9)
```

It handles device discovery (USB and network), the connection, the SCPI text protocol, and parsing of the binary measurement data into NumPy arrays with full metadata. It runs on Windows, macOS and Linux.

## 2. Installation & requirements

| Item | Detail |
|---|---|
| Install | `python -m pip install twopilabs-sense-x1000` |
| Python | ≥ 3.7.4 (package metadata) |
| Required | `numpy`, `pyyaml`, `twopilabs-utils-scpi ≥ 0.6.3` (provides the transports; uses pyusb/libusb for USB, zeroconf for network discovery, pyserial for serial ports) |
| Optional | `h5py` — only for `AcqData.to_hdf5()` / `AcqData.from_hdf5()` |
| Example scripts need | `matplotlib`, `scipy`, `h5py`, `PySide2`/`PySide6`, `pyqtgraph` (none of these are installed in your `.venv`) |
| USB IDs | Vendor `0x1FC9`, Product `0x8271` |
| License | LGPLv3 |

Notes from the official docs:

- **ARM machines (e.g. Apple M1/M2):** the `libusb` PyPI package ships no ARM library — install libusb system-wide (`brew install libusb`, or `apt` on Ubuntu).
- **Linux, non-root USB access:** add a udev rule and put the user in `plugdev`, then replug the device and log out/in (or reboot). `MODE="0666"` would allow all users.
  ```bash
  echo SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"1fc9\", ATTRS{idProduct}==\"8271\", GROUP=\"plugdev\", MODE=\"0660\" | sudo tee -a /etc/udev/rules.d/99-twopilabs-sense-x1000.rules
  sudo udevadm control --reload && sudo udevadm trigger
  sudo adduser <user> plugdev
  ```
- **Your environment:** `h5py` is not installed in `.venv`, and on its Python 3.15 interpreter `pip install h5py` currently fails (no pre-built wheel, source build fails). Use a Python version with h5py wheels if you need HDF5 recording.

## 3. Quick start

### 3.1 Python

```python
from twopilabs.sense.x1000 import SenseX1000

devices = SenseX1000.find_devices()                 # USB + network discovery (takes ~2 s)
if not devices:
    raise SystemExit('No 2πSENSE X1000 found')

with SenseX1000.open_device(devices[0], timeout=5.0) as device:   # closed automatically
    print(device.core.idn())                        # [manufacturer, model, serial, firmware]

    device.core.rst()                               # reset the device
    device.core.cls()                               # clear status / error queue

    device.sense.frequency_start(126e9)             # Hz
    device.sense.frequency_stop(182e9)              # Hz
    device.sense.sweep_time(1e-3)                   # s
    device.sense.sweep_count(16)                    # sweeps per acquisition
    device.sense.sweep_period(2e-3)                 # s between sweep starts
    device.calc.trace_list([0, 1, 2])               # traces (receive channels) to transfer

    acq  = device.initiate.immediate_and_receive()  # arm, trigger, wait for data
    data = acq.read()                               # read it immediately (all sweeps)

    print(data.array.shape)                         # (n_sweeps, n_traces, n_points)
    print(data.header.freq_axis[[0, -1]])           # instantaneous frequency of first/last sample
```

Range profile, computed the same way as in the official examples:

```python
import numpy as np
c    = 299792458.0
x    = data.array[:, 0, :] / data.header.acq_dtype.info.mag    # first trace, full-scale units
win  = np.hanning(data.n_points)
nfft = 8192
spec = np.fft.fft(x * win, n=nfft, axis=-1) / win.sum()
dist = np.arange(nfft) / nfft / abs(data.header.freq_step) * c / 2   # metres
half = nfft // 2                                     # real IF data -> only the first half is unique
profile_db = 20 * np.log10(np.abs(spec[:, :half]) * 2)              # dBFS
```

### 3.2 MATLAB (same flow as `test_x1000_matlab.m`)

```matlab
pyenv("Version", "C:\path\to\.venv\Scripts\python.exe");   % before the first py.* call
x1000 = py.importlib.import_module('twopilabs.sense.x1000');

devices = x1000.SenseX1000.find_devices();             % py.list of ScpiResource objects
device  = x1000.SenseX1000.open_device(devices{1});    % first device found

device.sense.frequency_start(126e9);
device.sense.frequency_stop(182e9);
device.sense.sweep_time(1e-3);
device.sense.sweep_count(int32(16));                   % int32 -> Python int
device.calc.trace_list(py.list({int32(0), int32(1), int32(2)}));

acq     = device.initiate.immediate_and_receive();
acqData = acq.read();
raw     = double(acqData.array);        % n_sweeps x n_traces x n_points (or your np2mat2 helper)
f       = double(acqData.freq_axes{1}); % Hz, one value per sample of sweep 1
device.close();
```

See §13 for MATLAB-specific details.

## 4. Core concepts

### 4.1 Object model

```
SenseX1000.find_devices()   ──►  [ScpiResource, ...]      how to reach a device (USB-TMC, TCP/IP, USB-CDC, serial)
SenseX1000.open_device(r)   ──►  SenseX1000ScpiDevice     the connected device
        .core      ScpiCore      IEEE 488.2 common commands (*IDN?, *RST, *CLS, *OPC? ...)
        .system    ScpiSystem    info, clock / time zone, LAN configuration
        .memory    ScpiMemory    store / load configuration and state in non-volatile memory
        .control   ScpiControl   low-level hardware (power, PLLs, front-end, GPIO, accessory)
        .sense     ScpiSense     FMCW sweep configuration
        .trigger   ScpiTrigger   trigger source / count / delay / timer
        .initiate  ScpiInitiate  arming (single / continuous)
        .calc      ScpiCalc      trace selection and data read-out
device.initiate.immediate_and_receive()  ──►  SenseX1000.Acquisition   header + live data stream
acquisition.read()                       ──►  SenseX1000.AcqData       NumPy array + metadata
```

All nested types are defined in `SenseX1000Base`; because `SenseX1000` inherits from it, you reach them as `SenseX1000.SweepMode`, `SenseX1000.TrigSource`, `SenseX1000.AcqData`, …

### 4.2 Measurement principle (FMCW)

From the docs: the device emits a linear frequency sweep (*chirp*) at its RF port. Reflections arrive delayed; mixing the received signal with the transmitted one gives the intermediate-frequency (IF) signal, i.e. their frequency difference. Because instantaneous frequency and delay are linearly related, farther targets produce higher IF ("beat") frequencies. The raw data you read are these IF samples ("sweep domain").

Standard FMCW relations (general background, not from the docs):

- bandwidth `B = |f_stop − f_start|`, slope `S = B / T_sweep`
- range from beat frequency: `R = c · f_IF / (2 · S)`
- range resolution: `ΔR = c / (2B)` → 126–182 GHz (B = 56 GHz) gives ≈ 2.7 mm
- library-style distance axis after an `N_fft`-point FFT: `R_k = k / (N_fft · |freq_step|) · c/2`; with real IF data only `k < N_fft/2` is unique, so `R_max ≈ c / (4 · |freq_step|)`

### 4.3 Arming and triggering

The device follows the SCPI model *idle → initiated (armed) → trigger → acquisition* (docs):

- After power-up the device is idle. `initiate.immediate()` arms it for **one** trigger, after which it returns to idle automatically.
- The default trigger source is `IMMEDIATE` (always true), so arming starts the acquisition at once.
- One trigger produces one **acquisition** of `sweep_count` sweeps, spaced by `sweep_period`.
- Other sources: `TIMER` (internal timer), `EXTERNAL` (trigger input), `INTERNAL` (internal trigger signal). `trigger.count` = triggers per arming, `trigger.eventcount` = events needed per trigger, `trigger.delay` = trigger-to-start delay, `initiate.continuous(True)` = re-arm automatically.

### 4.4 Get/set convention and error checking

Almost every method follows one pattern:

```python
device.sense.sweep_count()      # no argument -> query (SENSE:SWEEP:COUNT?)  -> returns the device's value
device.sense.sweep_count(16)    # argument    -> set   (SENSE:SWEEP:COUNT 16) -> returns 16 (what you passed)
```

- **A setter returns the value you passed in, not what the device applied.** The device may adjust values (e.g. `sweep_time` changes `sweep_points`, `frequency_center` changes `frequency_span`), so query afterwards if you need the effective value.
- After most commands the library calls `device.raise_error()`, which reads `SYST:ERR:NEXT?` and raises `ScpiErrorException` if the device reported an error (e.g. a frequency outside the allowed band). Not checked: all `core.*` methods, `system.preset()`, `system.comm_lan_ip_reset()`, `system.error_next()`.
- Enum arguments are sent by **name** (`SweepMode.ALTERNATING` → `SENSE:SWEEP:MODE ALTERNATING`); replies are converted back to enum members.
- Units: frequencies in Hz, times in seconds.

### 4.5 Acquisitions and data layout

- An **acquisition** is everything produced by one trigger: a binary header (+ optional subheaders) followed by `sweep_count × n_traces × n_points` samples.
- `Acquisition` is a lazy reader bound to the connection; `read(n)` pulls `n` sweeps and returns an `AcqData`.
- `AcqData.array` has shape **(N_sweeps, N_traces, N_points)**. Raw IF data are normally `int16` (type `S16RLE`); divide by `header.acq_dtype.info.mag` (32768) to get full-scale units.
- Arrays are **read-only** views (`array.flags.writeable == False`); use `data.array.astype(float)` for a writable copy.
- **Traces** are the receive channels selected with `calc.trace_list`; `data.trace_map[i]` tells which trace number is stored in `array[:, i, :]`. The examples accept trace numbers 0–2 and note "0 → center channel".
- Per-sweep metadata: `seq_nums`, `sweep_dirs`, `timestamps`, `freq_axes`, `time_axes`.
- **Docs:** read the data right after initiating, otherwise the device's internal buffers can overflow.

## 5. Package map

| Module `twopilabs.sense.x1000.…` | Contents |
|---|---|
| `__init__` | exports `SenseX1000`, `SenseX1000ScpiDevice` |
| `x1000` | `SenseX1000` — entry class |
| `x1000_scpi` | `SenseX1000ScpiDevice` — device object |
| `x1000_base` | `SenseX1000Base` (constants, enums, header and data classes) and the flag `H5PY_AVAILABLE` |
| `scpi_core` | `ScpiCore` → `device.core` |
| `scpi_sense` | `ScpiSense` → `device.sense` |
| `scpi_trigger` | `ScpiTrigger` → `device.trigger` |
| `scpi_initiate` | `ScpiInitiate` → `device.initiate` |
| `scpi_calc` | `ScpiCalc` → `device.calc` |
| `scpi_memory` | `ScpiMemory` → `device.memory` |
| `scpi_system` | `ScpiSystem` → `device.system` |
| `scpi_control` | `ScpiControl` → `device.control` 📄 |

You never create the `Scpi*` subsystem objects yourself: `SenseX1000ScpiDevice` creates them, and each one only stores `self.device` and translates method calls into SCPI.

## 6. `SenseX1000` — entry point

`class SenseX1000(SenseX1000Base)` — *"Entry class for controlling 2πSENSE X1000 series devices"*. It is used only through its class methods, and it exposes all constants and nested types of `SenseX1000Base` (§9).

Internal attribute: `_RESOURCE_DEVICE_MAP = {ScpiResource: SenseX1000ScpiDevice}` maps a resource type to the device class that `open_device` creates.

### `SenseX1000.find_devices(**kwargs) -> list[ScpiResource]`

Searches all transports for X1000 devices and returns a list of `ScpiResource` objects (an empty list if nothing is found). It calls `SenseX1000ScpiDevice.find_devices`, which always applies these filters:

- USB: vendor ID `0x1FC9`, product ID `0x8271`
- Network (mDNS / DNS-SD): service `_scpi-raw._tcp`, instance name matching the regex `2πSENSE X1000.*`, domain `local` (default)

| Keyword | Meaning |
|---|---|
| `transports` | list of transport classes to search, e.g. `[ScpiUsbTmcTransport]` (USB only) or `[ScpiTcpIpTransport]` (network only). Default: all available, in this order: serial, TCP/IP, USB-TMC, USB-CDC |
| `dnssd_domains` | DNS-SD domains to search (default `['local']`) |
| `dnssd_ipversions` | `'4'`, `'6'` or `'46'` for the mDNS query (default `''` = zeroconf's default) |
| `dnssd_timeout` | seconds to wait for mDNS answers (default 2.0); every network search takes at least this long |

Notes:

- The same device can appear more than once (once per transport, and once per IP address on the network). Check `resource.resource_name`, or restrict `transports`.
- An error inside one transport (e.g. no USB backend) is logged as a warning and that transport is skipped.
- Per the example comments, discovery over Ethernet needs a link-local IPv6 or zeroconf IPv4 address. With a static IP, create the resource manually (see below). The device only answers mDNS if advertising is enabled (`system.comm_lan_mdns_sd`).

```python
from twopilabs.utils.scpi import ScpiUsbTmcTransport, ScpiTcpIpTransport
usb_only = SenseX1000.find_devices(transports=[ScpiUsbTmcTransport])
lan_only = SenseX1000.find_devices(transports=[ScpiTcpIpTransport], dnssd_timeout=3.0)
for r in usb_only:
    print(r.resource_name, r.serialnum, r.location)
```

### `SenseX1000.open_device(resource, **kwargs) -> SenseX1000ScpiDevice`

Opens the connection described by `resource` and returns the device object (§7).

| Keyword | Meaning |
|---|---|
| `timeout` | read timeout in seconds (default 5 s in every transport) |
| other keywords | passed to the transport, e.g. `ip_family` (TCP/IP), `transfer_size_max` (USB), pyserial options (serial) |

- Raises `TypeError` if `resource` is not a `ScpiResource`, and `ScpiTransportException` if the connection cannot be opened (device not found, unknown host, …).
- Use `with SenseX1000.open_device(...) as device:` or call `device.close()` yourself; an open USB connection keeps the interface claimed.
- Choose a `timeout` longer than the longest wait you expect: `initiate.immediate_and_receive()` blocks until the acquisition is complete (long multi-sweep acquisitions, waiting for an external trigger). The firmware-upgrade example uses 30 s.

Connecting without discovery (e.g. static IP address):

```python
from twopilabs.utils.scpi import ScpiResource, ScpiTcpIpTransport
res = ScpiResource(ScpiTcpIpTransport, ('192.168.12.5', 5025))     # (host, port); 5025 = default SCPI port
# equivalent, from a VISA resource string:
res = ScpiResource.from_resource_name('TCPIP::192.168.12.5::5025::SOCKET')
device = SenseX1000.open_device(res, timeout=10)
```

The official examples write `ScpiResource(ScpiTcpIpTransport, '169.254.112.162:5025')`. A `"host:port"` string connects fine, but `resource_name` is then wrong (⚠ §14), so the tuple form is preferable.

## 7. `SenseX1000ScpiDevice` — the device object

`class SenseX1000ScpiDevice(ScpiDevice)` — *"Device interface class for controlling Sense X1000 series devices using SCPI Interface"*. Returned by `SenseX1000.open_device()`.

### Attributes

| Attribute | Type | Purpose |
|---|---|---|
| `core` | `ScpiCore` | mandatory and optional common commands |
| `system` | `ScpiSystem` | housekeeping and global configuration |
| `memory` | `ScpiMemory` | non-volatile configuration/state storage |
| `control` | `ScpiControl` | low-level hardware functions |
| `sense` | `ScpiSense` | device-specific high-level (measurement) configuration |
| `calc` | `ScpiCalc` | post-acquisition processing and data read-out |
| `trigger` | `ScpiTrigger` | synchronising acquisitions with events |
| `initiate` | `ScpiInitiate` | initiating (arming) the trigger system |
| `resource` | `ScpiResource` | the resource the device was opened from |
| `transport` | transport object | the open connection (`None` after `close()`) |

### Methods

| Method | Description |
|---|---|
| `__init__(resource, **kwargs)` | Opens the transport (`kwargs` go to the transport) and creates the subsystem objects. Normally called for you by `SenseX1000.open_device`. |
| `get_info() -> dict` | Same as `system.info()`. Keys used by the examples: `HWTYPE`, `HWREVISION`, `ID`, `POWER` {`CURRENT`, `VOLTAGE`, `SOURCE`}, `ROSCILLATOR` {`DCTCXO`, `ENABLED`, `HOLDOVER`, `LOCK`, `SOURCE`}, `TEMP`, `USB`. Your X1155S-U (firmware 1.1.3) also reports `IMU` {`ACCELEROMETER` {`X`, `Y`, `Z`}}: raw counts, about 15700 counts = 1 g at rest, noise ≈ 10–20 counts. The query takes about 110 ms, during which the radar does not measure (⚠ §14). |
| `get_state() -> dict` | `{'SENSE': sense.dump()}` — snapshot of the measurement configuration (the HDF5 examples store it as YAML next to the data). |
| `find_devices(**kwargs)` *(classmethod)* | Discovery with the X1000 filters — see `SenseX1000.find_devices`. |
| `execute(header, param=None, result=type(None))` | Sends any raw SCPI command. `param` is a SCPI type **instance** appended after a space (`ScpiNumber(5)`, `ScpiString('x')`, …). `result` is a SCPI type **class** used to parse the reply (leave it out for commands without a reply). Returns the parsed object (`.as_string()`, `.as_int()`, …). |
| `check_error() -> Optional[ScpiEvent]` | Pops one entry from the device error queue (`SYST:ERR:NEXT?`); returns it if its code ≠ 0, else `None`. |
| `raise_error()` | Calls `check_error()` and raises `ScpiErrorException` if there was an error. |
| `close()` | Closes the transport. Also done by `with` and when the object is garbage-collected. |
| `is_open() -> bool` | `True` until `close()` has been called. |
| `reset()` | Calls `transport.reset()`, which no built-in transport implements in this version (raises `AttributeError`). Use `device.core.rst()` to reset the instrument. |
| `__enter__()` / `__exit__()` | Context-manager support (`with … as device:`); `__exit__` closes the device. |

Raw SCPI example (taken from `fw-upgrade.py`):

```python
from twopilabs.utils.scpi import ScpiString, ScpiNumber
fw   = device.execute('SYSTEM:INFO:FIRMWARE:VERSION?', result=ScpiString).as_string()
date = device.execute('SYSTEM:INFO:FIRMWARE:DATE?', result=ScpiString).as_string()
device.execute('SENSE:SWEEP:COUNT', param=ScpiNumber(8))
device.raise_error()
```

## 8. Subsystem reference

Notation: "Set / Query" shows the SCPI text that is sent. Unless stated otherwise the argument defaults to `None`, which means *query*.

### 8.1 `device.core` — `ScpiCore` (IEEE 488.2 common commands)

*"Class containing core SCPI commands"*. None of these methods check the error queue.

| Method | SCPI | Returns | Description |
|---|---|---|---|
| `idn()` | `*IDN?` | `list[str]` | Identification split at commas: `[manufacturer, model, serial number, firmware version]`. |
| `cls()` | `*CLS` | `None` | Clears all status data structures (including the error queue). |
| `rst()` | `*RST` | `None` | Stops all overlapped (asynchronous) commands and resets the device. The examples call `rst()` and `cls()` right after connecting. |
| `sav(slot=0)` | `*SAV <slot>` | `None` | Saves the system state into a memory register slot. |
| `rcl(slot=0)` | `*RCL <slot>` | `None` | Recalls the system state from a slot. |
| `sds(slot=0)` | `*SDS <slot>` | `None` | Sets the system state stored in a slot to defaults. |
| `trg()` | `*TRG` | `None` | Software trigger for a device that is initiated and waiting for a trigger. |
| `wai()` | `*WAI` | `None` | Waits until all overlapped commands have finished. |
| `opc()` | `*OPC?` | `bool` | Waits until all overlapped commands have finished, then returns `True`. |

### 8.2 `device.sense` — `ScpiSense` (measurement configuration)

*"Class containing SCPI commands concerning SENSE subsystem"*. This is the subsystem you use most.

| Method | Set / Query | Value | Description |
|---|---|---|---|
| `dump()` | `SENSE:DUMP?` | → `dict` | Complete SENSE configuration (the device sends YAML). Used by `get_state()`. |
| `frequency_cw(freq)` | `SENSE:FREQ:CW <f> HZ` / `SENSE:FREQ:CW?` | Hz | Continuous-wave frequency, used when `frequency_mode` is `CW`. |
| `frequency_start(freq)` | `SENSE:FREQ:START <f> HZ` / `…?` | Hz | Sweep start frequency. Start above stop gives a falling chirp (docs example: 182 → 126 GHz). |
| `frequency_stop(freq)` | `SENSE:FREQ:STOP <f> HZ` / `…?` | Hz | Sweep stop frequency. |
| `frequency_center(freq)` | `SENSE:FREQ:CENTER <f> HZ` / `…?` | Hz | Sweep centre frequency; the device adjusts the span to stay within the allowed limits. |
| `frequency_span(freq)` | `SENSE:FREQ:SPAN <f> HZ` / `…?` | Hz, signed | Sweep span: positive = up-slope, negative = down-slope; the device adjusts the centre to stay within limits. |
| `frequency_overrange(enabled)` | `SENSE:FREQ:OVERRANGE ON/OFF` / `…?` | `bool` | Enables/disables frequency over-range mode. ⚠ The query returns a `ScpiBool` object — wrap it in `bool()`. |
| `frequency_mode(mode)` | `SENSE:FREQ:MODE CW/SWEEP` / `…?` | `FrequencyMode` | `CW` = single frequency (zero span), `SWEEP` = normal FMCW operation. |
| `sweep_points(points)` | `SENSE:SWEEP:POINTS <n>` / `…?` | `int` | Points (ADC samples) per sweep; the device changes the sweep time to keep its fixed sample rate. |
| `sweep_time(time)` | `SENSE:SWEEP:TIME <t> S` / `…?` | s | Sweep (chirp) duration; the device changes the number of points to keep its sample rate. |
| `sweep_direction(direction)` | `SENSE:SWEEP:DIRECTION UP/DOWN` / `…?` | `SweepDirection` | Slope of the first sweep in an acquisition. |
| `sweep_mode(mode)` | `SENSE:SWEEP:MODE NORMAL/ALTERNATING` / `…?` | `SweepMode` | `NORMAL`: every sweep has the same slope; `ALTERNATING`: the slope alternates between consecutive sweeps. |
| `sweep_count(count)` | `SENSE:SWEEP:COUNT <n>` / `…?` | `int` | Number of sweeps per acquisition (i.e. per trigger). |
| `sweep_period(period)` | `SENSE:SWEEP:PERIOD <t> S` / `…?` | s | Time between sweep starts in multi-sweep acquisitions ("pulse repetition period", the inverse of the sweep repetition rate). |
| `refosc_source_selected(source)` | `SENSE:ROSC:SOURCE:SELECTED <name>` / `…?` | `RefOscSource` | Selects the reference-oscillator source (`NONE`, `INTERNAL`, `EXTERNAL`). |
| `refosc_source_current()` | `SENSE:ROSC:SOURCE:CURRENT?` | → `RefOscSource` | Source that is actually in use. |
| `refosc_status()` | `SENSE:ROSC:STATUS?` | → `RefOscStatus` | State of the reference PLL (`OFF`, `HOLDOVER`, `LOCKING`, `LOCKED`). |
| `refosc_internal_freq_adjust(adjustment)` 📄 | `SENSE:ROSC:INTERNAL:FREQ:ADJUST <x>` / `…?` | `float` | Fine adjustment of the internal reference oscillator. No docstring and no documented unit — leave it unchanged unless 2π-LABS tells you otherwise. |

Example from the docs (4 sweeps, 60 ms period, each sweeping 182 → 126 GHz in 1 ms):

```python
device.sense.frequency_start(182E9)
device.sense.frequency_stop(126E9)
device.sense.sweep_time(1E-3)
device.sense.sweep_count(4)
device.sense.sweep_period(60E-3)
print(device.sense.sweep_points())     # number of points the device chose for 1 ms
```

The allowed frequency range can be limited by frequency-band regulations (the range-Doppler example uses 126–148.5 GHz, commented "FCC Reduced BW Setting"). If the device rejects a value it reports a SCPI error, which the library raises as `ScpiErrorException`.

### 8.3 `device.trigger` — `ScpiTrigger`

*"Class containing SCPI commands concerning the TRIGGER subsystem"*.

| Method | Set / Query | Value | Description |
|---|---|---|---|
| `immediate()` 📄 | `TRIG:IMMEDIATE` | — | Forces a trigger right now. No docstring upstream; in standard SCPI this bypasses the selected trigger source. The device must be initiated. |
| `source(source)` | `TRIG:SOURCE <name>` / `TRIG:SOURCE?` | `TrigSource` | Trigger source: `IMMEDIATE` (default — triggers as soon as the device is initiated), `TIMER` (internal timer), `EXTERNAL` (trigger input), `INTERNAL` (internal trigger signal). |
| `count(count)` | `TRIG:COUNT <n>` / `…?` | `int` | Number of triggers (= acquisitions) per initiate (arming). |
| `eventcount(eventcount)` | `TRIG:ECOUNT <n>` / `…?` | `int` | Number of trigger events required to trigger one acquisition. |
| `delay(delay)` | `TRIG:DELAY <t>` / `…?` | s | Delay between a trigger and the start of the acquisition; reported back in the header as `trig_delay`. |
| `timer_period(period)` | `TRIG:TIMER:PERIOD <t>` / `…?` | `float` | Period of the internal timer used for triggering (unit not stated in the docstring). |
| `timer_compare(timestamp)` | `TRIG:TIMER:COMPARE <t>` / `…?` | `float` | Timestamp at which the internal timer triggers (unit/epoch not stated in the docstring). |

### 8.4 `device.initiate` — `ScpiInitiate`

*"Class containing SCPI commands concerning the INITIATE subsystem"*.

| Method | SCPI | Returns | Description |
|---|---|---|---|
| `immediate()` | `INIT:IMM` | `None` | Arms the device for a single trigger event (with source `IMMEDIATE` it triggers at once); afterwards it returns to idle. |
| `immediate_and_receive()` | `INIT:IMM`, then `*WAI;CALC:DATA?` | `Acquisition` (or `None`) | Convenience method: arm, wait for the trigger and the data, and return the acquisition object with its header already parsed. Same as `immediate()` + `core.wai()` + `calc.data()` but with fewer round trips. |
| `continuous(enabled)` | `INIT:CONT ON/OFF` / `INIT:CONT?` | `bool` | Enters/leaves continuous initiation: the device re-arms itself after every acquisition. |

```python
acq  = device.initiate.immediate_and_receive()
data = acq.read()        # read immediately — the docs warn that the device buffers can overflow otherwise
```

### 8.5 `device.calc` — `ScpiCalc` (traces and data read-out)

*"Class containing SCPI commands concerning CALCULATE subsystem"*.

| Method | SCPI | Returns | Description |
|---|---|---|---|
| `trace_list(traces)` | `CALCULATE:TRACE:LIST (0,1,2)` / `…?` | `list[int]` | Selects the traces (receive channels) that are processed and for which data are delivered. Pass Python ints. |
| `data(wait=True)` 📄 | `*WAI;CALC:DATA?` (or `CALC:DATA?` when `wait=False`) | `Acquisition` or `None` | Next acquisition as a streaming object; `None` if the device FIFO is empty. Only the header is read now; the sweeps are read from the connection by `Acquisition.read()`. |
| `data_all(acq_count=None, wait=True)` | `[*WAI;]CALC:DATA:ALL? [n]` | `list[Acquisition]` | All buffered acquisitions (or `acq_count` of them), transferred as one block into memory. Empty list if there are none. |
| `data_available(wait=True)` | `[*WAI;]CALC:DATA:AVAILABLE?` | `list[Acquisition]` | All acquisitions available right now, transferred into memory. Empty list if there are none. (The docstring mentions "given number", but there is no count parameter.) |

**Important:** an `Acquisition` returned by `data()` or `immediate_and_receive()` still has its sweep data waiting in the connection. Read it completely (`acq.read()`) before sending any other command through the same device object. Acquisitions from `data_all()` / `data_available()` are already in memory.

### 8.6 `device.memory` — `ScpiMemory`

*"Class containing SCPI commands concerning MEMORY subsystem"*. "Configuration" is the system configuration (for example the network settings); "state" is the system state (the same notion as `core.sav()` / `core.rcl()`). All methods return `None` and check the error queue.

| Method | SCPI | Description |
|---|---|---|
| `config_store(slot=0)` | `MEM:CONF:STORE <slot>` | Saves the current system configuration to non-volatile memory. |
| `config_load(slot=0)` | `MEM:CONF:LOAD <slot>` | Recalls the system configuration from non-volatile memory. |
| `config_clear(slot=0)` | `MEM:CONF:CLEAR <slot>` | Resets the stored configuration to defaults (the factory defaults then load at the next start). |
| `config_preset()` | `MEM:CONF:PRESET` | Loads the default configuration without altering non-volatile memory. |
| `state_store(slot=0)` | `MEM:STATE:STORE <slot>` | Saves the system state to a slot in non-volatile memory. |
| `state_load(slot=0)` | `MEM:STATE:LOAD <slot>` | Recalls the system state from a slot. |
| `state_clear(slot=0)` | `MEM:STATE:CLEAR <slot>` | Resets the state stored in a slot to defaults. |
| `state_preset(slot)` | `MEM:STATE:PRESET` | Loads the default state without altering non-volatile memory. ⚠ Requires a `slot` argument that is ignored — call `state_preset(0)`. |

Workflow from the docs: call `device.memory.config_store()` to make settings permanent. To return to factory defaults permanently, either call `config_preset()` and then `config_store()`, or call `config_clear()` and restart the device.

### 8.7 `device.system` — `ScpiSystem`

*"Class containing SCPI commands concerning SYSTEM subsystem"*.

General and clock:

| Method | Set / Query | Value | Description |
|---|---|---|---|
| `error_next()` 📄 | `SYST:ERR:NEXT?` | → `ScpiEvent` | Pops the next error-queue entry (code 0 = no error). Usually used via `device.check_error()`. |
| `info()` | `SYST:INFO?` | → `dict` | System information (YAML → dict); keys are listed under `get_info()` in §7. |
| `version()` | `SYST:VERSION?` | → `str` | Supported SCPI version. |
| `preset()` | `SYST:PRESET` | → `None` | Loads the system defaults (no error check). |
| `license_list()` | `SYST:LICENSE:LIST?` | → `dict` | Licenses of the software libraries used in the firmware. |
| `time(time)` | `SYST:TIME h,m,s` / `…?` | `datetime.time` | Device clock time. |
| `date(date)` | `SYST:DATE y,m,d` / `…?` | `datetime.date` | Device clock date. |
| `timezone(posix)` | `SYST:TZONE "<tz>"` / `…?` | `str` | Local time zone in POSIX 1003.1 format, e.g. `"CET-1CEST,M3.5.0,M10.5.0/3"`. |
| `utc(timestamp)` | `SYST:UTC <t>` / `…?` | `float` | Device clock as a UTC POSIX timestamp in seconds. The examples set it from the PC so that acquisition timestamps are meaningful: `device.system.utc(datetime.now(timezone.utc).timestamp())`. Measured on your X1155S-U: after power-up the clock reads 2000-01-01; the query returns whole seconds, but the set takes fractions: set from a high-resolution PC clock, the trigger timestamps agreed with it within about 1 ms (drift ≈ 6 ppm). |

LAN — the `ipversion` argument (default `SenseX1000.IPVersion.IPV4`) selects `IP4` or `IP6` in the command path, e.g. `SYST:COMM:LAN:IP6:ADDR`:

| Method | Set / Query | Value | Description |
|---|---|---|---|
| `comm_lan_ip_reset()` | `SYST:COMM:LAN:IP:RESET` | — | Resets the IP subsystem so IP configuration changes take effect. No error check (the connection may drop). |
| `comm_lan_ip_enable(enabled, ipversion)` | `SYST:COMM:LAN:IP4 ON/OFF` / `…?` | `bool` | Administrative status of the IPv4/IPv6 interface. |
| `comm_lan_ip_address(address, ipversion)` | `SYST:COMM:LAN:IP4:ADDR "<addr>"` / `…?` | `str` | IP address. |
| `comm_lan_ip_mask(mask, ipversion)` | `SYST:COMM:LAN:IP4:MASK "<mask>"` / `…?` | `str` | Subnet mask. |
| `comm_lan_ip_gateway(gateway, ipversion)` | `SYST:COMM:LAN:IP4:GATEWAY "<addr>"` / `…?` | `str` | Gateway address. |
| `comm_lan_ip_autoconf_enable(enabled, ipversion)` | `SYST:COMM:LAN:IP4:AUTOCONF ON/OFF` / `…?` | `bool` | IPv4 AutoIP (zeroconf) or IPv6 autoconfiguration (SLAAC-RA). |
| `comm_lan_ip_autoconf_status(ipversion)` | `SYST:COMM:LAN:IP4:AUTOCONF:STATUS?` | → `IPv4AutoconfStatus` / `IPv6AutoconfStatus` | Current autoconfiguration status. |
| `comm_lan_ip_dhcp_enable(enabled, ipversion)` | `SYST:COMM:LAN:IP4:DHCP ON/OFF` / `…?` | `bool` | DHCP on/off. ⚠ The query returns a `ScpiBool` object — wrap it in `bool()`. |
| `comm_lan_ip_dhcp_status(ipversion)` | `SYST:COMM:LAN:IP4:DHCP:STATUS?` | → `IPv4DHCPStatus` / `IPv6DHCPStatus` | Current DHCP client state. |
| `comm_lan_dns_servers(servers)` | `SYST:COMM:LAN:DNS:SERVERS "a,b"` / `…?` | `list[str]` | System-wide DNS servers. |
| `comm_lan_ntp_servers(servers)` | `SYST:COMM:LAN:NTP:SERVERS "a,b"` / `…?` | `list[str]` | System-wide NTP servers. |
| `comm_lan_ntp_sync()` | `SYST:COMM:LAN:NTP:SYNC?` | → `datetime.datetime` or `None` | Date/time of the last successful (S)NTP synchronisation (`None` = never). |
| `comm_lan_hostname(hostname)` | `SYST:COMM:LAN:HOSTNAME "<name>"` / `…?` | `str` | Host name. |
| `comm_lan_eui48()` | `SYST:COMM:LAN:EUI48?` | → `bytes` | MAC address, e.g. `device.system.comm_lan_eui48().hex(':')`. |
| `comm_lan_eui64()` | `SYST:COMM:LAN:EUI64?` | → `bytes` | EUI-64 hardware address. |
| `comm_lan_mdns_sd(enabled)` | `SYST:COMM:LAN:MDNS:SD ON/OFF` / `…?` | `bool` | mDNS service-discovery advertising (what `find_devices()` relies on over the network). |
| `comm_lan_lldp_advertising(enabled)` | `SYST:COMM:LAN:LLDP:ADV ON/OFF` / `…?` | `bool` | LLDP capability advertising. |

Network example from the docs (the default configuration is DHCPv4/SLAAC plus zeroconf):

```python
device.system.comm_lan_ip_address("192.168.12.5")    # IPv4 address
device.system.comm_lan_ip_mask("255.255.255.0")      # IPv4 netmask
device.system.comm_lan_ip_gateway("192.168.12.1")    # IPv4 gateway
device.system.comm_lan_ip_reset()                    # apply now (the connection may drop)
# make sure you can reach the new address, THEN make it permanent:
device.memory.config_store()
```

The docs warn: test the new address before storing it. With an invalid address/netmask (e.g. 192.168.55.0/255.255.255.0) the device may only be reachable through its IPv6 link-local address, which is always configured.

### 8.8 `device.control` — `ScpiControl` (low-level hardware) 📄

*"Class containing SCPI commands concerning CONTROL subsystem"*. This subsystem is not in the online reference and is not needed for normal measurements (`sense` configures the sweep for you). Docstrings are one-liners and most units are undocumented, so only change these settings if you know what they do. `ramp_idx` and `chan_idx` are inserted into the command (`RINC0`, `CHANNEL2`, …); `input_idx`, `output_idx` and `slot_num` are optional (⚠ the value `0` is treated as "not given").

Power and radar state:

| Method | SCPI | Value | Description |
|---|---|---|---|
| `power_level_selected(level)` | `CONTROL:POWER:LEVEL:SELECTED` | `PowerLevel` | Selected power level. |
| `power_level_detected()` | `CONTROL:POWER:LEVEL:DETECTED?` | → `PowerLevel` | Detected power level. |
| `power_level_current()` | `CONTROL:POWER:LEVEL:CURRENT?` | → `PowerLevel` | Currently effective power level. |
| `radar_state_selected(state)` | `CONTROL:RADAR:STATE:SELECTED` | `RadarState` | Selected state of the radar subsystem. |
| `radar_state_current()` | `CONTROL:RADAR:STATE:CURRENT?` | → `RadarState` | Current state of the radar subsystem. |

PLLs (main PLL = `MPLL`, auxiliary PLL = `APLL`):

| Method | SCPI | Value | Description |
|---|---|---|---|
| `radar_mainpll_nint(nint)` | `CONTROL:RADAR:MPLL:NINT` | `int` | Main PLL N divider, integer part. |
| `radar_mainpll_nfrac(nfrac)` | `CONTROL:RADAR:MPLL:NFRAC` | `int` | Main PLL N divider, fractional part. |
| `radar_mainpll_rampmode(mode)` | `CONTROL:RADAR:MPLL:RMODE` | `RampMode` | Ramp mode: `SINGLE`, `DOUBLE` or `ALTERNATING` ramps per trigger. |
| `radar_mainpll_rampincr(incr, ramp_idx=0)` | `CONTROL:RADAR:MPLL:RINC<i>` | `int` | Ramp increment of ramp `i`. |
| `radar_mainpll_ramplen(length, ramp_idx=0)` | `CONTROL:RADAR:MPLL:RLEN<i>` | `int` | Ramp length of ramp `i`. |
| `radar_mainpll_rampdwell(dwell, ramp_idx=0)` | `CONTROL:RADAR:MPLL:RDWELL<i>` | `int` | Ramp dwell of ramp `i`. |
| `radar_mainpll_locked()` | `CONTROL:RADAR:MPLL:LOCK?` | → `bool` | Main PLL lock status. |
| `radar_auxpll_nint(nint)` | `CONTROL:RADAR:APLL:NINT` | `int` | Aux PLL N divider, integer part. |
| `radar_auxpll_nfrac(nfrac)` | `CONTROL:RADAR:APLL:NFRAC` | `int` | Aux PLL N divider, fractional part. |
| `radar_auxpll_locked()` | `CONTROL:RADAR:APLL:LOCK?` | → `bool` | Aux PLL lock status (its docstring wrongly says "main PLL"). |

The examples print both PLL lock states as a health check after configuring the device.

Acquisition engine, front-end channels and processing table:

| Method | SCPI | Value | Description |
|---|---|---|---|
| `radar_acq_samplecount(samplecount)` | `CONTROL:RADAR:ACQ:SCOUNT` | `int` | Acquisition sample count. |
| `radar_acq_repeatcount(repeatcount)` | `CONTROL:RADAR:ACQ:RCOUNT` | `int` | Acquisition repeat count. |
| `radar_acq_repeatperiod(repeatperiod)` | `CONTROL:RADAR:ACQ:RPERIOD` | `int` | Acquisition repeat period. |
| `radar_frontend_enable(enabled, chan_idx=0)` | `CONTROL:RADAR:FRONTEND:CHANNEL<n>:ENABLE` | `bool` | Enables/disables front-end channel `n`. |
| `radar_frontend_force(force, chan_idx=0)` | `CONTROL:RADAR:FRONTEND:CHANNEL<n>:FORCE` | `ChannelForce` | Forces the channel always on or always off (`NONE`, `ON`, `OFF`). |
| `radar_frontend_power(power, chan_idx=0)` | `CONTROL:RADAR:FRONTEND:CHANNEL<n>:POWER` | `float` | TX power of channel `n`. |
| `radar_frontend_coupling(coupling, chan_idx=0)` | `CONTROL:RADAR:FRONTEND:CHANNEL<n>:COUPLING` | `ChannelCoupling` | Receive coupling: `GND`, `DC`, or `AC` (AC = R² compensation). |
| `radar_processing_offset_length(chan_idx=0)` | `CONTROL:RADAR:PROCESSING:OFFSET<n>:LENGTH?` | → `int` | Length of the processing offset table. |
| `radar_processing_offset_blocksize(chan_idx=0)` | `CONTROL:RADAR:PROCESSING:OFFSET<n>:BSIZE?` | → `int` | Block size of the processing offset table. |
| `radar_processing_offset_data(address, block=None, chan_idx=0)` | `…:OFFSET<n>:DATA <addr>,<block>` / `…:DATA? <addr>` | `bytes` | Writes/reads one block of the offset table at `address`. |

GPIO and accessory:

| Method | SCPI | Value | Description |
|---|---|---|---|
| `gpio_input_state(input_idx=None)` | `CONTROL:GPIO:IN[<n>]:STATE?` | → `bool` | State of a GPIO input. |
| `gpio_output_state(onoff, output_idx=None)` | `CONTROL:GPIO:OUT[<n>]:STATE` | `bool` | Sets/gets a GPIO output. |
| `accessory_enable(onoff)` | `CONTROL:ACCESSORY:ENABLE` | `bool` | Switches power to the device accessory on/off. |
| `accessory_rgb_slot(slot_num)` | `CONTROL:ACCESSORY:RGB:SLOT:NUM` | `int` | Selects the settings slot of the RGB accessory. ⚠ The query always returns `None`. |
| `accessory_rgb_hsb_value(hsb_value, slot_num=None)` | `CONTROL:ACCESSORY:RGB:HSB[<s>]:VALUE` | `[h, s, b]` | Hue/saturation/brightness (HSB = HSV) triplet in one command. |
| `accessory_rgb_hsb_rate(hsb_rate, slot_num=None)` | `CONTROL:ACCESSORY:RGB:HSB[<s>]:RATE` | `[h, s, b]` | Animation-rate triplet in one command. |

RGB accessory, per colour component. `<C>` is `HUE`, `SAT` or `BRIGHTNESS`; `slot_num=None` means the current slot. All take the same arguments as the hue variant shown in the first column. (The docstrings of the saturation/brightness variants say "hue" — copy/paste.)

| Operation | Hue | Saturation | Brightness | SCPI | Value |
|---|---|---|---|---|---|
| value | `accessory_rgb_hue_value(value, slot_num)` | `accessory_rgb_sat_value` | `accessory_rgb_brightness_value` | `CONTROL:ACCESSORY:RGB:<C>[<s>]:VALUE` | `float` |
| animation rate | `accessory_rgb_hue_rate(rate, slot_num)` | `accessory_rgb_sat_rate` | `accessory_rgb_brightness_rate` | `…:<C>[<s>]:RATE` | `float` |
| lookup-table data | `accessory_rgb_hue_table_data(table_data, slot_num)` | `accessory_rgb_sat_table_data` | `accessory_rgb_brightness_table_data` | `…:TABLE:DATA` (binary block) | `bytes` |
| lookup-table length | `accessory_rgb_hue_table_length(slot_num)` | `accessory_rgb_sat_table_length` | `accessory_rgb_brightness_table_length` | `…:TABLE:LENGTH?` | → `int` |
| tap indices into the table | `accessory_rgb_hue_tap_data(tap_data, slot_num)` | `accessory_rgb_sat_tap_data` | `accessory_rgb_brightness_tap_data` | `…:TAP:DATA` | `list[float]` |
| number of taps | `accessory_rgb_hue_tap_count(slot_num)` | `accessory_rgb_sat_tap_count` | `accessory_rgb_brightness_tap_count` | `…:TAP:COUNT?` | → `int` |
| spread taps evenly over the table | `accessory_rgb_hue_tap_spread(tap_spread, slot_num)` | `accessory_rgb_sat_tap_spread` | `accessory_rgb_brightness_tap_spread` | `…:TAP:SPREAD <x>` | set only; ⚠ value required |

## 9. `SenseX1000Base` — constants, enums, headers, data containers

Defined in `x1000_base.py`. Reach everything through `SenseX1000.<Name>` (e.g. `SenseX1000.TrigSource.EXTERNAL`).

### 9.1 Constants

| Name | Value | Meaning |
|---|---|---|
| `USB_VID` | `0x1FC9` | USB vendor ID used for discovery |
| `USB_PID` | `0x8271` | USB product ID used for discovery |
| `x1000_base.H5PY_AVAILABLE` (module flag) | `bool` | `True` if `h5py` could be imported (HDF5 support available) |

### 9.2 Configuration enums

Members are sent and received by **name**.

| Enum | Members (value) | Used by |
|---|---|---|
| `IPVersion` | `IPV4` (4), `IPV6` (6) | `ipversion=` of the `system.comm_lan_ip_*` methods |
| `IPv4AutoconfStatus` | `OFF`, `PROBING`, `ANNOUNCING`, `BOUND` | `system.comm_lan_ip_autoconf_status()` |
| `IPv6AutoconfStatus` | `NOTIMPLEMENTED` | same, for IPv6 |
| `IPv4DHCPStatus` | `OFF`, `REQUESTING`, `INIT`, `REBOOTING`, `REBINDING`, `RENEWING`, `SELECTING`, `INFORMING`, `CHECKING`, `PERMANENT`, `BOUND`, `RELEASING`, `BACKINGOFF` | `system.comm_lan_ip_dhcp_status()` |
| `IPv6DHCPStatus` | `NOTIMPLEMENTED` | same, for IPv6 |
| `PowerLevel` | `UNKNOWN`, `LOW`, `MEDIUM`, `HIGH`, `AUTO` | `control.power_level_*` |
| `RadarState` | `OFF` (0), `STANDBY` (11), `IDLE` (12), `BUSY` (15), `DETECTFAIL` (40), `MAGICFAIL` (45) | `control.radar_state_*` |
| `RampMode` | `SINGLE` (single ramp per trigger), `DOUBLE` (double ramps), `ALTERNATING` (alternating ramps) | `control.radar_mainpll_rampmode` |
| `ChannelCoupling` | `GND`, `DC`, `AC` (R² compensation) | `control.radar_frontend_coupling` |
| `ChannelForce` | `NONE` (don't force), `ON` (always on), `OFF` (always off) | `control.radar_frontend_force` |
| `FrequencyMode` | `CW` (single frequency, zero span), `SWEEP` (normal) | `sense.frequency_mode` |
| `SweepDirection` | `DOWN` (−1), `UP` (+1) — slope of the (first) sweep | `sense.sweep_direction` |
| `SweepMode` | `NORMAL` (constant slope, jumps back to the start frequency), `ALTERNATING` (slope alternates between sweeps) | `sense.sweep_mode` |
| `RefOscSource` | `NONE` (free running), `INTERNAL`, `EXTERNAL` (each if available) | `sense.refosc_source_*` |
| `RefOscStatus` | `OFF` (PLL disabled, e.g. power-off), `HOLDOVER` (source lost), `LOCKING`, `LOCKED`; `LOCK` is a deprecated alias of `LOCKED` | `sense.refosc_status` |
| `TrigSource` | `IMMEDIATE` (triggers right after initiate), `TIMER` (internal timer), `EXTERNAL` (trigger input), `INTERNAL` (internal trigger signal) | `trigger.source` |

### 9.3 Data types and flags

`AcqDTypeInfo(NamedTuple)` — fields `min`, `max`, `mag` (full-scale magnitude, used for normalisation) and `np` (NumPy dtype). You get it from `AcqDType.<X>.info` or `header.acq_dtype.info`.

`AcqDType(Enum)` — sample format of an acquisition (`header.data_type`). Property `info` → `AcqDTypeInfo`.

| Member | Value | Format | `info.np` | `info.mag` |
|---|---|---|---|---|
| `S16RLE` | 0 | signed 16-bit real, little endian (raw IF data) | `int16` | 32768 |
| `S16RILE` | 4 | signed 16-bit (real, imaginary) | structured `re`, `im` (int16) | 32768 |
| `S32RLE` | 8 | signed 32-bit real | `int32` | 2³¹ |
| `F32RLE` | 10 | float 32-bit real | `float32` | 1.0 |
| `S32RILE` | 12 | signed 32-bit (real, imaginary) | structured int32 | 2³¹ |
| `F32RILE` | 14 | float 32-bit (real, imaginary) | structured float32 | 1.0 |
| `S64RLE` | 16 | signed 64-bit real | `int64` | 2⁶³ |
| `F64RLE` | 18 | float 64-bit real | `float64` | 1.0 |
| `S64RILE` | 20 | signed 64-bit (real, imaginary) | structured int64 | 2⁶³ |
| `F64RILE` | 22 | float 64-bit (real, imaginary) | structured float64 | 1.0 |
| `F64DTMALE` | 32 | one tuple (distance, propagation time, magnitude, angle) | structured `dist`, `time`, `mag`, `angle` (float64) | — |
| `F64DTMA2LE`, `F64DTMA3LE`, `F64DTMA4LE`, `F64DTMA5LE`, `F64DTMA6LE` | 33, 34, 35, 36, 37 | 2, 3, 4, 5, 6 such tuples per point | sub-array of that structure | — |

For the complex `…RILE` types build a complex array with `a['re'] + 1j * a['im']`. The library has no command that selects the data type; your X1155S-U (firmware 1.1.3) always delivers `S16RLE` sweep-domain data (flags `SDOMAIN|DIRECTION`, subheaders timing, time axis, frequency axis and environmental info, no supplemental data).

`AcqFlags(Flag)` — `header.acq_flags`:

| Flag | Bit | Meaning |
|---|---|---|
| `INVALID` | 1<<1 | configuration and/or data are invalid for processing |
| `INFINITE` | 1<<2 | acquisition contains an infinite number of sweeps |
| `SDOMAIN` | 1<<4 | sweep-domain data (raw IF samples) |
| `RDOMAIN` | 1<<5 | range-domain data |
| `DIRECTION` | 1<<8 | slope of the first sweep: 0 = falling, 1 = rising |
| `ALTERNATING` | 1<<9 | 0 = constant slope, 1 = alternating slope in multi-sweep acquisitions |

`AcqSubheaderFlags(IntFlag)` — which subheaders follow the base header: `ACQTIMING` (1<<4, timestamps), `TIMEAXIS` (1<<8), `FREQAXIS` (1<<9), `ENVINFO` (1<<11, environmental info), `SUPPDATA` (1<<12, supplemental data).

### 9.4 Subheaders

Created by the library when parsing; you see them as `header.subhdr_*`. All are NamedTuples with internal helpers (`_struct`, `_npdtype`, `_flag`, `_from_stream`, `_from_array`, `_to_stream`, `__array__`) for binary parsing and HDF5 serialisation.

| Class | Fields | Extras |
|---|---|---|
| `AcqSubheaderAcqTiming` | `trigger_timestamp` (ns UTC since 1970-01-01), `trigger_delay` (ns), `sweep_period` (ns) | — |
| `AcqSubheaderTimeAxis` | `start`, `stop` (s) | property `delta = stop − start` |
| `AcqSubheaderFreqAxis` | `start`, `stop` (Hz) | property `delta` (its docstring says "time axis" — copy/paste) |
| `AcqSubheaderEnvironmentalInfo` | `items`: 4 × `AcqSubheaderEnvironmentalInfoItem` | property `sensors` → list of dicts; `_n_items()` = 4 |
| `AcqSubheaderEnvironmentalInfo.AcqSubheaderEnvironmentalInfoItem` | `address0`, `address1`, `address2`, `address3`, `address4`, `address5`, `age`, `rssi`, `battery`, `temperature`, `humidity`, `pressure` | property `address` → 6-tuple |
| `AcqSubheaderSupplementalData` | `items`: 4 × `AcqSubHeaderSupplementalDataItem` | `_n_items()` = 4 |
| `AcqSubheaderSupplementalData.AcqSubHeaderSupplementalDataItem` | `data_points`, `data_size`, `data_type`, `data_flags` | property `dtype` → `AcqDType` |

### 9.5 `AcqBaseHeader` and `AcqHeader`

`AcqBaseHeader(NamedTuple)` — *"the header prepended to every acquisition"* (binary layout `<HBBLLLLLBBLH`, 32 bytes; header version must be 1, otherwise parsing raises `AssertionError`).

| Field | Meaning |
|---|---|
| `header_length` | binary header size including subheaders (bytes) |
| `header_id` | binary header identifier |
| `header_version` | binary header version |
| `flags` | acquisition flags as an integer → see `acq_flags` |
| `bytes_total` | total number of bytes in the acquisition |
| `sweep_count` | number of sweeps in the acquisition |
| `trace_mask` | bit mask of the enabled traces |
| `data_points` | number of points per trace |
| `data_size` | size of one point in bytes |
| `data_type` | data-type identifier → see `acq_dtype` |
| `acq_index` | index (counter) of this acquisition |
| `subheader_flags` | which subheaders are included |
| `subhdr_acq_timing`, `subhdr_time_axis`, `subhdr_freq_axis`, `subhdr_env_info`, `subhdr_supp_data` | the subheaders (or `None`) |

Internal: `_struct()`, `_version()`, `_from_stream()`, `_from_dict()`, `_to_stream()`.

`AcqHeader(AcqBaseHeader)` — the header you actually get as `acq.header` / `data.header`. It adds these properties:

| Property | Type | Meaning |
|---|---|---|
| `trace_map` | `list[int]` | trace numbers enabled in this acquisition (decoded from `trace_mask`) |
| `trace_count` | `int` | number of enabled traces |
| `trace_size` | `int` | bytes per trace (including supplemental data) |
| `sweep_size` | `int` | bytes per sweep |
| `acq_dtype` | `AcqDType` | sample data type (`.info.mag` for normalisation) |
| `acq_flags` | `AcqFlags` | decoded flags |
| `trig_timestamp` | `np.datetime64` (ns) | trigger time (NaT without timing subheader) |
| `trig_delay` | `np.timedelta64` (ns) | trigger delay |
| `sweep_period` | `np.timedelta64` (ns) | sweep period |
| `time_axis` | `np.ndarray` or `None` | `linspace(start, stop, data_points)` in s — sample time of each point (read-only) |
| `freq_axis` | `np.ndarray` or `None` | instantaneous transmit frequency of each point in Hz (read-only) |
| `time_step` | `float` or `None` | "delta-time per point" = `delta / data_points` (⚠ slightly different from the axis spacing, §14) |
| `freq_step` | `float` or `None` | "delta-frequency per point" = `delta / data_points`; negative for falling sweeps |
| `environmental_info` | `dict` or `None` | `{"aa:bb:cc:dd:ee:ff": {age, rssi, battery, temperature, humidity, pressure}}` for every valid sensor |

Internal: `_subheaders()`, `_from_stream()`, `_from_dict()`, `_to_stream()`.

### 9.6 `AcqData` — the data container

`AcqData(NamedTuple)` — *"an acquisition data object holding data for one or more consecutive sweeps"*. Returned by `Acquisition.read()`, `Acquisition.data()` and `AcqData.from_hdf5()`.

| Field | Type | Meaning |
|---|---|---|
| `header` | `AcqHeader` | copy of the acquisition header |
| `array` | `np.ndarray` | the data, shape `(N_sweeps, N_traces, N_points)`, read-only |
| `supplemental_data` | `list` of `np.ndarray` or `None` | supplemental arrays, shape `(N_sweeps, N_traces, item.data_points)` |
| `n_sweeps` | `int` | sweeps in this container (`array.shape[0]`) |
| `n_traces` | `int` | traces per sweep (`array.shape[1]`) |
| `n_points` | `int` | points per trace (`array.shape[2]`) |
| `trace_map` | `list` | trace number for each index along axis 1 |
| `seq_nums` | `range` | sequence number of each sweep within the whole acquisition |
| `sweep_dirs` | `list[int]` | slope of each sweep (+1 rising / −1 falling); alternates in `ALTERNATING` mode |
| `timestamps` | `list[np.datetime64]` | start of each sweep, ns UTC = `trig_timestamp + trig_delay + seq_num × sweep_period` |
| `time_axes` | `list[np.ndarray]` | time axis of each sweep |
| `freq_axes` | `list[np.ndarray]` | frequency axis of each sweep (reversed for odd sweeps in alternating sweep-domain acquisitions) |

**`to_hdf5(parent, name=None, format='normal', **kwargs) -> h5py.Group`** — stores the container (data plus the full header) in an HDF5 group.

- `parent`: an `h5py.File` / `h5py.Group`, or a filename (opened in append mode `'a'`; ⚠ the library never closes that file, so prefer passing your own `h5py.File` in a `with` block).
- `name`: name of the new group; default `"<trig_timestamp>[<first seq_num>:<last seq_num + 1>]"`.
- `format`: `'normal'` or `'verbose'` (verbose adds `freq_axis` / `time_axis` dimension scales, per-sweep axis references and extra attributes).
- `**kwargs`: passed to `create_dataset` (e.g. `compression='gzip'`).
- Layout: group attributes = header fields; dataset `acquisition_data` (dimensions labelled sweep/trace/point); datasets `supplemental_data[i]`; dimension scales `seq_nums`, `sweep_dirs`, `timestamps` (int64 ns) and `trace_map`.
- Raises `RuntimeError` if h5py is missing and `ValueError` for an invalid `format`.

**`from_hdf5(parent, name, **kwargs) -> AcqData`** *(classmethod)* — rebuilds an identical `AcqData` from a group written by `to_hdf5`, so the same processing code works on live and recorded data. `parent` is an h5py group/file or a filename (opened read-only). Raises `TypeError` if the group is not a stored `AcqData`, `ValueError` if the header information is missing, and `RuntimeError` without h5py.

Internal: `_from_stream()`, `_to_stream()`, `_create()`.

### 9.7 `Acquisition` — reading an acquisition

`class Acquisition` — *"Container class representing an entire acquisition"*. Produced by `initiate.immediate_and_receive()`, `calc.data()`, `calc.data_all()` and `calc.data_available()`.

| Member | Description |
|---|---|
| `header` *(property)* | the `AcqHeader` of this acquisition |
| `sweeps_remaining` *(property)* | number of sweeps that can still be read |
| `seq_num` *(property)* | sequence number of the next sweep to be read |
| `read(n_sweeps=-1) -> AcqData` | reads `n_sweeps` sweeps (default `-1` = all remaining; capped at what remains). For `INFINITE` acquisitions `n_sweeps` must be given, otherwise `ValueError`. |
| `data(n_sweeps=1)` | generator that yields `AcqData` chunks of `n_sweeps` sweeps until everything is read (never ends for `INFINITE` acquisitions) |
| `_from_stream(stream)`, `__init__(header, stream)` | internal construction |

```python
acq = device.initiate.immediate_and_receive()
for chunk in acq.data(n_sweeps=32):        # process a big acquisition piece by piece
    process(chunk.array)                   # chunk.seq_nums tells you which sweeps these are
```

## 10. Supporting library `twopilabs.utils.scpi`

The device layer is built on `twopilabs-utils-scpi` (v0.7.1 in your `.venv`). You meet these classes when you choose transports, connect manually, send raw SCPI or handle errors. Import them with `from twopilabs.utils.scpi import ScpiResource, ScpiTcpIpTransport, ScpiUsbTmcTransport, ScpiString, …`; the exceptions live in `twopilabs.utils.scpi.scpi_exceptions`.

### 10.1 `ScpiResource` (NamedTuple)

Fields: `transport` (transport class), `address`, `location`, `name`, `manufacturer`, `model`, `serialnum`, `info`.

| Member | Description |
|---|---|
| `discover(transports=None, **kwargs)` *(classmethod)* | Generic discovery over the given (default: all) transports; `SenseX1000.find_devices` calls it with the X1000 filters. |
| `from_resource_name(resource_name, transports=None)` *(classmethod)* | Builds a resource from a VISA resource string; raises `ValueError` if no transport accepts it. |
| `resource_name` *(property)* | VISA resource string of this resource. |
| `resource_class`, `resource_type`, `resource_info` *(properties)* | constant strings `'ScpiResource'`, `'SCPI'`, `'SCPI Resource'` |

VISA resource strings understood by `from_resource_name`:

| Transport | Format | Example |
|---|---|---|
| TCP/IP | `TCPIP::<host>::<port>::SOCKET` (IPv6 host in `[...]`) | `TCPIP::192.168.12.5::5025::SOCKET` |
| USB-TMC | `USB::<vid>::<pid>::<serial>[::<interface>]::INSTR` | `USB::0x1fc9::0x8271::<serial>::INSTR`; without serial: `USB::0x1fc9::0x8271::::INSTR` |
| USB-CDC | `CDC::<vid>::<pid>::<serial>[::<interface>]::INSTR` | `CDC::0x1fc9::0x8271::<serial>::INSTR` |
| Serial | `ASRL<n>::INSTR` | `ASRL3::INSTR` → `COM3` |

### 10.2 `ScpiDevice`

Base class of `SenseX1000ScpiDevice`. It provides `execute`, `check_error`, `raise_error`, `close`, `is_open`, `reset` and the context-manager methods — see §7.

### 10.3 SCPI data types

Used as `param=` (an instance) or `result=` (the class) of `device.execute()`. All derive from `ScpiTypeBase`, which defines `compose()` (object → bytes to send) and the classmethod `parse(stream)` (reply → object).

| Type | Wire format | Accessors |
|---|---|---|
| `ScpiChars(s)` | mnemonic text, e.g. `ALTERNATING` | `as_string()`, `as_bytes()` |
| `ScpiNumber(value, unit=None, base=10)` | ints as NR1 (prefix `#H` / `#Q` / `#B` for base 16 / 8 / 2), floats with 15 significant digits, optional unit (`HZ`, `S`) | `as_int()`, `as_float()`, `unit`, `base`, `int()`, `float()` |
| `ScpiNumberArray(numbers)` | comma-separated numbers | `as_int_list()`, `as_float_list()` |
| `ScpiString(data)` | quoted string `"..."` | `as_string()`, `as_bytes()` |
| `ScpiBool(onoff)` | `ON` / `OFF` (reply `1` / `0`) | `as_bool()`, `bool()` |
| `ScpiNumList(numlist)` | `(0,1,2)` | `as_list()` |
| `ScpiArbBlock(data)` | definite-length block `#<n><len><bytes>`, read completely into memory | file-like `read()`, `seek()`, `readall()`; `as_bytes()`, `as_memoryview()` |
| `ScpiArbStream(stream, length, indefinite=False)` | arbitrary block read lazily from the connection (used for `CALC:DATA?`) | `read()`, `readall()` |
| `ScpiEvent(code, description, info=None)` | error-queue entry, e.g. `-222,"Data out of range;details"` | `code`, `description`, `info`, `as_string()` |

### 10.4 Exceptions (`twopilabs.utils.scpi.scpi_exceptions`)

| Exception | Raised when |
|---|---|
| `ScpiException` | generic SCPI error (base class) |
| `ScpiErrorException(scpi_error)` | the device reported an error; `e.scpi_error` is the `ScpiEvent` (`code`, `description`, `info`) |
| `ScpiTransportException` | the connection cannot be opened (device not found, unknown host, …) |

Timeouts during communication come from the transport layer: e.g. `UsbTmcTimeoutException` (USB) or a socket timeout (TCP/IP).

```python
from twopilabs.utils.scpi.scpi_exceptions import ScpiErrorException
try:
    device.sense.frequency_start(500e9)
except ScpiErrorException as e:
    print(e.scpi_error.code, e.scpi_error.description, e.scpi_error.info)
```

### 10.5 Transports

| Class | `discover(...)` keywords | Constructor | Notes |
|---|---|---|---|
| `ScpiUsbTmcTransport` | `usb_vid`, `usb_pid` | `(address: UsbTmcDeviceAddress, timeout=5, **kwargs)` | main USB transport (libusb) |
| `ScpiTcpIpTransport` | `dnssd_services`, `dnssd_domains`, `dnssd_names`, `dnssd_ipversions`, `dnssd_timeout` | `(address, timeout=5.0, ip_family=AF_UNSPEC)`; `address` = `(host, port)` or `"host[:port]"` | `DEFAULT_PORT = 5025`; mDNS discovery via zeroconf |
| `ScpiUsbCdcTransport` | `usb_vid`, `usb_pid` | `(address: UsbCdcDeviceAddress, timeout=5, **kwargs)` | USB CDC (virtual serial) via libusb |
| `ScpiSerialTransport` | `usb_vid`, `usb_pid` | `(port: str, timeout=5, **pyserial_kwargs)` | COM ports via pyserial |

Every transport also provides `from_resource_name()`, `to_resource_name()` and file-like `read()`, `write()`, `readline()`, `writeline()`, `close()`.

Low-level packages, normally not used directly:

- `twopilabs.utils.usbtmc`: `UsbTmcDevice` (`list_devices`, `get_backend`, `get_capabilities`, `clear`, `trigger`, `locate`, `read`, `write`, `readline`, `close`), `UsbTmcDeviceAddress(vid, pid, serial_number, interface_number)`, `UsbTmcDeviceInfo`, and the exceptions `UsbTmcException`, `UsbTmcConnectionException`, `UsbTmcStatusException`, `UsbTmcTimeoutException`, `UsbTmcUnsupportedException`.
- `twopilabs.utils.usbcdc`: `UsbCdcDevice`, `UsbCdcDeviceAddress`, `UsbCdcDeviceInfo`, and the exceptions `UsbCdcException`, `UsbCdcConnectionException`, `UsbCdcTimeoutException`.

### 10.6 Logging

The library logs through Python's `logging` module: at INFO level every SCPI exchange (`exec [...]: SENSE:FREQ:START (...) -> (...)`), at DEBUG level the raw transport traffic. The examples switch this on with `-v` / `-vv`:

```python
import logging
logging.basicConfig(level=logging.INFO)
```

## 11. Recipes

### 11.1 Health check after connecting (as done in every example)

```python
device.core.rst(); device.core.cls()
print(device.core.idn())
info = device.get_info()
print(info['HWTYPE'], info['HWREVISION'], info['TEMP'], info['ROSCILLATOR']['LOCK'])
print('Main PLL lock:', device.control.radar_mainpll_locked())
print('Aux PLL lock :', device.control.radar_auxpll_locked())
print('Ref. osc.    :', device.sense.refosc_source_current(), device.sense.refosc_status())
print('Config       :', device.sense.dump())
```

### 11.2 Multi-sweep acquisition with precise timing (range-Doppler style)

```python
import numpy as np
device.sense.frequency_start(126e9); device.sense.frequency_stop(148.5e9)
device.sense.sweep_time(0.2e-3);    device.sense.sweep_period(0.5e-3)
device.sense.sweep_count(100);      device.sense.sweep_mode(SenseX1000.SweepMode.NORMAL)
device.calc.trace_list([0])

data   = device.initiate.immediate_and_receive().read()
x      = data.array[:, 0, :] / data.header.acq_dtype.info.mag              # slow time x fast time
t_slow = np.array(data.seq_nums) * (data.header.sweep_period / np.timedelta64(1, 's'))   # s
prf    = np.timedelta64(1, 's') / data.header.sweep_period                 # sweep repetition rate, Hz
```

The range-Doppler example then windows both axes and applies a 2-D FFT (fast time → range, slow time → Doppler).

### 11.3 Alternating up/down sweeps

```python
device.sense.sweep_mode(SenseX1000.SweepMode.ALTERNATING)
device.sense.sweep_count(4)
data = device.initiate.immediate_and_receive().read()
for i in range(data.n_sweeps):
    f     = data.freq_axes[i]       # already reversed for the falling sweeps
    slope = data.sweep_dirs[i]      # +1 rising, -1 falling
```

### 11.4 One acquisition per position (e.g. SAR scanning)

Software-triggered — move, then measure:

```python
for pos in positions:
    move_to(pos)                                          # your positioner
    data = device.initiate.immediate_and_receive().read() # default source IMMEDIATE
    store(pos, data)
```

Hardware-triggered — the positioner fires the trigger input:

```python
device.trigger.source(SenseX1000.TrigSource.EXTERNAL)
for pos in positions:
    acq  = device.initiate.immediate_and_receive()        # blocks until the external trigger fires
    data = acq.read()
```

The examples offer exactly this loop with `--trigsource EXTERNAL`. Open the device with a `timeout` longer than the longest wait for a trigger. `trigger.delay`, `trigger.eventcount`, `trigger.count`, `initiate.continuous` and `calc.data_all()` / `calc.data_available()` are the building blocks for more elaborate schemes (e.g. buffering several acquisitions) — test the buffering behaviour with your firmware.

### 11.5 Timestamps for synchronisation

```python
from datetime import datetime, timezone
device.system.utc(datetime.now(timezone.utc).timestamp())   # sync the device clock first
data = device.initiate.immediate_and_receive().read()
print(data.timestamps[0])        # numpy datetime64[ns]: start of sweep 0 (UTC)
```

### 11.6 Recording and replaying (HDF5, needs h5py)

```python
import h5py, yaml
with h5py.File('run.h5', 'w') as f:
    g = f.create_group('/session1')
    g.attrs.create('info',  yaml.dump(device.get_info()))
    g.attrs.create('state', yaml.dump(device.get_state()))
    for k in range(10):
        data = device.initiate.immediate_and_receive().read()
        data.to_hdf5(g)                                   # one auto-named sub-group per acquisition

with h5py.File('run.h5', 'r') as f:
    g = f['/session1']
    for name in g.keys():
        data = SenseX1000.AcqData.from_hdf5(g, name)      # identical AcqData object
```

### 11.7 Other tasks

- Network configuration and making it permanent: §8.7 and §8.6.
- Raw SCPI commands (e.g. firmware version): §7.
- Error handling: §10.4.
- Connecting to a static IP without discovery: §6.

## 12. Bundled examples

Installed with the package in `.venv\Lib\site-packages\examples\` (the online docs show the same scripts):

| Script | What it shows | Extra packages |
|---|---|---|
| `live-plot.py` | matplotlib live plot of the raw IF sweep and the FFT range profile in dBFS; CLI options `--fstart --fstop --tsweep --nsweeps --tperiod --traces --sweepmode --trigsource --timeout -v` | matplotlib, scipy |
| `fast-plot.py` | responsive Qt GUI: range and phase plots, peak detection with quadratic interpolation, range history, optional HDF5 recording | PySide2/6, pyqtgraph, scipy, h5py (recording) |
| `range-doppler.py` | 2-D range-Doppler image from 100 sweeps (0.2 ms sweeps, 0.5 ms period, 126–148.5 GHz) | PySide2/6, pyqtgraph, scipy |
| `hdf5-record.py` | records `AcqData` plus device info/state (YAML attributes) into an HDF5 file | h5py |
| `hdf5-replay.py` | loads the recording with `AcqData.from_hdf5()` and plots it | h5py, matplotlib, scipy |
| `fw-upgrade.py` 📄 | firmware upgrade through raw SCPI (`DEVICE:IMAGE:*` commands) — use with care | — |

Processing details worth reusing:

- normalise: `data.array / data.header.acq_dtype.info.mag`
- distance axis: `np.linspace(0, 1/abs(data.header.freq_step), nfft, endpoint=False) * c/2`
- IF-frequency axis: `np.linspace(0, 1/abs(data.header.time_step), nfft, endpoint=False)`
- magnitude in dBFS: `20*log10(abs(FFT) / sum(window) * 2)`
- `fast-plot.py` uses an inverse FFT plus the linear-phase correction `exp(-1j*pi*k*(N-1)/nfft)`, which references the phase to the centre of the sweep, before plotting the phase.

## 13. Using the library from MATLAB

`test_x1000_matlab.m` uses MATLAB's Python interface. Things to watch:

1. **Interpreter:** `pyenv("Version", ...)` must run before the first `py.` call (restart MATLAB to change it). Your script points to `C:\twoipilabsd\.venv\Scripts\python.exe`; this repository's `.venv` (Python 3.15) also contains `twopilabs-sense-x1000` 0.7.12. Each MATLAB release supports only certain Python versions.
2. **Numbers:** a MATLAB `double` becomes a Python `float`. That is fine for frequencies and times. For integer arguments (sweep counts, trace numbers, slots, indices) pass `int32(...)` (or `py.int(...)`), e.g. `sweep_count(int32(16))` and `trace_list(py.list({int32(0), int32(1), int32(2)}))`. With doubles, `trace_list` sends `(0.0,1.0,2.0)` instead of `(0,1,2)`.
3. **Enums and keyword arguments:**
   ```matlab
   SX = x1000.SenseX1000;
   device.sense.sweep_mode(SX.SweepMode.ALTERNATING);
   device.trigger.source(SX.TrigSource.EXTERNAL);
   device  = SX.open_device(devices{1}, pyargs('timeout', 10));
   usb     = py.list({py.twopilabs.utils.scpi.ScpiUsbTmcTransport});
   devices = SX.find_devices(pyargs('transports', usb));   % USB only, skips the 2 s mDNS wait
   ```
4. **Arrays:** `acqData.array` is a read-only NumPy array (`int16` for raw IF data) of size n_sweeps × n_traces × n_points. Recent MATLAB releases convert it with `double(acqData.array)`; otherwise keep your `np2mat2` helper (it is not part of the library or of this repository). `acqData.freq_axes{1}` / `acqData.time_axes{1}` are the axes of the first sweep.
5. **Indexing:** trace numbers are 0-based (Python), MATLAB indices are 1-based. `raw(:,1,:)` is the first entry of `trace_map` (trace 0 when `[0,1,2]` is selected).
6. **Scaling:** divide by `double(acqData.header.acq_dtype.info.mag)` (32768) for full-scale units.
7. **Other conversions:** `cell(pyList)`, `struct(pyDict)`, `string(pyStr)`, `double(pyFloat)`.
8. **Clean-up:** call `device.close()` when finished; otherwise the USB interface stays claimed until the object is cleared.
9. **Range axis:** your `0.5*((1:NFFT)-1)/NFFT*3e8/(f(2)-f(1))` is correct (it uses the exact sample spacing of the frequency axis). Use `abs(f(2)-f(1))` if you sweep downwards.

## 14. Known quirks & pitfalls (v0.7.12)

Found by reading the installed source; most were also reproduced by running the library against a simulated device.

| # | Where | Issue | Workaround |
|---|---|---|---|
| 1 | all setters | Return the value you passed, not the value the device applied. | Query afterwards. |
| 2 | `sense.frequency_overrange()`, `system.comm_lan_ip_dhcp_enable()` | The query returns a `ScpiBool` object instead of `bool`. | Wrap it in `bool()`. |
| 3 | `control.accessory_rgb_slot()` | The query always returns `None`. | `device.execute('CONTROL:ACCESSORY:RGB:SLOT:NUM?', result=ScpiNumber).as_int()` |
| 4 | `memory.state_preset(slot)` | Requires an argument that is ignored. | `state_preset(0)` |
| 5 | `control.accessory_rgb_*_tap_spread()` | Annotated as optional, but a value is required. | Always pass a value. |
| 6 | `gpio_*`, `accessory_*` index arguments | `0` is treated as "not given" (`GPIO:OUT:STATE` instead of `GPIO:OUT0:STATE`). | Keep in mind that index 0 addresses the default port / current slot. |
| 7 | `header.time_step`, `header.freq_step` | Computed as `delta / N`, while the axes use the `linspace` spacing `delta / (N−1)`. Negligible for large N. | For exact spacing use `np.diff(header.freq_axis[:2])` (as your MATLAB script does). |
| 8 | `ScpiResource(ScpiTcpIpTransport, "host:port")` | Connects, but `resource_name` becomes garbage (`TCPIP::1::6::SOCKET`). | Use a `(host, port)` tuple or `from_resource_name`. |
| 9 | `from_resource_name('USB::0x1fc9::0x8271::INSTR')` | "INSTR" is parsed as the serial number. | Use `USB::0x1fc9::0x8271::::INSTR` or include the serial. |
| 10 | `calc.trace_list()` with floats | Sends `(0.0,1.0,2.0)`. | Pass ints (`int32` in MATLAB). |
| 11 | `Acquisition` from `calc.data()` / `immediate_and_receive()` | Its data are still in the connection; sending another command before reading breaks the protocol stream. | Call `acq.read()` first. |
| 12 | `INFINITE` acquisitions | `read()` needs `n_sweeps`, `sweeps_remaining` never decreases, and `data()` never ends. | Loop over `read(n)` with your own stop condition. |
| 13 | `device.reset()` | No built-in transport implements `reset()`. | Use `device.core.rst()`. |
| 14 | `AcqData.to_hdf5("file.h5")` | Opens the file in append mode and never closes it. | Pass an open `h5py.File`. |
| 15 | `find_devices()` | The same device can appear several times; transport order is serial → TCP/IP → USB-TMC → USB-CDC. | Filter with `transports=` or check `resource_name`. |
| 16 | `core.*`, `system.preset()`, `system.comm_lan_ip_reset()` | Do not check the device error queue. | Call `device.raise_error()` yourself if needed. |
| 17 | docstrings | Copy/paste errors: `radar_auxpll_locked` ("main PLL"), sat/brightness methods ("hue"), `AcqSubheaderFreqAxis` ("time axis"), `data_available` ("given number"). | — |
| 18 | `SenseX1000ScpiDevice` | `memory` is missing from the class annotations (the attribute exists). | — |
| 19 | `ChannelForce.ON` | Its value is the tuple `(1,)` (trailing comma in the source). | Harmless — names are sent on the wire. |
| 20 | online API reference | No page for `scpi_control`; methods without docstrings (`calc.data`, `trigger.immediate`, `system.error_next`, `sense.refosc_internal_freq_adjust`) are not listed. | Covered in this guide. |
| 21 | HDF5 in your `.venv` | h5py is not installed and `pip install h5py` fails on its Python 3.15. | Use a Python version with h5py wheels. |
| 22 | Applied frequencies (X1155S-U, fw 1.1.3) | The device rounds start/stop for the sweep time in effect when they are set: 126–182 GHz is applied as 126–181.998116 GHz for a 1 ms sweep, but as 126–181.990665 GHz if the frequencies were set while a 0.5 ms sweep time was active and the time was changed to 1 ms afterwards. | Set `sweep_time` before the frequencies (as `radar_setup.py` does) and read the applied values back with `sense.dump()`. |
| 23 | Unknown queries (X1155S-U, fw 1.1.3) | A query the firmware does not know (e.g. `SYST:INFO:IMU?`, `SYST:INFO:TEMP?`, `SYST:HELP:HEAD?`) gets no reply; the read times out, the USB-TMC abort times out too, and every later command fails. (`SYSTEM:INFO:FIRMWARE:VERSION?` works.) | Close and reopen the device. |
| 24 | `get_info()` / `SYST:INFO?` (X1155S-U, fw 1.1.3) | Takes about 110 ms; it is the only source of the accelerometer, so every accelerometer read-out pauses the measurements (111 → 8 frames/s when read after every frame). | Do not query it during measurements (`radar_setup.py` reads it once, for the health report). |
| 25 | `time.time()` on Windows with Python 3.12 | Advances in 15.6 ms steps (`GetSystemTimeAsFileTime`), so consecutive frames can get the same PC time stamp. | Use the radar trigger timestamps (as the recordings do), or `time.perf_counter()` for intervals. |
