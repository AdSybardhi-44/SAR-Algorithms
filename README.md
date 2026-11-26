# SAR-Algorithms

The names are not what is actually going on, they are test files I am using to realize the scope of each method.

## Radar recordings with the linear stage

1. Start **OTA Hub** → *OTA Verification & Measurements* → **Linear Actuator**. MATLAB opens with the app.
2. In the app: port `COM4` → **Connect** (takes a while) → **Create stepper** → **Start** (Auto home off).
   Create stepper sets the position counter to 0 where the stage stands.
3. In the MATLAB Command Window:

   ```matlab
   addpath('C:\Users\kryeziuarber\Desktop\SAR-Algorithms'); actuator_logger
   ```

   It logs the stage position with UTC time stamps to `Results\actuator_<yyyyMMdd_HHmmss>.csv`
   until `actuator_logger stop` or until MATLAB is closed (`actuator_logger status` shows what it
   does). It can also be started before Connect; `help actuator_logger` describes the file.
4. Start the radar GUI (`.venv\Scripts\python.exe radar_gui.py`), then **Start** and **Record**
   (or *Save data*). When the recording starts, the radar clock is set to the PC clock and the log
   says whether the stage positions are being logged.
5. Move the stage (Jog, Go to, a plan). Do not use *Set home here* or *Auto home* during a recording:
   they change the coordinates.
6. Stop the recording: after saving, the log names the stage log that covers it and warns about gaps.

`radar_timestamps` of a recording and `timestamp_s` of the stage log are both POSIX seconds UTC of
the PC clock, so the stage position of every sweep is a linear interpolation:

```matlab
A = readtable('Results\actuator_20261120_141500.csv');
R = load('Results\acquisition_20261120_141530.mat');
t = R.radar_timestamps;
k = ~isnan(A.position_mm) & A.timestamp_s >= t(1) - 10 & A.timestamp_s <= t(end) + 10;
unique(A.segment(k))                                   % must be one segment
x = interp1(A.timestamp_s(k), A.position_mm(k), t);    % mm per sweep, NaN outside the log
```

In Python, `actuator_log.read_actuator_log` reads the stage log (see `actuator_log.py`).
`trx_stage_positions` does the interpolation for the TRx sweeps (trace 0) of a recording and plots
the positions, the stage speed and the range profiles over the position:

```matlab
S = trx_stage_positions();   % newest recording in Results and the stage log that covers it
S = trx_stage_positions('Results\acquisition_20261120_165016.mat', '', 'trx_positions.png');
```

For `Stripmap_SAR_TDBP.m`, copy the recording as `radar_data.mat` and the stage log of the same
measurement (the radar GUI names it after saving the recording) as `actuator_data.csv`. The script
saves the constant-velocity cruise portion of the stage scan during the recording to `actuator_data.mat`:

```matlab
load('radar_data.mat')
actuator_csv_to_mat('actuator_data.csv', 'actuator_data.mat', radar_timestamps)
load('actuator_data.mat')
mask = actuator_timestamps(1) <= radar_timestamps & radar_timestamps <= actuator_timestamps(end);
x = interp1(actuator_timestamps, actuator_position, radar_timestamps(mask));   % m per sweep
```

`actuator_timestamps` (POSIX s UTC), `actuator_position` (m) and `actuator_velocity` (m/s) cover
the longest constant-velocity segment during the recording, trimming out pre-/post-move idle polls,
startup acceleration, shutdown deceleration and pauses (pass `ConstantVelocity=false` to keep full
moves from rest to stop). The moves of the log and cruise-velocity statistics are listed in the
Command Window.

Checked with the radar on the stage: the logged positions match the motion the radar sees within
about 20 ms (0.05 mm at 2.5 mm/s). After the stage reverses its direction, the carriage lags the
logged position at the start of the move (backlash: the lag fell from about 0.3 s to 0.05 s within
5 s), so start a scan with some travel in its direction before the part you use. The log holds the
real counter positions: at 2.5 mm/s set, the stage ran at about 2.3 to 2.4 mm/s.
