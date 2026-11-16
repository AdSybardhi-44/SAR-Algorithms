# 2πSENSE X1000 — Experiment Plan

*1D range, antenna characterisation, 2D and 3D localisation: everything before Stripmap SAR*

126–182 GHz FMCW radar · 1 TRx + 2 Rx channels · lens antenna (≈ 3.5°) and VNA/calibration add-on · based on `test_x1000_matlab.m` and `SenseX1000_Python_API_Guide.md` · 10 November 2026

> **How the plan is organised.** Each phase builds on the previous one. Phase 0 checks the instrument itself. Phase 1 shows what a single channel measures: range, amplitude and phase. Phase 2 measures the antennas, which you need before you can combine channels. Phases 3 and 4 combine the three channels to get angle and 3D position. SAR experiments are deliberately left out.

## Key numbers

For the 126–182 GHz sweep used in your script. `c` is the speed of light and `N` the number of points per sweep (read it with `sense.sweep_points()`).

| Quantity | Formula | Value |
|---|---|---|
| Bandwidth B | `f_stop − f_start` | 56 GHz |
| Centre frequency f_c | `(f_start + f_stop)/2` | 154 GHz |
| Wavelength λ | `c/f_c` | 1.95 mm (2.38 mm at 126 GHz, 1.65 mm at 182 GHz) |
| Range resolution (rectangular window) | `c/2B` | 2.68 mm |
| Smallest separation of two equal targets (Hann window) | `≈ 2·c/2B` | ≈ 5.4 mm |
| Phase change with range | `4π/λ` | 360° per 0.97 mm, i.e. 0.37° per µm |
| Maximum range | `N·c/4B` | N × 1.34 mm |
| Lens beamwidth, one-way / two-way | — | ≈ 3.5° / ≈ 2.5° |
| Lens diameter D implied by 3.5° | `≈ (1.0–1.2)·λ/θ` | ≈ 3–4 cm |
| Distance where the lens beam starts to spread | `≈ 2D²/λ` | ≈ 1–1.5 m |

### What the three traces are

- **Trace 0, centre channel (TRx):** transmits and receives on the same antenna (monostatic). This is the channel your script uses (`data_ramp1(:,1,:)`).
- **Traces 1 and 2, Rx channels:** receive the signal transmitted by the TRx antenna (bistatic). Which physical antenna belongs to which trace is found in experiment 2.2.
- The IF samples are real, so only the first half of the FFT is unique. Your script already plots only that half.

## Phase 0 — Check the instrument (before any target)

### 0.1 Health check and the settings actually applied

- After connecting, log the PLL lock (main and aux), the reference-oscillator status, the temperature and `sense.dump()` (API guide §11.1).
- Setters return the value you sent, not the value the device applied. Read back `sweep_points`, `sweep_time` and the start and stop frequencies, and build the range axis from the frequency axis delivered with the data (as your script does), not from the requested values.
- Read, but don't change, the power level and the receiver coupling of each channel, and store them with every dataset.

### 0.2 Empty scene: internal reflections and noise floor

- Point the radar at absorber and record all three traces.
- Expect fixed peaks close to zero range: leakage from transmitter to receiver inside the module, and reflections from the lens surfaces (and from the add-on when it's fitted). Their positions show where range zero lies relative to the lens, and they hide targets very close to the lens.
- Save this as a complex background and subtract it, before taking the magnitude, in later experiments. Repeat it now and then to see how stable it is.
- Average 1, 4, 16 and 64 sweeps. In a static scene the noise floor should drop by `10·log₁₀(N)` dB. If it drops less, something in the chain is drifting or vibrating.

### 0.3 Warm-up and drift

- From a cold start, measure a fixed corner reflector every 10 s or so for an hour. Log the peak range, amplitude, phase and device temperature.
- Because the phase changes by 0.37° per µm, it also shows thermal expansion of the radar and the mount. This tells you how long to warm up, and whether you need a fixed reference reflector in the scene to correct drift.

## Phase 1 — 1D range

### 1.1 Range calibration: scale and zero

- Put a corner reflector on a rail at 10–20 known distances, from about 0.2 m to near the maximum range. Measure the true distance with a laser distance meter.
- Find each peak with sub-bin precision: zero-pad the FFT and fit a parabola through the three highest bins (`fast-plot.py` does the same).
- Fit `measured = a·true + b`.
  - `a` should be 1 to within about 10⁻³. A clear deviation means the frequency axis you use doesn't match the real sweep.
  - `b` is the offset between range zero (inside the radar) and your reference point, e.g. the front face of the lens.
  - Residuals that grow or oscillate with range point to a non-linear sweep or to multipath from the rail or table.

### 1.2 Range resolution and windows

- Use two small reflectors at almost the same range, e.g. two thin vertical wires or two small spheres staggered in depth, or a metal plate with machined steps of known height. Reduce the separation from about 2 cm down to 2 mm.
- Process every measurement with a rectangular, a Hann and a Blackman window, and note where the two peaks merge:
  - rectangular: ≈ 2.7 mm, sidelobes −13 dB
  - Hann: ≈ 5 mm, sidelobes −31 dB
  - Blackman: ≈ 6 mm, sidelobes −58 dB
- At each separation, shift one target by about λ/8 (0.25 mm) a few times. Whether two close peaks look resolved depends on the relative phase of the two echoes, not only on their separation.
- Check that a single target gives the same peak width at every range. If the peak widens with range, the sweep isn't perfectly linear.
- Process the two half-bands (126–154 and 154–182 GHz) separately. The peaks become twice as wide. You'll use this trade-off again in Phase 2 to see how things change with frequency.

### 1.3 Precision: peak position vs. phase

- Put a corner reflector on a micrometer stage at 0.5–1 m. Move it in 10–50 µm steps over about 2 mm, and repeat each position a few times.
- Estimate the displacement in two ways: from the interpolated peak position and from the phase of the peak.
- Expected precision at a peak SNR of 40 dB:
  - peak position: `σ ≈ 0.55·(c/2B)/√SNR` ≈ 15 µm
  - phase: `σ ≈ λ/(4π·√SNR)` ≈ 1.5 µm, about ten times better
- The phase wraps every λ/2 ≈ 0.97 mm of range. Either move less than λ/4 ≈ 0.49 mm between measurements so you can unwrap it, or let the peak position pick the right wrap and the phase give the fine value. Because the peak precision is far smaller than 0.97 mm, this gives absolute range with micrometre precision.
- Reference the phase to the centre of the sweep, as `fast-plot.py` does. The phase is then `4π·f_c·R/c` with f_c = 154 GHz, and it doesn't jump when the peak moves from one FFT bin to the next.
- If you reference the phase to the start of the sweep instead, the effective frequency is 126 GHz. Mixing the two conventions gives about a 20 % error in the displacement scale.
- **Why:** measuring displacement from phase is the basis for angle measurement between channels, and later for SAR.

### 1.4 Amplitude vs. range, and dynamic range

Measure the peak amplitude of a sphere, a corner reflector and a plate at different ranges.

- Small targets far from the lens fall off as R⁻⁴ in power.
- A plate larger than the beam acts like a mirror and falls off as R⁻².
- Close to the lens, where the beam is still a narrow column, the fall-off is weaker still.
- The receiver has a coupling mode labelled "AC = R² compensation"; query `control.radar_frontend_coupling` to see which mode is active. It is presumably an IF high-pass filter that boosts far targets, so know which mode you're in before you interpret amplitudes.
- Then bring a plate close to the radar and watch for clipping (raw int16 samples reaching ±32767) and for ghost peaks at 2R and 3R (harmonics). Harmonics come mainly from clipping of the raw IF. A real double bounce between the plate and the radar front also appears near 2R, but it is there without clipping.

### 1.5 Ambiguity limit

- Place a target just beyond the maximum range and see whether it folds back to a shorter range or disappears (removed by the IF filter).
- Repeat with a longer `sweep_time`. More points per sweep should give a larger maximum range.

### 1.6 Slow time (optional)

- Use multi-sweep acquisitions with a fixed `sweep_period`. Track the phase of a vibrating target, such as a speaker or a tuning fork, from sweep to sweep. This gives a vibration spectrum at µm amplitudes. The sweep period must be shorter than half the vibration period (for example, 0.2 ms sweeps every 0.5 ms cover vibrations up to 1 kHz).
- With `ALTERNATING` sweeps, the peak of a moving target shifts by `+v·f_c·T/B` in one sweep direction and by `−v·f_c·T/B` in the other (v = radial speed, T = sweep time). The difference between the up- and down-sweep ranges, `2·v·f_c·T/B`, gives you v (55 µm for 1 cm/s with 1 ms sweeps).

## Phase 2 — Understand the antennas (needed before 2D and 3D)

### 2.1 Beam pattern and footprint

Use a small sphere (it reflects the same from every angle) or a corner reflector.

- Scan it sideways across the beam at 0.2, 0.5, 1, 2 and 3 m. Alternatively, rotate the radar with the target at 1.5 m or more.
- Record the amplitude **and** the phase of all channels. Also process each part of the band separately, for example four 14 GHz sub-bands.

**What to expect**

- A lens focused at infinity produces a parallel beam. Within the first few tens of centimetres, the beam is a column about the size of the lens.
- Beyond about 1–1.5 m it spreads at about 2.5° (two-way). A 3.5° one-way beamwidth implies a lens about 3–4 cm across, so compare that with your lens.
- The beam narrows as frequency rises. A target off the beam axis is therefore seen mainly by the lower frequencies, which means less effective bandwidth and poorer range resolution at the beam edge.
- Far from the lens, the phase vs. sideways position follows `√(R² + x²)`. Near the lens it flattens because the wavefront is almost flat there. Fitting this curve tells you where the antenna's phase centre is.
- Keep this data, because the beam footprint is also what sets the SAR aperture length later.

### 2.2 Channel geometry

Repeat the scan both horizontally and vertically. Plot |Rx1|/|Rx2|, ∠Rx1 − ∠TRx and ∠Rx2 − ∠TRx against angle. This tells you:

- **Arrangement:** whether the antennas are in a line (Rx–TRx–Rx, which fits the "centre channel" label) or in an L-shape. Only an L-shape gives both horizontal and vertical angle from one measurement.
- **Spacing d:** the slope of the phase difference is `2π·d/λ` per radian. It also fixes your sign convention, because `ifft` and `fft` give opposite phase signs.
- **How angle is encoded:** if all three feeds sit behind the one lens, you get slightly tilted beams with almost no phase difference between them. Angle then has to be estimated from amplitude ratios (amplitude-comparison monopulse). Separate antennas, as with your add-on, encode angle in the phase difference (interferometry). I'd expect the first behaviour with the lens and the second with the add-on.
- **Polarisation:** rotate a thin metal strip in the beam. The return is strongest when the strip is parallel to the E-field.

### 2.3 Channel calibration

- Place a target straight ahead of the radar, far enough away that the beam is already spreading.
- For each channel, measure the range offset, the complex gain, and the full response across 126–182 GHz, so you can match the channels to each other.
- With the add-on, compare the peak width against the lens case. If the add-on contains waveguide sections, their dispersion broadens the peak and needs a frequency-dependent phase correction.
- Then place the target at 5–10 known positions and fit the antenna phase-centre positions by least squares. An error of only 0.1 mm in an antenna position already shifts the phase by about 18°.

## Phase 3 — 2D (range + angle)

### 3.1 Angle of a single target

Put a target on a rotary stage and compare estimated vs. true angle. From this you get the bias, the spread, and the usable field of view.

- **With the lens:** a very small field of view (about ±1–2°), but very precise. It is good for locating or tracking a target inside the beam.
- **With the add-on:** phase alone is unambiguous only within `±asin(λ/2d)`. That is ±5.6° for d = 10 mm and ±1.1° for d = 50 mm.
  - Your bandwidth solves this. The path difference between channels also shifts the range peak, which gives a coarse but unambiguous value. The phase then gives the fine value, ambiguous only in steps of λ.
  - This is the same coarse-range-plus-fine-phase idea as in 1D (experiment 1.3).
- **Expected precision:** `σ_θ ≈ λ/(2π·d·√SNR)`. That is about 0.06° for d = 1 cm at 30 dB SNR, or roughly 1 mm sideways at 1 m.
- **At short distance:** closer than `2d²/λ` (about 0.9 m for d = 3 cm), the usual far-field formula with sin θ is biased. Instead, compute the exact distances from each candidate target position to each antenna and keep the position that best matches the measured ranges and phases.

### 3.2 Multiple targets: the key limitation

- Place 3–5 spheres at different ranges and angles. Estimating one angle per range bin gives a range–angle point cloud.
- Then put two spheres at the *same* range but different angles. The angle estimate becomes a wrong, fluctuating mix of the two (called "glint").
- Separate them in range by more than 5 mm, and both angles come out right again.
- **Lesson:** three channels let you *measure* the angle of one dominant reflector per range bin. They cannot *separate* two reflectors at the same range. Your 2.7 mm range resolution is what makes this approach work on real objects.

### 3.3 Real-beam scanning

- Rotate the radar horizontally in steps of about 0.5° and stack the range profiles. The result is a range–azimuth image with 2.7 mm range resolution and about 2.5° angular resolution.
- Within each beam position, add the angle measurement from 3.1 to get accuracy finer than the beamwidth. Compare the result with the point cloud from 3.2.

## Phase 4 — 3D

### 4.1 3D from one measurement

Each channel measures one path length very precisely:

- The TRx channel places the target on a sphere around the TRx antenna.
- Each Rx channel places it on an ellipsoid whose foci are the TX and that Rx antenna.
- If the antennas are **not** in a line, the three surfaces meet at one point in front of the radar. That gives range, horizontal angle and vertical angle.
- If they **are** in a line, the possible positions form a circle around that line, so you only get one angle.
  - Workaround: take a second measurement with the radar rolled 90° about its boresight axis, with that axis passing through the TRx antenna's phase centre.
  - The TRx range profile must then look identical in both measurements, which also checks your phase-centre estimate.
  - That lets you match each reflector between the two measurements by its range.

### 4.2 3D test object

- Mount spheres or corner reflectors on Styrofoam (nearly transparent at these frequencies) at known x, y, z positions, a few centimetres apart in depth.
- Compare the reconstructed positions with the true ones. The differences give your 3D error budget.

### 4.3 Raster scanning

- Scan the lens beam in both directions with a pan-tilt mount. This gives a volume image with 2.7 mm depth resolution and about 2.5° angular resolution.
- Optionally, move the radar on an x–y grid with the object close to the lens (inside the parallel-beam zone) and simply stack the range profiles (no coherent combining). The sideways resolution will be at centimetre level. This is the baseline your SAR has to beat later.

## Equipment for the bench

- **Targets:** small trihedral corner reflectors (1–3 cm edge) and steel balls (5–20 mm).
- **Mounting:** Styrofoam target mounts and a rigid (not plastic) radar mount. Use absorber to cover metal posts and the table, or keep them out of the beam.
- **Positioning:** a rail with a scale, a micrometer stage and a rotary stage.
- **Ground truth:** a laser distance meter.

## Notes on your script

- For every acquisition, save the raw IF of all traces together with the frequency axis, settings, temperature and timestamp. Process offline, so you can rerun old experiments when your processing improves.
- Averaging the 16 sweeps with `mean` before the FFT is coherent averaging. That is fine for static scenes but smears anything that moves.
- `py.list({0,1,2})` passes MATLAB doubles, so the trace list is sent as floats (API guide §14, item 10). Pass `int32` values and check `acqData.trace_map` to be sure which column holds which trace.
- For now, only read the `device.control.*` settings (power, coupling, PLL) and don't change them, because they are undocumented.

## Looking ahead

> **SAR resolution with the lens.** With this lens, stripmap SAR resolution along the scan direction is limited to about `λ/(2θ)` ≈ 1.5–2 cm. Wide-beam antennas on the add-on could reach millimetre level, which is another reason to characterise both configurations in Phase 2.
>
> **Open inputs.** The add-on's antenna spacing and the stages you have available. With those, the numbers above turn into concrete settings for each experiment: sweep time, distances and step sizes.
