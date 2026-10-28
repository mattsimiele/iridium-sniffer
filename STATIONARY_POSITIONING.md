# Experimental stationary IRA positioning

This branch repairs the Doppler estimator and supplies observation replay through
exactly the same C engine as live reception. It does **not** establish a reliable
field position fix, receiver velocity, or absolute timing. Receiver motion must be
negligible over the 600-second observation window. Height aiding uses **WGS84
ellipsoidal height**, not mean sea level. The default live height remains 0 m for
compatibility; explicitly supply a surveyed height.

## Build and test without an SDR

Requirements: C99 compiler, pthreads, libm, make, Python 3 (standard library only).

```sh
make -f Makefile.doppler test
./build-doppler/test build-doppler/synthetic.csv
python3 tools/replay.py build-doppler/synthetic.csv --height 100 > build-doppler/fixes.jsonl
python3 tools/score_replay.py build-doppler/fixes.jsonl --truth 40 -75 100 --max-error 5000
```

The synthetic fixture uses independently evaluated circular trajectories, Earth
rotation, constant receiver frequency bias, and 4 km XYZ quantization. Its orbit
planes are deliberately diverse and are **not a simulation of actual Iridium
coverage**. Measurements are noiseless apart from orbit quantization. This is a
regression fixture, not an accuracy forecast. The current reference run gives
about 897 m final 3D error; valid replay epochs have about 1.60 km median horizontal
error and 4.99 km maximum. The 5 km test limit is a regression threshold, not a
field specification. One quantization-affected endpoint is rejected even in this
fixture, illustrating why the residual gates still need capture calibration.

Tests cover signed time differences, fixed IRA carrier, analytic velocity versus
orbit fit (including endpoints), finite-difference Doppler derivatives, duplicate
and invalid observations, sat ID zero, out-of-order insertion, quantized full
solve, outlier removal/refit counts, rank loss, reset, stale output, repeatable
replay, sorting, malformed CSV, and truth scoring. CI additionally builds the full
application on Linux and runs the solver on Linux/macOS with ASan/UBSan.

## Record and replay

Build the application normally (FFTW required; suitable SDR backend for live use).
Add these options to the normal receive command:

```sh
--position=100 --position-log=stationary.csv
```

The log path must not exist. It is flushed after each orbital IRA observation;
write failures stop the process. Keep raw IQ and the receiver configuration as
well: observation replay starts **after demodulation** and cannot reveal a biased
PLL, lost samples, or incorrect timestamps by itself.

```sh
python3 tools/replay.py stationary.csv --height 100 > fixes.jsonl
python3 tools/score_replay.py fixes.jsonl --truth LAT LON ELLIPSOID_HEIGHT --max-error 5000
```

Replace the example height and truth with independently surveyed values. Truth
latitude/longitude enter only the scorer. Height is explicit external aiding.
Scoring exits nonzero when there are no valid fixes or any valid epoch exceeds
the selected horizontal error limit. Inspect availability and vertical errors
separately; a low median from a few accepted epochs does not establish reliability.

CSV schema (exact header):

```text
timestamp_ns,sat_id,x4km,y4km,z4km,frequency_hz,sigma_hz
```

Times are integer nanoseconds in one continuous capture time scale; they need not
be UTC. XYZ fields are the raw signed broadcast coordinates in 4 km units.
Frequency is the demodulator's absolute observed carrier in Hz. Live sigma is
currently a **placeholder 50 Hz**, not an estimated per-burst uncertainty.
Replay validates CSV, stable-sorts by event time, solves every 10 seconds (adjust
with `--interval`), at the final observation, and 31 seconds later to exercise
staleness. Replay's event-time scheduling differs from live wall-clock scheduling;
identical recorded observations produce deterministic replay, not necessarily
identical live intermediate estimates. The replay loads the file into memory.

## Estimator changes

- Use the IRA nominal carrier 1,626,270,833.333... Hz directly, without rounding
  Doppler-shifted observations onto an incompatible channel grid.
- Subtract timestamps before floating conversion without unsigned underflow.
- Fit local cubic trajectories after rotating each broadcast position into a
  common inertial frame. Convert velocity back to ECEF with Earth rotation once.
- Use a stationary ECEF range-rate model and its consistent analytic Jacobian.
- Estimate a common range-rate frequency bias with position. This is not clock
  offset or an absolute time solution; satellite transmitter offsets remain a risk.
- Use scaled, damped least squares with objective acceptance and independent
  undamped geometry checks. Keep an immutable row extent and active masks during
  outlier refits. Require a refit before accepting the candidate.
- Report explicit collecting/valid/stale/poor_geometry/rejected states. Failed
  solutions contain no coordinates; the live map clears the receiver marker.
- Replace the misleading HDOP display with conditional horizontal sigma in metres.
  This covariance assumes independent errors and is **not an integrity bound**.

The local cubic smoother is an experimental replacement for the old orbit fit,
not precision orbit determination. Current engineering gates include two satellite
IDs, 12 usable rows, 60 seconds of orbital history, 30-second freshness, 50 m/s
residual RMS, and 10 km conditional horizontal sigma. Rows from a shared orbit fit
are correlated; a large row count does not create independent information.

## What remains before a reliable field result

1. Replay several stationary captures with independent surveyed truth, fresh
   sessions, differing geometry, and no truth-based initialization. Preserve all
   rejected epochs and report availability, first-fix time, error distributions,
   residuals by satellite, and confidently wrong fixes.
2. Validate absolute frequency and burst timing against raw IQ. Fix/report dropped
   samples and timestamp discontinuities at the capture boundary. The existing
   detector timing and PLL frequency estimator are not repaired by this branch.
3. Use a stable receiver reference and record reference/clock state, tuning,
   sample rate, antenna, gain, capture time origin, and sample-loss counters.
   Determine per-observation uncertainties empirically; 50 Hz is provisional.
4. Model correlated orbit errors, evaluate robust orbit fitting and external
   ephemerides, and test transmitter bias and receiver drift. Compare independent
   passes rather than trusting the internal covariance.
5. Replace normal-equation inversion with rank-revealing QR/SVD if conditioning
   tests warrant it; test poor/mirrored geometry and initial-condition sensitivity
   more broadly. Current gates are conservative heuristics, not validated integrity.

A practical next capture is 15–30 minutes at a fixed, surveyed, open-sky antenna
with a stable L-band receiver/reference and raw IQ retained. Hardware selection
and achieved accuracy still depend on measured frequency/timing performance.
