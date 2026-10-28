# Stationary N310 capture

`tools/capture_n310.py` is a standalone Python/UHD recorder. It does not use the
sniffer's live USRP backend. Default settings are one channel, 125 MHz master
clock, 2.5 MS/s, 1626 MHz center, requested 5 MHz analog bandwidth, manual 30 dB
gain, RX2 antenna input, 30 minutes, GPSDO clock and time sources. Inspect actual
readbacks and determine gain using a pilot recording. Antenna must cover the
Iridium IRA carrier at 1626.270833 MHz with RHCP and clear sky view.

## Workstation prerequisites

Use your working UHD installation, with its matching **UHD Python bindings** and
NumPy. Do not replace a functioning N310 host/FPGA installation just for this
script. On an Ubuntu workstation using distribution UHD packages, the Python
packages are `python3-uhd` and `python3-numpy`; use `/usr/bin/python3` to access
them. Avoid unrelated PyPI packages named `uhd`.

```sh
/usr/bin/python3 -c 'import uhd, numpy; print(uhd.get_version_string())'
```

Connect the Iridium antenna to the selected RF channel's RX2 input. Connect an
appropriate GPS antenna to the N310 GPS input. Allow thermal warm-up and GPSDO
settling; the script's 30-second stable-lock check is not a frequency-stability
measurement. Host UTC must be synchronized (e.g. NTP): it is a 5-second sanity
check against wrong GPS epoch/time scale, **not** the source of sample timestamps.

## Pilot, then full recording

Run from the fork on `codex/stationary-doppler-replay`. Replace the IP with the
N310's streaming-interface address and use a new directory for every run:

```sh
/usr/bin/python3 tools/capture_n310.py \
  --args addr=192.168.10.2 \
  --output pilot-01 --duration 60 --gain 30

/usr/bin/python3 tools/capture_n310.py \
  --args addr=192.168.10.2 \
  --output stationary-01 --duration 1800 --gain 30
```

The output parent directory must exist. Existing run directories are refused.
A 30-minute run needs 18 GB for IQ plus logs and free-space margin. Inspect pilot
IQ for clipping, burst headroom and decoding before committing to a longer run.
The script detects conversion saturation, not every possible RF/ADC compression
condition. Lock sensors are checked approximately once per second during capture;
very short lock excursions between checks are not guaranteed to be detected.

For a wider recording (90 GB IQ per 30 minutes):

```sh
/usr/bin/python3 tools/capture_n310.py \
  --args addr=192.168.10.2 \
  --output wideband-01 --duration 1800 \
  --rate 12500000 --freq 1622000000 --bandwidth 20000000 --gain 30
```

Use a suitable data link and SSD. The synchronous Python receive/write path has
not been throughput-qualified on your workstation, particularly at 12.5 MS/s.
It aborts if it cannot keep up. No hardware capture has been validated by CI.

## What is recorded

- `capture.sigmf-data`: little-endian signed 16-bit interleaved I,Q (`ci16_le`).
  UHD wire format is sc16, CPU buffers are fc32, converted using round(x*32768)
  and saturation. Conversion saturation makes the run fail. This conversion is
  explicit and is not a claim of bit-identical sc16 wire preservation.
- `capture.sigmf-meta`: sample rate, frequency, UTC start, hardware description,
  UHD version, requested/actual parameters, hardware start tick, written sample
  count and completion state. Project-specific fields use `n310_capture:`.
- `capture.events.jsonl`: every receive buffer's hardware timestamp in integer
  sample ticks, expected tick, count, signed sample discrepancy, UHD error and
  sequence flag, plus lock states and lifecycle events. Timestamps are computed
  from separate integer/fractional UHD time parts to preserve sample resolution.

Clock and time sources are forced to GPSDO. Required GPS/ref sensors must exist
and remain locked for the configured interval (default 30 s, within a 300 s
wait). The script sets time on the next PPS from `gps_time`, verifies the PPS
epoch, then schedules reception at a future integer device second. First-buffer
and every subsequent timestamp must match the expected sample tick exactly.

Any overflow, timeout, sequence error, missing timestamp, gap/overlap, non-finite
IQ, lock loss, conversion saturation or disk error exits nonzero. No post-gap
samples are appended and no zero-fill conceals a gap. A positive timestamp delta
quantifies missing samples; an overflow without a subsequent valid timestamp has
an **unknown** missing-sample count. Read both the error and delta fields.

Partial files remain for diagnosis, with `n310_capture:valid=false` and failure
reason. Ctrl-C/SIGTERM also fail the run and attempt to stop the stream and save
metadata. Power loss/SIGKILL can leave `status=incomplete`; do not use such a run
as a validated continuous capture. Metadata is saved atomically at setup/start
and finalization; it is not continuously checkpointed. Disk failure can prevent
final metadata updates, in which case the initial incomplete state remains.

## Check before replay

Require exit code zero, `status=complete`, `valid=true`, and data byte length equal
to `4 * samples_written`. The decoder currently does not enforce these custom
metadata fields itself. For example:

```sh
python3 - stationary-01 <<'PY'
import json, pathlib, sys
p = pathlib.Path(sys.argv[1])
g = json.loads((p/'capture.sigmf-meta').read_text())['global']
assert g['n310_capture:valid'] and g['n310_capture:status'] == 'complete'
assert (p/'capture.sigmf-data').stat().st_size == 4*g['n310_capture:samples_written']
print('Capture complete:', g['n310_capture:samples_written'], 'samples')
PY

./build/iridium-sniffer -f stationary-01/capture.sigmf-data \
  --position=100 --position-log=stationary-01/observations.csv
python3 tools/replay.py stationary-01/observations.csv \
  --height 100 > stationary-01/fixes.jsonl
python3 tools/score_replay.py stationary-01/fixes.jsonl \
  --truth LAT LON 100 --max-error 5000
```

Replace 100 with surveyed **ellipsoidal antenna height** and LAT/LON with surveyed
truth only in the scorer. Keep survey records separately; the capture script
neither needs nor reads truth coordinates. The file decoder currently derives
relative burst timing from sample count and does not restore the SigMF UTC epoch;
the original hardware/UTC time mapping remains in these capture artifacts for
later analysis. Do not mistake a GPSDO-disciplined experiment for a demonstration
of GNSS-independent operation.

## Tests and sources

```sh
python3 tests/test_capture_n310.py  # requires NumPy, no UHD/radio
```

Fake-radio tests cover continuity, gaps/overlap, bad first timestamp, overflow,
timeout, missing timestamps, sequence errors, IQ layout, clipping, final-buffer
trimming, lock loss/stability, epoch checks, metadata finalization and overwrite
protection. Linux CI also imports actual UHD Python bindings and checks the API
objects used to construct the stream; this cannot establish N310 RF performance.

- [UHD N3xx manual](https://files.ettus.com/manual/page_usrp_n3xx.html)
- [UHD synchronization](https://files.ettus.com/manual/page_sync.html)
- [UHD sync_to_gps example](https://github.com/EttusResearch/uhd/blob/master/host/examples/sync_to_gps.cpp)
- [Stationary estimator and replay assumptions](STATIONARY_POSITIONING.md)
