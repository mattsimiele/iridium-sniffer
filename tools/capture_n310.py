#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Strict, single-channel N310 GPSDO capture to SigMF; abort on discontinuity."""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shutil
import signal
import sys
import time


class CaptureError(RuntimeError):
    pass


def ticks(ts, rate):
    """Never convert a Unix-sized hardware time to a floating-point second."""
    return int(ts.get_full_secs()) * rate + round(ts.get_frac_secs() * rate)


class Continuity:
    def __init__(self, first_tick):
        self.expected = first_tick

    def check(self, actual, count):
        delta = actual - self.expected
        if delta:
            kind = 'missing' if delta > 0 else 'overlap/backward'
            raise CaptureError(f'timestamp discontinuity: {kind} {abs(delta)} samples')
        if count <= 0:
            raise CaptureError('empty receive buffer')
        self.expected += count


def lock_state(radio):
    return {name: bool(radio.get_mboard_sensor(name, 0).to_bool())
            for name in ('gps_locked', 'ref_locked')}


def require_lock(radio):
    state = lock_state(radio)
    if not all(state.values()):
        raise CaptureError(f'reference lock lost: {state}')
    return state


def wait_lock(radio, timeout, stable, emit, clock=time.monotonic, sleep=time.sleep):
    deadline = clock() + timeout
    since = None
    while clock() < deadline:
        state = lock_state(radio)
        emit('lock', **state)
        now = clock()
        if all(state.values()):
            if since is None:
                since = now
            if now - since >= stable:
                return
        else:
            since = None
        sleep(1)
    raise CaptureError('GPSDO/ref lock did not remain stable before timeout')


def sync_epoch(radio, uhd, emit):
    # Follow UHD's sync_to_gps example; validate both the applied PPS epoch and
    # the time scale. gps_time must represent UTC Unix seconds on this device.
    gps = radio.get_mboard_sensor('gps_time', 0).to_int()
    if abs(gps - time.time()) > 5:
        raise CaptureError('gps_time is not within 5 s of host UTC; check host NTP, GPS and time scale')
    radio.set_time_next_pps(uhd.types.TimeSpec(int(gps) + 1, 0.0), 0)
    time.sleep(2.2)
    for _ in range(10):
        before = radio.get_time_last_pps(0)
        gps = radio.get_mboard_sensor('gps_time', 0).to_int()
        after = radio.get_time_last_pps(0)
        if ticks(before, 1_000_000_000) == ticks(after, 1_000_000_000) == int(gps)*1_000_000_000:
            require_lock(radio)
            emit('epoch_synchronized', unix_seconds=int(gps))
            return
        time.sleep(0.2)
    raise CaptureError('PPS/device epoch does not agree with gps_time')


def uhd_version(uhd):
    # Older distro bindings do not re-export get_version_string at top level.
    for namespace in (uhd, getattr(uhd, 'libpyuhd', None)):
        getter = getattr(namespace, 'get_version_string', None)
        if callable(getter):
            return str(getter())
    return str(getattr(uhd, '__version__', 'unavailable; save uhd_config_info --version separately'))


def metadata(actual, requested, hardware):
    return {'global': {
        'core:datatype': 'ci16_le', 'core:sample_rate': actual['rate'],
        'core:version': '1.2.0', 'core:num_channels': 1,
        'core:recorder': 'iridium-sniffer/tools/capture_n310.py',
        'core:hw': hardware,
        'core:description': 'Stationary Iridium IRA; GPSDO-disciplined; not GNSS-independent',
        'n310_capture:requested': requested, 'n310_capture:actual': actual,
        'n310_capture:status': 'incomplete', 'n310_capture:valid': False,
        'n310_capture:samples_written': 0,
        'n310_capture:timestamp_basis': 'UHD gps_time checked against host UTC Unix seconds',
        'n310_capture:wire_format': 'sc16', 'n310_capture:cpu_format': 'fc32',
        'n310_capture:conversion': 'round(fc32 * 32768), saturate to signed little-endian int16',
    }, 'captures': [], 'annotations': []}


def save_meta(path, meta):
    temporary = path.with_suffix('.tmp')
    with temporary.open('w') as f:
        json.dump(meta, f, indent=2, allow_nan=False)
        f.write('\n')
        f.flush()
        os.fsync(f.fileno())
    os.replace(temporary, path)


def receive(radio, streamer, uhd, np, data, emit, meta, duration, start_seconds,
            clock=time.monotonic):
    rate = int(meta['global']['core:sample_rate'])
    total = round(duration * rate)
    first_tick = start_seconds * rate
    continuity = Continuity(first_tick)
    buf = np.empty((1, 65536), dtype=np.complex64)
    md = uhd.types.RXMetadata()
    command = uhd.types.StreamCMD(uhd.types.StreamMode.start_cont)
    command.stream_now = False
    command.time_spec = uhd.types.TimeSpec(start_seconds, 0.0)
    command_sent = False
    written = clipped = 0
    next_check = clock()
    try:
        streamer.issue_stream_cmd(command)
        command_sent = True
        while written < total:
            if clock() >= next_check:
                emit('lock', **require_lock(radio))
                next_check = clock() + 1
            count = streamer.recv(buf, md, 5.0)
            stamp = ticks(md.time_spec, rate) if md.has_time_spec else None
            emit('receive', sample_start=written, count=int(count),
                 hardware_tick=stamp, expected_tick=continuity.expected,
                 delta_samples=None if stamp is None else stamp-continuity.expected,
                 error=str(md.error_code), out_of_sequence=bool(md.out_of_sequence))
            # Overflow may report before its missing-sample extent is known.
            # Abort immediately; never invent a precise drop count or append
            # good-looking data after the error.
            if md.error_code != uhd.types.RXMetadataErrorCode.none or md.out_of_sequence:
                raise CaptureError('UHD receive error: ' + md.strerror())
            if stamp is None:
                raise CaptureError('receive metadata has no hardware timestamp')
            continuity.check(stamp, count)
            count = min(count, total-written)
            floats = buf[0, :count].view(np.float32)
            if not np.isfinite(floats).all():
                raise CaptureError('non-finite IQ samples')
            converted = np.rint(floats * 32768.0)
            clipped += int(np.count_nonzero((converted < -32768) | (converted > 32767)))
            packed = np.clip(converted, -32768, 32767).astype('<i2').tobytes()
            if data.write(packed) != len(packed):
                raise CaptureError('short IQ file write')
            if not written:
                utc = datetime.fromtimestamp(start_seconds, timezone.utc).isoformat().replace('+00:00','Z')
                meta['captures'] = [{'core:sample_start': 0,
                    'core:frequency': meta['global']['n310_capture:actual']['frequency'],
                    'core:datetime': utc, 'n310_capture:hardware_tick': stamp}]
            written += count
            meta['global']['n310_capture:samples_written'] = written
            meta['global']['n310_capture:conversion_clipped_components'] = clipped
        emit('lock', **require_lock(radio))
        if clipped:
            raise CaptureError(f'{clipped} IQ components saturated during int16 conversion; reduce gain')
    finally:
        if command_sent:
            stop = uhd.types.StreamCMD(uhd.types.StreamMode.stop_cont)
            streamer.issue_stream_cmd(stop)
    return written


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--args', required=True, help='UHD address, e.g. addr=192.168.10.2')
    p.add_argument('--output', type=Path, required=True, help='NEW capture directory; never overwritten')
    p.add_argument('--duration', type=float, default=1800)
    p.add_argument('--rate', type=float, choices=(2_500_000, 12_500_000), default=2_500_000)
    p.add_argument('--freq', type=float, default=1_626_000_000)
    p.add_argument('--gain', type=float, default=30, help='manual dB; adjust using a short pilot recording')
    p.add_argument('--bandwidth', type=float, default=5_000_000)
    p.add_argument('--channel', type=int, choices=range(4), default=0)
    p.add_argument('--antenna', default='RX2')
    p.add_argument('--lock-timeout', type=float, default=300)
    p.add_argument('--stable-lock', type=float, default=30)
    return p


def capture(a, uhd, np):
    # Fix clock policy, regardless of any contradicting generic device args.
    kv = dict(part.split('=', 1) for part in a.args.split(',') if part)
    kv.update(master_clock_rate='125e6', clock_source='gpsdo', time_source='gpsdo')
    device_args = ','.join(f'{k}={v}' for k,v in kv.items())
    radio = uhd.usrp.MultiUSRP(device_args)
    if radio.get_num_mboards() != 1 or 'N310' not in radio.get_mboard_name(0).upper():
        raise CaptureError('this script requires exactly one N310')
    names = set(radio.get_mboard_sensor_names(0))
    if not {'gps_locked','ref_locked','gps_time'} <= names:
        raise CaptureError('required GPSDO sensors unavailable')
    radio.set_clock_source('gpsdo', 0)
    radio.set_time_source('gpsdo', 0)
    ch = a.channel
    radio.set_rx_rate(a.rate, ch)
    tuning = radio.set_rx_freq(uhd.types.TuneRequest(a.freq), ch)
    # N310 uses manual gain; UHD set_rx_agc is unsupported on this radio.
    radio.set_rx_gain(a.gain, ch)
    radio.set_rx_bandwidth(a.bandwidth, ch)
    radio.set_rx_antenna(a.antenna, ch)
    actual = {'rate': radio.get_rx_rate(ch), 'frequency': radio.get_rx_freq(ch),
              'rf_frequency': tuning.actual_rf_freq, 'dsp_frequency': tuning.actual_dsp_freq,
              'gain': radio.get_rx_gain(ch), 'bandwidth': radio.get_rx_bandwidth(ch),
              'antenna': radio.get_rx_antenna(ch), 'channel': ch,
              'master_clock_rate': radio.get_master_clock_rate(0),
              'clock_source': radio.get_clock_source(0), 'time_source': radio.get_time_source(0)}
    if abs(actual['rate']-a.rate) > 0.01 or abs(actual['master_clock_rate']-125e6) > 1:
        raise CaptureError(f'unexpected clock/rate readback: {actual}')
    if actual['clock_source'] != 'gpsdo' or actual['time_source'] != 'gpsdo':
        raise CaptureError('GPSDO source readback mismatch')
    if actual['bandwidth'] < a.rate or abs(actual['frequency']-a.freq) > 1:
        raise CaptureError(f'unexpected RF frequency/bandwidth readback: {actual}')
    target = 1_626_270_833.3333333
    if abs(target-actual['frequency']) + 100_000 > a.rate/2:
        raise CaptureError('IRA target lacks 100 kHz capture-edge margin')
    a.output.mkdir()  # Exclusive creation reserves all three output paths.
    meta_path = a.output/'capture.sigmf-meta'
    requested = {k:str(v) if isinstance(v,Path) else v for k,v in vars(a).items()}
    meta = metadata(actual, requested, radio.get_pp_string())
    meta['global']['n310_capture:uhd_version'] = uhd_version(uhd)
    save_meta(meta_path, meta)
    with (a.output/'capture.events.jsonl').open('x') as events:
        def emit(event, **fields):
            events.write(json.dumps({'event':event, 'host_unix_ns':time.time_ns(), **fields},allow_nan=False)+'\n')
            if event != 'receive':
                events.flush()
                print(event, fields, file=sys.stderr, flush=True)
        try:
            required = math.ceil(a.duration*a.rate)*4
            if shutil.disk_usage(a.output).free < required*1.05 + 100_000_000:
                raise CaptureError(f'insufficient free disk space for {required} IQ bytes plus margin')
            emit('configuration', **actual)
            wait_lock(radio, a.lock_timeout, a.stable_lock, emit)
            sync_epoch(radio, uhd, emit)
            args = uhd.usrp.StreamArgs('fc32','sc16')
            args.channels = [ch]
            streamer = radio.get_rx_stream(args)
            start = radio.get_time_now(0).get_full_secs()+3
            meta['global']['n310_capture:scheduled_start_unix_seconds'] = start
            save_meta(meta_path, meta)
            with (a.output/'capture.sigmf-data').open('xb') as data:
                receive(radio, streamer, uhd, np, data, emit, meta, a.duration, start)
                data.flush()
                os.fsync(data.fileno())
            meta['global'].update({'n310_capture:status':'complete','n310_capture:valid':True})
            emit('complete', samples=meta['global']['n310_capture:samples_written'])
        except (Exception, KeyboardInterrupt) as exc:
            meta['global'].update({'n310_capture:status':'failed','n310_capture:valid':False,
                                   'n310_capture:failure':str(exc) or type(exc).__name__})
            emit('failed', reason=meta['global']['n310_capture:failure'])
            raise
        finally:
            save_meta(meta_path, meta)


def main():
    p = parser()
    a = p.parse_args()
    values = (a.duration,a.freq,a.gain,a.bandwidth,a.lock_timeout,a.stable_lock)
    if not all(map(math.isfinite,values)) or a.duration <= 0 or a.duration*a.rate < 1:
        p.error('settings must be finite and duration must include at least one sample')
    if a.bandwidth < a.rate or a.stable_lock < 1 or a.lock_timeout <= a.stable_lock:
        p.error('bandwidth must cover sample rate; lock timeout must exceed stable-lock >= 1 s')
    if a.output.exists():
        p.error('output directory already exists; choose a new run name')
    def terminated(signum, frame):
        raise CaptureError(f'interrupted by signal {signum}')
    signal.signal(signal.SIGTERM, terminated)
    try:
        import numpy as np
        import uhd
        capture(a, uhd, np)
    except (Exception, KeyboardInterrupt) as exc:
        print(f'Capture FAILED: {exc or type(exc).__name__}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
