#!/usr/bin/env python3
"""Validate/sort observation CSV, replay through the production C solver."""
import argparse
import csv
import math
import pathlib
import subprocess
import sys

FIELDS = 'timestamp_ns,sat_id,x4km,y4km,z4km,frequency_hz,sigma_hz'.split(',')

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('csv')
    p.add_argument('--height', type=float, required=True, help='WGS84 ellipsoidal height, metres')
    p.add_argument('--interval', type=int, default=10, help='solve cadence, seconds')
    args = p.parse_args()
    if not math.isfinite(args.height) or not -500 <= args.height <= 10000 or args.interval < 1:
        p.error('invalid height or interval')
    rows = []
    try:
        with open(args.csv, newline='') as f:
            reader = csv.DictReader(f)
            if reader.fieldnames != FIELDS:
                raise ValueError('header must be ' + ','.join(FIELDS))
            for row in reader:
                if None in row or any(row[k] is None for k in FIELDS):
                    raise ValueError('wrong column count')
                t, sat, x, y, z = (int(row[k]) for k in FIELDS[:5])
                freq, sigma = (float(row[k]) for k in FIELDS[5:])
                if not 0 < t < 2**64 - 32_000_000_000 or not 0 <= sat < 128 or any(not -2048 <= q <= 2047 for q in (x,y,z)):
                    raise ValueError('integer out of range')
                if not math.isfinite(freq) or not math.isfinite(sigma) or sigma <= 0:
                    raise ValueError('invalid frequency or uncertainty')
                rows.append((t,sat,x,y,z,freq,sigma))
        if not rows:
            raise ValueError('empty observation file')
    except (ValueError, OSError) as e:
        p.error(str(e))
    rows.sort(key=lambda r:r[0])  # stable event-time order; integer nanoseconds
    commands = [f'H {args.height}\n']
    tick = rows[0][0] + args.interval * 10**9
    for row in rows:
        while tick < row[0]:
            commands.append(f'S {tick}\n')
            tick += args.interval * 10**9
        commands.append('O ' + ' '.join(str(v) for v in row) + '\n')
    commands.extend([f'S {rows[-1][0]}\n', f'S {rows[-1][0]+31_000_000_000}\n'])
    exe = pathlib.Path(__file__).resolve().parents[1] / 'build-doppler/replay'
    try:
        result = subprocess.run([str(exe)], input=''.join(commands), text=True)
    except OSError as e:
        p.error(str(e) + '; run make -f Makefile.doppler test')
    return result.returncode

if __name__ == '__main__':
    sys.exit(main())
