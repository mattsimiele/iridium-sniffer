#!/usr/bin/env python3
"""Score replay JSONL against surveyed truth; truth is never sent to the solver."""
import argparse
import json
import math
import statistics

def ecef(lat, lon, alt):
    lat, lon = map(math.radians, (lat, lon))
    e2 = (1/298.257223563)*(2-1/298.257223563)
    n = 6378137/math.sqrt(1-e2*math.sin(lat)**2)
    return ((n+alt)*math.cos(lat)*math.cos(lon), (n+alt)*math.cos(lat)*math.sin(lon), (n*(1-e2)+alt)*math.sin(lat))

def score(rows, truth):
    origin = ecef(*truth)
    lat, lon = map(math.radians, truth[:2])
    east = (-math.sin(lon), math.cos(lon), 0)
    north = (-math.sin(lat)*math.cos(lon), -math.sin(lat)*math.sin(lon), math.cos(lat))
    errors = []
    first = None
    for row in rows:
        if row['status'] != 'valid':
            continue
        if first is None:
            first = (row['time_ns']-rows[0]['time_ns'])/1e9
        delta = [a-b for a,b in zip(ecef(row['lat'],row['lon'],row['alt_m']),origin)]
        errors.append(math.hypot(sum(a*b for a,b in zip(east,delta)),sum(a*b for a,b in zip(north,delta))))
    return {'epochs':len(rows), 'valid_epochs':len(errors), 'first_valid_from_first_output_s':first,
            'horizontal_median_m':statistics.median(errors) if errors else None,
            'horizontal_max_m':max(errors) if errors else None}

if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('jsonl')
    p.add_argument('--truth', nargs=3, type=float, required=True, metavar=('LAT','LON','ELLIPSOID_M'))
    p.add_argument('--max-error', type=float, default=5000, help='fail if any valid epoch exceeds this horizontal error')
    a = p.parse_args()
    if not all(map(math.isfinite,a.truth)) or not -90<=a.truth[0]<=90 or not -180<=a.truth[1]<=180 or not math.isfinite(a.max_error) or a.max_error<=0:
        p.error('invalid truth or error limit')
    with open(a.jsonl) as f:
        result = score([json.loads(line) for line in f], a.truth)
    print(json.dumps(result,allow_nan=False,indent=2))
    raise SystemExit(0 if result['valid_epochs'] and result['horizontal_max_m']<=a.max_error else 1)
