"""Summarize saved bench logs. Does not equate logged-frame counts to accuracy.

Usage: python tools/summarize_doa_bench.py tools/real_final_*gain2.json
"""
import argparse
import glob
import json
import math
from pathlib import Path
import re
import statistics


def summarize(patterns):
    records = []
    for pattern in patterns:
        for file in glob.glob(pattern):
            path = Path(file)
            rows = json.loads(path.read_text(encoding='utf-8'))
            for row in rows:
                angles = row.get('angles', [])
                if angles:
                    row['mean_angle'] = math.degrees(math.atan2(
                        sum(math.sin(math.radians(x)) for x in angles),
                        sum(math.cos(math.radians(x)) for x in angles))) % 360
                    row['angle_range'] = [min(angles), max(angles)]
                row['log'] = str(path.with_suffix('.log'))
                records.append(row)
            log = path.with_suffix('.log')
            if log.exists():
                text = log.read_text(encoding='utf-8')
                ovf = list(map(int, re.findall(r'ovf=(\d+)', text)))
                times = list(map(int, re.findall(r'us=(\d+)', text)))
                print(path.name, 'overflow=', [min(ovf), max(ovf)] if ovf else None,
                      'median_us=', statistics.median(times) if times else None)
    print('Clips with at least one valid logged direction:',
          sum(bool(row['valid']) for row in records), '/', len(records))
    for row in records:
        angle = row.get('mean_angle')
        print(row['name'], 'valid_rows=', row['valid'],
              'mean=', round(angle, 1) if angle is not None else None,
              'range=', row.get('angle_range'), 'pixels=', sorted(set(row.get('pixels', []))))
    return records


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('patterns', nargs='+')
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    rows = summarize(args.patterns)
    if args.out:
        args.out.write_text(json.dumps(rows, ensure_ascii=False, indent=2), encoding='utf-8')
