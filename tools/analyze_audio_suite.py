"""Summarize physical real-audio tests using frame-time playback windows.

The reference angle is a recorded music direction, not surveyed ground truth.
Responses count clips with any accepted update; time coverage and per-frame
outliers are reported separately so a response count is not called accuracy.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import re
import statistics


def analyze(path, reference=300.):
    clips=[]
    for line in path.read_text(encoding='utf8').splitlines():
        if line.startswith('CLIP '):
            item=json.loads(line[5:]);item['rows']=[];clips.append(item)
            continue
        tm=re.match(r'\[(-?[\d.]+)\]',line)
        if not tm or 'DOA=' not in line:continue
        t=float(tm[1])
        values=dict(re.findall(r'([a-zA-Z_]+)=(-?\d+(?:\.\d+)?)',line))
        rms=re.search(r'rms=\[([^]]+)\]',line)
        row=dict(time=t,angle=float(values['DOA']),raw=float(values['raw']),
                 active=int(values.get('act',0)),display=int(values.get('dsp',0)),
                 valid='status=valid' in line,pixel=int(values.get('led',-1)),
                 band_rms=float(values.get('br',0)),floor=float(values.get('fl',0)),
                 confidence=float(values.get('conf',0)),score=float(values.get('srp',0)),
                 mirror=float(values.get('mir',0)),overflow=int(values.get('ovf',0)),
                 rms=max(map(float,rms[1].split())) if rms else 0.,
                 processing_us=int(values.get('us',0)))
        for clip in reversed(clips):
            if clip['start']-.03<=t<=clip['start']+clip['duration']+.3:
                clip['rows'].append(row);break
    results=[]
    for clip in clips:
        rows=sorted(clip['rows'],key=lambda r:r['time'])
        valid=[r for r in rows if r['valid']]
        angles=[r['angle'] for r in valid]
        mean=None
        spread=None
        if angles:
            mean=math.degrees(math.atan2(sum(math.sin(math.radians(a)) for a in angles),
                                       sum(math.cos(math.radians(a)) for a in angles)))%360
            spread=math.sqrt(sum(((a-mean+180)%360-180)**2 for a in angles)/len(angles))
        weights=[max(0,min(.5,b['time']-a['time'])) for a,b in zip(rows,rows[1:])]
        weights.append(weights[-1] if weights else .02)
        total=sum(weights) or 1.
        rejected=[r for r in rows if r['active'] and not r['valid']]
        reasons=Counter()
        for r in rejected:
            if r['band_rms']<max(.0004,r['floor']*1.6):reasons['low_band_energy']+=1
            if r['score']<.006:reasons['low_spatial_score']+=1
            if r['mirror']>=.9 or (r['mirror']>=.6 and r['confidence']<.16):reasons['ambiguous_opposite']+=1
            if r['confidence']<.025:reasons['low_confidence']+=1
        results.append(dict(name=clip['name'],gain=clip.get('gain'),duration=clip['duration'],
                            rows=len(rows),valid_rows=len(valid),responded=bool(valid),
                            first_update_seconds=valid[0]['time']-clip['start'] if valid else None,
                            mean_angle=mean,rms_spread=spread,
                            angle_range=[min(angles),max(angles)] if angles else None,
                            reference_outlier_rows=sum(abs((a-reference+180)%360-180)>120 for a in angles),
                            active_time_fraction=sum(w*r['active'] for w,r in zip(weights,rows))/total,
                            display_time_fraction=sum(w*r['display'] for w,r in zip(weights,rows))/total,
                            max_raw_rms=max((r['rms'] for r in rows),default=0),
                            max_filtered_snr=max((r['band_rms']/max(.0001,r['floor']) for r in rows),default=0),
                            overflow_delta=rows[-1]['overflow']-rows[0]['overflow'] if rows else None,
                            rejection_observations=dict(reasons)))
    return results


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('logs',nargs='+',type=Path)
    ap.add_argument('--reference',type=float,default=300.)
    ap.add_argument('--out',type=Path)
    args=ap.parse_args()
    report={str(path):analyze(path,args.reference) for path in args.logs}
    for file,rows in report.items():
        print(file,'responses',sum(r['responded'] for r in rows),'/',len(rows))
        for r in rows:
            print(r['name'], 'valid',r['valid_rows'],'mean',round(r['mean_angle'],1) if r['mean_angle'] is not None else None,
                  'spread',round(r['rms_spread'],1) if r['rms_spread'] is not None else None,
                  'SNRmax',round(r['max_filtered_snr'],1),'outliers',r['reference_outlier_rows'])
    if args.out:args.out.write_text(json.dumps(report,indent=2),encoding='utf8')


if __name__=='__main__':main()
