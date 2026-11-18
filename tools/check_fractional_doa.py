"""Compare actual C estimator interpolation on fractional-delay plane waves.

This is an offline algorithm regression, not a substitute for real speaker
tests. Temporary variants are built in .fetch; firmware is not edited here.
"""
import ctypes
import json
import math
import os
from pathlib import Path
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tests'))
from msvc import VCVARS, CL


def build_variant(folder, cubic):
    folder.mkdir(parents=True, exist_ok=True)
    source=(ROOT/'main/doa/gcc_phat.c').read_text(encoding='utf8')
    if not cubic and '    /* The correlation is band-limited;' in source:
        start=source.index('    /* The correlation is band-limited;')
        end=source.index('\n}',start)
        source=source[:start]+'    return corr[i0] + fraction * (corr[i1] - corr[i0]);'+source[end:]
    if cubic:
        source=source.replace('return corr[i0] + fraction * (corr[i1] - corr[i0]);', '''
    float ym = corr[(i0 + GCC_FFT_SIZE - 1) % GCC_FFT_SIZE];
    float y0 = corr[i0], y1 = corr[i1];
    float yp = corr[(i1 + 1) % GCC_FFT_SIZE];
    return y0 + .5f * fraction * (y1 - ym + fraction *
        (2.f * ym - 5.f * y0 + 4.f * y1 - yp + fraction *
         (3.f * (y0-y1) + yp-ym)));''')
    (folder/'estimator.c').write_text(source,encoding='utf8')
    (folder/'wrapper.c').write_text('''
#include "gcc_phat.h"
static int16_t ch[4][GCC_FFT_SIZE];
__declspec(dllexport) void reset_estimator(void) { gcc_phat_reset_history(); }
__declspec(dllexport) void estimate(int16_t *samples, float *out) {
    for (int i=0;i<GCC_FFT_SIZE;++i) for (int c=0;c<4;++c) ch[c][i]=samples[i*4+c];
    const int16_t *m[4]={ch[0],ch[1],ch[2],ch[3]};
    gcc_phat_result_t r;
    gcc_phat_estimate_4ch(m,GCC_FFT_SIZE,48000,&r);
    out[0]=r.angle_deg;out[1]=r.score;out[2]=r.confidence;out[3]=r.valid;
}
''',encoding='utf8')
    env=os.environ.copy()
    output=subprocess.check_output(f'cmd /c ""{VCVARS}" >nul && set"',text=True)
    env.update(line.split('=',1) for line in output.splitlines() if '=' in line)
    dll=folder/'estimator.dll'
    p=subprocess.run([str(CL),'/nologo','/utf-8','/O2','/LD','/std:c11',
                      '/I'+str(ROOT/'tests/stubs'),'/I'+str(ROOT/'main/doa'),
                      str(folder/'estimator.c'),str(folder/'wrapper.c'),'/Fe:'+str(dll)],
                     cwd=folder,env=env,capture_output=True,text=True,encoding='utf8',errors='replace')
    if p.returncode:
        raise RuntimeError(p.stdout+p.stderr)
    lib=ctypes.CDLL(str(dll))
    lib.estimate.argtypes=[ctypes.POINTER(ctypes.c_int16),ctypes.POINTER(ctypes.c_float)]
    return lib


def main():
    cache=ROOT/'.fetch/fractional-doa'
    variants={name:build_variant(cache/name,name=='cubic') for name in ['linear','cubic']}
    rng=np.random.default_rng(8241)
    freq=rng.uniform(400,5900,48)
    phase=rng.uniform(-np.pi,np.pi,len(freq))
    pos=np.array([[0,-.04],[-.04,0],[.04,0],[0,.04]])
    records={name:[] for name in variants}
    for angle in np.arange(0,360,5):
        u=np.array([math.cos(math.radians(angle)),math.sin(math.radians(angle))])
        delay=-(pos@u)/343
        t=np.arange(1024)/48000
        x=np.sin(2*np.pi*(t[:,None,None]-delay[None,:,None])*freq[None,None,:]+phase).sum(axis=2)
        samples=np.ascontiguousarray(np.round(x*1600).astype(np.int16))
        for name,lib in variants.items():
            lib.reset_estimator()
            out=(ctypes.c_float*4)()
            lib.estimate(samples.ctypes.data_as(ctypes.POINTER(ctypes.c_int16)),out)
            error=abs((out[0]-angle+180)%360-180)
            records[name].append(dict(angle=int(angle),estimated=out[0],error=error,valid=bool(out[3])))
    for name,rows in records.items():
        errors=np.array([r['error'] for r in rows])
        print(name,'mean_error',float(errors.mean()),'max_error',float(errors.max()),
              'valid',sum(r['valid'] for r in rows),'/',len(rows))
    (cache/'comparison.json').write_text(json.dumps(records,indent=2),encoding='utf8')


if __name__=='__main__':
    main()
