"""Replay captured 4-channel PCM through actual C firmware DSP/gating functions.

Replays the currently saved C sources; capture PCM is not modified.
Generated host glue extracts the main loop and helpers from saved main.c.
"""
import argparse
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import wave

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tests'))
from msvc import VCVARS,CL


def function(source,name):
    match=re.search(r'static\s+[\w\s*]+\b'+name+r'\([^;{]*\)\s*\{',source)
    if not match:raise ValueError('Function not found: '+name)
    start=match.start();end=source.index('{',match.start());depth=1;end+=1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}');end+=1
    return source[start:end]+'\n'


def build(variant):
    main=(ROOT/'main/main.c').read_text(encoding='utf8')
    gcc=(ROOT/'main/doa/gcc_phat.c').read_text(encoding='utf8')
    tracker=(ROOT/'main/doa/doa_tracker.c').read_text(encoding='utf8')
    defines='\n'.join(line for line in main.splitlines() if line.startswith('#define '))
    helper='\n'.join(function(main,n) for n in ['bandpass_for_gcc_i16','calc_rms_norm_i16',
                         'calc_peak_block_rms_norm_i16','update_source_active','count_clipped',
                         'channel_onset_time','onset_doa_estimate'])
    a=main.index('        /* ---- GCC-PHAT 声源定位 (预滤波后) ---- */')
    b=main.index('#elif APP_MODE == APP_MODE_SOUND_CLASSIFY',a)
    loop=main[a:b]
    nf_a=main.index('static const float s_nf_mic_pos_m')
    nf_b=main.index('};',nf_a)+2
    suffix='''
__declspec(dllexport) void replay_gcc_reset(void) {
    s_noise_initialized=false;s_steering_rate=0;s_score_ema_alpha=.33f;
    s_instant_enabled=true;gcc_phat_init(1024);gcc_phat_reset_history();
}
float replay_center_score(void) {
    float sum=0,weights=0;
    for(int p=0;p<PAIR_COUNT;++p) if(s_pair_weight[p]>=PAIR_MIN_QUALITY) {
        sum+=s_pair_weight[p]*s_corr[p][0];weights+=s_pair_weight[p];
    }
    return weights>0?sum/weights:0;
}
'''
    wrapper='''
#include <math.h>
#include <string.h>
#include "gcc_phat.h"
#include "doa_tracker.h"
#include "doa_display_gate.h"
#include "doa_event.h"
#include "direction_mapping.h"
#include "doa_led_scene.h"
#define M_PI 3.14159265358979323846
'''+defines+'''
typedef struct {float x_prev,hp_prev,lp_prev;bool initialized;} gcc_filter_state_t;
static gcc_filter_state_t s_gcc_filter[4];
static int16_t s_ch[4][1024],s_ch_gcc[4][1024];
static float s_rms_floor=.0010f,s_rms_smooth,s_doa_angle;
static bool s_rms_valid,s_source_active;
static unsigned s_source_hold_frames,s_source_idle_frames,s_sustained_frames;
static float s_previous_frame_rms,s_previous_transient_rms;
static doa_tracker_t s_doa_tracker;
static doa_display_gate_t s_doa_display_gate;
static doa_led_scene_t s_scene_tracker;
static uint32_t s_replay_frame;
'''+main[nf_a:nf_b]+helper+'''
__declspec(dllexport) void replay_pipeline_reset(void) {
    extern void replay_gcc_reset(void);replay_gcc_reset();
    memset(s_gcc_filter,0,sizeof(s_gcc_filter));
    s_rms_floor=.001f;s_rms_smooth=s_doa_angle=0;
    s_rms_valid=s_source_active=false;
    s_source_hold_frames=s_source_idle_frames=s_sustained_frames=0;
    s_previous_frame_rms=s_previous_transient_rms=0;
    doa_tracker_init(&s_doa_tracker,DOA_TRACK_MIN_CONF);
    s_doa_tracker.acquisition_frames=5U;
    doa_display_gate_init(&s_doa_display_gate);
    doa_led_scene_init(&s_scene_tracker);s_replay_frame=0;
}
__declspec(dllexport) void replay_pipeline_frame(const int16_t *pcm,float *out) {
    extern float replay_center_score(void);
    const int frames=1024;
    for (int i=0;i<frames;++i) for (int ch=0;ch<4;++ch) s_ch[ch][i]=pcm[i*4+ch];
    float rms0=calc_rms_norm_i16(s_ch[0],frames),rms1=calc_rms_norm_i16(s_ch[1],frames);
    float rms2=calc_rms_norm_i16(s_ch[2],frames),rms3=calc_rms_norm_i16(s_ch[3],frames);
    float rms=fmaxf(fmaxf(rms0,rms1),fmaxf(rms2,rms3));
'''+loop+'''
    s_doa_angle=angle;
    float display_angle=angle;
    if (tracker_updated && transient_doa) doa_display_gate_init(&s_doa_display_gate);
    bool display=doa_display_gate_update(&s_doa_display_gate,tracker_updated,
        source_active,localization_valid,doa_energy,angle,&display_angle);
    led_effect_scene_t scene;
    doa_led_scene_update(&s_scene_tracker,&doa_result,
        source_active && !chain_clipped && brms>=fmaxf(.0004f,s_rms_floor*1.8f),
        display,direction_led_angle_of_index(s_doa_display_gate.held_pixel),
        brms,s_rms_floor,s_replay_frame++*1024U/48U,&scene);
    float values[]={angle,angle_raw,doa_result.score,doa_result.confidence,
        doa_result.mirror_ratio,brms,s_rms_floor,source_active,localization_valid,
        tracker_updated,display,s_doa_display_gate.held_pixel,
        direction_calibrate_doa(doa_result.instant_angle_deg),doa_result.instant_score,
        doa_result.instant_confidence,transient_rms,transient_peak_ratio,
        rms,transient_doa,analyze_instant,replay_center_score(),
        doa_result.display_peak_count,scene.count};
    memcpy(out,values,sizeof(values));
}
'''
    dependency_sources=''.join((ROOT/'main/doa'/s).read_text(encoding='utf8')
        for s in ['doa_event.c','doa_display_gate.c','direction_mapping.c',
                  'gcc_phat.h','doa_tracker.h','doa_event.h','doa_display_gate.h'])
    stamp=hashlib.sha256((gcc+suffix+wrapper+tracker+dependency_sources).encode()).hexdigest()[:12]
    folder=ROOT/'.fetch/raw-replay/host'/f'{variant}-{stamp}';folder.mkdir(parents=True,exist_ok=True)
    dll=folder/'pipeline.dll'
    if not dll.exists():
        (folder/'gcc.c').write_text(gcc+suffix,encoding='utf8')
        (folder/'wrapper.c').write_text(wrapper,encoding='utf8')
        (folder/'tracker.c').write_text(tracker,encoding='utf8')
        env=os.environ.copy();output=subprocess.check_output(f'cmd /c ""{VCVARS}" >nul && set"',text=True)
        env.update(line.split('=',1) for line in output.splitlines() if '=' in line)
        sources=[str(ROOT/'main/doa'/s) for s in ['doa_event.c','doa_display_gate.c','direction_mapping.c','doa_led_scene.c']]+[str(folder/'tracker.c'),str(ROOT/'main/led/led_effects.c')]
        result=subprocess.run([str(CL),'/nologo','/utf-8','/O2','/LD','/std:c11',
            '/I'+str(ROOT/'main/doa'),'/I'+str(ROOT/'main/led'),'/I'+str(ROOT/'tests/stubs'),
            str(folder/'gcc.c'),str(folder/'wrapper.c'),*sources,'/Fe:'+str(dll)],
            cwd=folder,env=env,capture_output=True,text=True,encoding='utf8',errors='replace')
        if result.returncode:raise RuntimeError(result.stdout+result.stderr)
    lib=ctypes.CDLL(str(dll));lib.replay_pipeline_frame.argtypes=[ctypes.POINTER(ctypes.c_int16),ctypes.POINTER(ctypes.c_float)]
    return lib


FIELDS=['angle','raw','score','confidence','mirror','band_rms','noise','active','quality',
        'updated','display','pixel','instant_angle','instant_score','instant_confidence',
        'transient_rms','peak_ratio','rms','transient','instant_analysis','center_score','display_peak_count','scene_count']


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('wav',type=Path)
    ap.add_argument('--variants',nargs='+',choices=['baseline'],default=['baseline'])
    ap.add_argument('--out',type=Path,required=True)
    args=ap.parse_args()
    metadata=json.loads(args.wav.with_suffix('.json').read_text())
    if any(metadata.get(k,0) for k in ['crc_errors','sequence_gaps','dma_overflows']):raise ValueError('Capture is not lossless')
    with wave.open(str(args.wav)) as w:
        assert w.getnchannels()==4 and w.getframerate()==48000
        pcm=np.frombuffer(w.readframes(w.getnframes()),'<i2').reshape(-1,4)
    result={}
    for variant in args.variants:
        lib=build(variant);lib.replay_pipeline_reset();output=[]
        data=pcm
        fs=48000
        for start in range(0,len(data)-1023,1024):
            block=np.ascontiguousarray(data[start:start+1024]);values=(ctypes.c_float*len(FIELDS))()
            lib.replay_pipeline_frame(block.ctypes.data_as(ctypes.POINTER(ctypes.c_int16)),values)
            output.append([start/fs,*values])
        output=np.array(output)
        clips=[]
        selected=metadata['clips'] or [dict(name='ambient',actual_start=0,duration=len(pcm)/48000)]
        for clip in selected:
            start=clip.get('actual_start',clip.get('start',0))
            rows=output[(output[:,0]>=start)&(output[:,0]<=start+clip['duration']+.15)]
            valid=rows[rows[:,10]>.5]
            clips.append(dict(name=clip['name'],updated_frames=len(valid),
                mean_angle=math.degrees(math.atan2(np.sin(np.deg2rad(valid[:,1])).sum(),np.cos(np.deg2rad(valid[:,1])).sum()))%360 if len(valid) else None,
                angle_range=[float(valid[:,1].min()),float(valid[:,1].max())] if len(valid) else None,
                source_frames=int((rows[:,8]>.5).sum()),quality_frames=int((rows[:,9]>.5).sum()),
                peak_score=float(rows[:,3].max()),peak_snr=float((rows[:,6]/rows[:,7]).max())))
        result[variant]=dict(fields=['time',*FIELDS],rows=output.tolist(),clips=clips)
        print(variant,json.dumps(clips),flush=True)
    args.out.write_text(json.dumps(result,indent=2),encoding='utf8')


if __name__=='__main__':main()
