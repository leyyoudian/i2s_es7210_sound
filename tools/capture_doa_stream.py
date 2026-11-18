"""Record continuous loss-checked 48kHz/4ch PCM from DOA_PCM_STREAM firmware.

Only use the resulting WAV for offline timing analysis when crc_errors,
sequence_gaps and dma_overflows are all zero. Restore normal firmware after
capture. Optional real-recording playback uses the same preparation as benches.
"""
import argparse
import json
from pathlib import Path
import struct
import time
import wave
import winsound
import zlib

import serial
from real_audio_bench import playback_file

MAGIC=b'DOAPCM2!'
HEADER=struct.Struct('<8s6I')


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port',required=True)
    ap.add_argument('--manifest',type=Path)
    ap.add_argument('--offset',type=int,default=0)
    ap.add_argument('--count',type=int,default=4)
    ap.add_argument('--gain',type=float,default=1)
    ap.add_argument('--quiet-seconds',type=float,default=3)
    ap.add_argument('--out',type=Path,required=True)
    args=ap.parse_args()
    args.out.parent.mkdir(parents=True,exist_ok=True)
    prepared=[]
    next_time=args.quiet_seconds
    if args.manifest:
        items=json.loads(args.manifest.read_text(encoding='utf8'))
        for item in items[args.offset:args.offset+args.count]:
            path,duration=playback_file(item,args.gain)
            prepared.append(dict(name=item['name'],wav=str(path),start=next_time,duration=duration))
            next_time+=duration+1.5
    duration=next_time+2 if prepared else args.quiet_seconds
    frames=0
    packets=[]
    buffer=bytearray()
    gaps=crc_errors=0
    previous_sequence=None
    current_clip=0
    epoch=0
    with serial.Serial(args.port,115200,timeout=.025) as ser, wave.open(str(args.out),'wb') as wav:
        wav.setparams((4,2,48000,0,'NONE','not compressed'))
        ser.write(b'S')
        drain=time.monotonic()+.8
        while time.monotonic()<drain:ser.read(ser.in_waiting or 1)
        ser.reset_input_buffer()
        epoch=time.monotonic()
        ser.write(b'R')
        print('Recording continuous 4-channel PCM for',round(duration,2),'seconds',flush=True)
        try:
            while time.monotonic()-epoch<duration:
                now=time.monotonic()-epoch
                if current_clip<len(prepared) and now>=prepared[current_clip]['start']:
                    clip=prepared[current_clip]
                    winsound.PlaySound(clip['wav'],winsound.SND_FILENAME|winsound.SND_ASYNC)
                    clip['actual_start']=time.monotonic()-epoch
                    print('Playing',clip['name'],'at',round(clip['actual_start'],3),flush=True)
                    current_clip+=1
                buffer.extend(ser.read(min(65536,max(4096,ser.in_waiting))))
                while True:
                    start=buffer.find(MAGIC)
                    if start<0:
                        if len(buffer)>7:del buffer[:-7]
                        break
                    if start:del buffer[:start]
                    if len(buffer)<HEADER.size:break
                    _,seq,n,fs,overflow,device_ms,crc=HEADER.unpack_from(buffer)
                    if not 0<n<=1024 or fs!=48000:
                        del buffer[0]
                        continue
                    size=HEADER.size+n*8
                    if len(buffer)<size:break
                    payload=bytes(buffer[HEADER.size:size]);del buffer[:size]
                    if zlib.crc32(payload)!=crc:
                        crc_errors+=1
                        if crc_errors>=3:raise RuntimeError('PCM CRC mismatch')
                        continue
                    if previous_sequence is not None and seq!=previous_sequence+1:gaps+=1
                    previous_sequence=seq
                    packets.append(dict(sequence=seq,frames=n,sample_offset=frames,
                                        device_ms=device_ms,received=time.monotonic()-epoch,
                                        overflow=overflow))
                    wav.writeframesraw(payload)
                    frames+=n
        finally:
            ser.write(b'S')
            winsound.PlaySound(None,0)
    result=dict(sample_rate=48000,channels=4,frames=frames,recorded_seconds=frames/48000,
                crc_errors=crc_errors,sequence_gaps=gaps,
                dma_overflows=max((p['overflow'] for p in packets),default=0),
                clips=prepared,packets=packets)
    args.out.with_suffix('.json').write_text(json.dumps(result,indent=2),encoding='utf8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('clips','packets')}),flush=True)
    if crc_errors or gaps or result['dma_overflows']:
        raise RuntimeError('Capture has losses; do not treat the WAV as continuous')


if __name__=='__main__':main()
