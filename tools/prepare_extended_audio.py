"""Cache diverse public real recordings for physical speaker/DOA testing.

ESC-50 source takes from fold 2 broaden the original fold-1 suite. Speech is
real audiobook narration from the librosa audio registry. Original files and
source hashes are retained; converted speech clips are contiguous excerpts.
"""
import concurrent.futures
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import urllib.request

from real_audio_bench import github_file, ROOT

CACHE = ROOT / '.fetch/extended-audio'
CATEGORIES = ('cat', 'crying_baby', 'laughing', 'footsteps', 'glass_breaking',
              'can_opening', 'keyboard_typing', 'siren', 'engine',
              'vacuum_cleaner', 'rain', 'wind')


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--esc50-dir',type=Path,help='Optional existing ESC-50 audio/ directory')
    args=ap.parse_args()
    CACHE.mkdir(parents=True, exist_ok=True)
    metadata=CACHE/'esc50.csv'
    if not metadata.exists():
        metadata.write_bytes(github_file('karolpiczak/ESC-50','meta/esc50.csv'))
    rows = list(csv.DictReader(metadata.open(encoding='utf8')))
    selected=[]
    for category in CATEGORIES:
        choices=[r for r in rows if r['category']==category and r['fold']=='2'][:2]
        if len(choices)!=2:
            raise ValueError(f'Expected two real clips for {category}')
        selected.extend(choices)

    def fetch(row):
        file=CACHE/row['filename']
        if not file.exists():
            try:
                file.write_bytes(github_file('karolpiczak/ESC-50','audio/'+row['filename']))
            except Exception:
                local=args.esc50_dir/row['filename'] if args.esc50_dir else None
                if local is None or not local.exists(): raise
                file.write_bytes(local.read_bytes())
                print('Used existing ESC-50 cache:',row['filename'],flush=True)
        item=dict(name=row['category']+'-'+row['filename'],category=row['category'],
                  file=str(file),source='https://github.com/karolpiczak/ESC-50/blob/master/audio/'+row['filename'],
                  attribution=f'https://freesound.org/s/{row["src_file"]}/',
                  sha256=hashlib.sha256(file.read_bytes()).hexdigest(),
                  license='ESC-50 and original Freesound source attribution')
        print('Ready',item['name'],flush=True)
        return item

    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
        items=list(pool.map(fetch,selected))

    registry_path=CACHE/'librosa-registry.txt'
    if not registry_path.exists():
        registry_path.write_bytes(github_file('librosa/librosa','librosa/util/example_data/registry.txt'))
    registry=registry_path.read_text(encoding='utf8').splitlines()
    for basename,reader in [('5703-47212-0000','Garth Comira'),('198-209-0000','Heather Barnett')]:
        original=CACHE/(basename+'.ogg')
        url='https://librosa.org/data/audio/'+basename+'.ogg'
        if not original.exists():
            request=urllib.request.Request(url,headers={'User-Agent':'LocalAcousticBench'})
            original.write_bytes(urllib.request.urlopen(request,timeout=30).read())
        digest=hashlib.sha256(original.read_bytes()).hexdigest()
        expected=next(r.split()[-1] for r in registry if r.startswith(basename+'.ogg '))
        if digest!=expected:
            raise ValueError('Audio registry SHA256 mismatch: '+basename)
        file=CACHE/(basename+'-8s.wav')
        subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-i',str(original),
                        '-t','8','-ac','1','-ar','48000','-c:a','pcm_s16le',str(file)],check=True)
        items.append(dict(name='speech-'+reader.replace(' ','_'),category='speech',file=str(file),
                          source=url,reader=reader,original_sha256=digest,
                          sha256=hashlib.sha256(file.read_bytes()).hexdigest(),
                          license='LibriSpeech SLR12 recording, CC BY 4.0; see librosa attribution'))
        print('Ready speech',reader,flush=True)
    manifest=CACHE/'manifest.json'
    manifest.write_text(json.dumps(items,ensure_ascii=False,indent=2),encoding='utf8')
    print('Suite:',len(items),'real recordings ->',manifest)


if __name__=='__main__':
    main()
