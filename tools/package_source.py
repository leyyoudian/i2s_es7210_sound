"""Create a shareable source ZIP from a Git commit, then verify its contents.

No .git history, ignored recordings, build outputs or private Wi-Fi headers.
The commit is authoritative: uncommitted/untracked work is not silently packed.
"""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[1]
PREFIX = 'i2s_es7210_sound/'
FORBIDDEN_DIRS = {'.git', '.fetch', '.vscode', '.pytest_cache', '__pycache__',
                  'build', 'build-current', 'managed_components', 'uploads', 'videos',
                  '.venv', 'venv', 'node_modules', 'memory'}
FORBIDDEN_SUFFIXES = {'.bin', '.elf', '.map', '.obj', '.o', '.exe', '.dll', '.exp',
                      '.log', '.wav', '.mp3', '.ogg', '.mp4', '.pyc', '.pyo', '.zip'}


def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT, text=True).strip()


def inspect_archive(path, private_values=()):
    with zipfile.ZipFile(path) as archive:
        bad = archive.testzip()
        if bad:
            raise RuntimeError('ZIP CRC failure: '+bad)
        files = []
        for info in archive.infolist():
            if info.is_dir():
                continue
            file = PurePosixPath(info.filename)
            relative = PurePosixPath(*file.parts[1:])
            if not info.filename.startswith(PREFIX) or '..' in file.parts:
                raise RuntimeError('Unexpected archive path: '+info.filename)
            if any(part in FORBIDDEN_DIRS or part.startswith('build-') for part in relative.parts):
                raise RuntimeError('Local/generated directory in archive: '+str(relative))
            if relative.name == 'wifi_credentials.h' or relative.suffix.lower() in FORBIDDEN_SUFFIXES:
                raise RuntimeError('Private/generated file in archive: '+str(relative))
            data = archive.read(info)
            if any(value and value in data for value in private_values):
                # Never print the matched credential value.
                raise RuntimeError('Private network value found in: '+str(relative))
            files.append(str(relative))
        required = {'README.md', 'CMakeLists.txt', 'sdkconfig', 'partitions.csv',
                    'main/main.c', 'main/led/led_effects_config.h',
                    'main/wifi/wifi_credentials.example.h', 'tools/requirements.txt',
                    'bird-sound-cpp-mcu-v2/edge-impulse-sdk/LICENSE'}
        if not required.issubset(files):
            raise RuntimeError('Missing required source files: '+', '.join(sorted(required-set(files))))
        return dict(files=len(files), uncompressed_bytes=sum(x.file_size for x in archive.infolist()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ref', default='HEAD')
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    commit = git('rev-parse', '--verify', args.ref+'^{commit}')
    target = args.out.resolve()
    if target.exists():
        raise FileExistsError('Archive already exists; choose a new name: '+str(target))
    target.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(['git', 'archive', '--format=zip', '--prefix='+PREFIX,
                    '--output='+str(target), commit], cwd=ROOT, check=True)
    source = dict(commit=commit, committed_at=git('show', '-s', '--format=%cI', commit),
                  repository='https://github.com/leyyoudian/i2s_es7210_sound',
                  hardware='https://oshwhub.com/ley6666/project_labyaziz',
                  package='Source only; no private Wi-Fi configuration, recordings, build outputs or Git history.')
    with zipfile.ZipFile(target, 'a', compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr(PREFIX+'SOURCE_VERSION.json', json.dumps(source, ensure_ascii=False, indent=2)+'\n')
    private_values = []
    private_file = ROOT/'main/wifi/wifi_credentials.h'
    if private_file.exists():
        text = private_file.read_text(encoding='utf-8')
        private_values = [value.encode('utf-8') for value in
                          re.findall(r'#define\s+WIFI_(?:SSID|PASSWORD)\s+"([^"\n]+)"', text)]
    result = inspect_archive(target, private_values)
    digest = hashlib.sha256(target.read_bytes()).hexdigest()
    target.with_suffix(target.suffix+'.sha256').write_text(digest+'  '+target.name+'\n', encoding='utf-8')
    print(json.dumps(dict(path=str(target), commit=commit, sha256=digest,
                          zip_bytes=target.stat().st_size, **result), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
