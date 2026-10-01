import pathlib
import os
import subprocess


ROOT = pathlib.Path(__file__).resolve().parents[1]
from msvc import VCVARS, CL


def test_tracker_wraps_and_holds_invalid_frames(tmp_path):
    exe = tmp_path / "doa_tracker.exe"
    output = subprocess.check_output(f'cmd /c ""{VCVARS}" >nul && set"', text=True)
    env = os.environ.copy()
    env.update(line.split("=", 1) for line in output.splitlines() if "=" in line)
    subprocess.run([
        str(CL), "/nologo", "/std:c11", "/W4", f'/I{ROOT / "main" / "doa"}',
        str(ROOT / "main/doa/doa_tracker.c"), str(ROOT / "tests/test_doa_tracker_main.c"),
        f'/Fe:{exe}',
    ], check=True, env=env)
    subprocess.run([str(exe)], check=True)
