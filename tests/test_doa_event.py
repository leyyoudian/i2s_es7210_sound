import os
import pathlib
import subprocess


ROOT = pathlib.Path(__file__).resolve().parents[1]
from msvc import VCVARS, CL


def test_weak_taps_use_fast_path_without_accepting_reflections(tmp_path):
    exe = tmp_path / "doa_event.exe"
    output = subprocess.check_output(f'cmd /c ""{VCVARS}" >nul && set"', text=True)
    env = os.environ.copy()
    env.update(line.split("=", 1) for line in output.splitlines() if "=" in line)
    subprocess.run([
        str(CL), "/nologo", "/std:c11", "/W4", f'/I{ROOT / "main" / "doa"}',
        str(ROOT / "main/doa/doa_event.c"), str(ROOT / "tests/test_doa_event_main.c"),
        f'/Fe:{exe}',
    ], check=True, env=env)
    subprocess.run([str(exe)], check=True)
