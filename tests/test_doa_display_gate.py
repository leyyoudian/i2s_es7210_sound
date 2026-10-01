import os
import pathlib
import subprocess


ROOT = pathlib.Path(__file__).resolve().parents[1]
from msvc import VCVARS, CL


def test_display_gate_blanks_stale_weak_rejections_without_resetting_tracker(tmp_path):
    exe = tmp_path / "doa_display_gate.exe"
    output = subprocess.check_output(f'cmd /c ""{VCVARS}" >nul && set"', text=True)
    env = os.environ.copy()
    env.update(line.split("=", 1) for line in output.splitlines() if "=" in line)
    subprocess.run([
        str(CL), "/nologo", "/std:c11", "/W4", f'/I{ROOT / "main" / "doa"}',
        str(ROOT / "main/doa/doa_display_gate.c"),
        str(ROOT / "main/doa/direction_mapping.c"),
        str(ROOT / "tests/test_doa_display_gate_main.c"),
        f'/Fe:{exe}',
    ], check=True, env=env)
    subprocess.run([str(exe)], check=True)
