import os
import pathlib
import subprocess


ROOT = pathlib.Path(__file__).resolve().parents[1]
from msvc import VCVARS, CL


def test_bird_decision_requires_consecutive_windows(tmp_path):
    exe = tmp_path / "bird_decision.exe"
    output = subprocess.check_output(f'cmd /c ""{VCVARS}" >nul && set"', text=True)
    env = os.environ.copy()
    env.update(line.split("=", 1) for line in output.splitlines() if "=" in line)
    subprocess.run([
        str(CL), "/nologo", "/std:c11", "/W4", f'/I{ROOT / "main" / "bird_ai"}',
        str(ROOT / "main/bird_ai/bird_decision.c"), str(ROOT / "tests/test_bird_decision_main.c"),
        f'/Fe:{exe}',
    ], check=True, env=env)
    subprocess.run([str(exe)], check=True)
