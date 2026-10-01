import pathlib
import os
import subprocess


ROOT = pathlib.Path(__file__).resolve().parents[1]
from msvc import VCVARS, CL


def test_four_microphone_srp_phat_finds_known_directions(tmp_path):
    exe = tmp_path / "srp_phat.exe"
    output = subprocess.check_output(f'cmd /c ""{VCVARS}" >nul && set"', text=True)
    env = os.environ.copy()
    env.update(line.split("=", 1) for line in output.splitlines() if "=" in line)
    subprocess.run([
        str(CL), "/nologo", "/std:c11", "/W4",
        f'/I{ROOT / "tests" / "stubs"}', f'/I{ROOT / "main" / "doa"}',
        str(ROOT / "main/doa/gcc_phat.c"), str(ROOT / "tests/test_srp_phat_main.c"),
        f'/Fe:{exe}',
    ], check=True, env=env)
    subprocess.run([str(exe)], check=True)
