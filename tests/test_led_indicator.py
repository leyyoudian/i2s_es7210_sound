import os
import pathlib
import subprocess


ROOT = pathlib.Path(__file__).resolve().parents[1]
from msvc import VCVARS, CL


def test_led_switch_requires_three_frames_and_refreshes_clear_first(tmp_path):
    executable = tmp_path / "led_indicator.exe"
    output = subprocess.check_output(f'cmd /c ""{VCVARS}" >nul && set"', text=True)
    env = os.environ.copy()
    env.update(line.split("=", 1) for line in output.splitlines() if "=" in line)
    subprocess.run([
        str(CL), "/nologo", "/std:c11", "/W4",
        f'/I{ROOT / "tests/fakes"}', f'/I{ROOT / "main" / "led"}',
        f'/I{ROOT / "main" / "doa"}',
        str(ROOT / "main/led/led_indicator.c"),
        str(ROOT / "main/doa/direction_mapping.c"),
        str(ROOT / "tests/test_led_indicator_main.c"),
        f'/Fe:{executable}',
    ], check=True, cwd=tmp_path, env=env)
    subprocess.run([str(executable)], check=True)
