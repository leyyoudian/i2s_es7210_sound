import pathlib
import os
import subprocess


ROOT = pathlib.Path(__file__).resolve().parents[1]
from msvc import VCVARS, CL


def test_direction_mapping_matches_physical_led_positions(tmp_path):
    executable = tmp_path / "direction_mapping.exe"
    output = subprocess.check_output(f'cmd /c ""{VCVARS}" >nul && set"', text=True)
    env = os.environ.copy()
    env.update(line.split("=", 1) for line in output.splitlines() if "=" in line)
    subprocess.run(
        [
            str(CL), "/nologo", "/std:c11", "/W4",
            f"/I{ROOT / 'main' / 'doa'}",
            str(ROOT / "main" / "doa" / "direction_mapping.c"),
            str(ROOT / "tests" / "test_direction_mapping_main.c"),
            f"/Fe:{executable}",
        ],
        check=True,
        env=env,
    )
    subprocess.run([str(executable)], check=True)
