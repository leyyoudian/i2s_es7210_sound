import os
import pathlib
import subprocess
from msvc import VCVARS, CL

ROOT = pathlib.Path(__file__).resolve().parents[1]

def test_strength_breath_motion_multisource_and_fade(tmp_path):
    output = subprocess.check_output(f'cmd /c ""{VCVARS}" >nul && set"', text=True)
    env = os.environ.copy()
    env.update(line.split('=', 1) for line in output.splitlines() if '=' in line)
    executable = tmp_path / 'led_effects.exe'
    subprocess.run([str(CL), '/nologo', '/utf-8', '/std:c11', '/W4',
                    '/I'+str(ROOT/'main/led'), '/I'+str(ROOT/'main/doa'),
                    str(ROOT/'main/led/led_effects.c'), str(ROOT/'main/doa/doa_led_scene.c'),
                    str(ROOT/'main/doa/direction_mapping.c'),
                    str(ROOT/'tests/test_led_effects_main.c'), '/Fe:'+str(executable)],
                   check=True, env=env, cwd=tmp_path)
    subprocess.run([str(executable)], check=True)
