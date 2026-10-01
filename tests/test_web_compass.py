import json
import math
import pathlib
import re
import subprocess
from html.parser import HTMLParser


ROOT = pathlib.Path(__file__).resolve().parents[1]


class _DashboardParser(HTMLParser):
    def __init__(self):
        super().__init__(convert_charrefs=True)
        self.ids = set()
        self.text = []

    def handle_starttag(self, tag, attrs):
        del tag
        for name, value in attrs:
            if name == "id" and value:
                self.ids.add(value)

    def handle_data(self, data):
        self.text.append(data)


def _js_function(script, name):
    match = re.search(
        rf"function\s+{name}\([^)]*\)\{{.*?\n\}}",
        script,
        flags=re.DOTALL,
    )
    assert match, f"{name} JavaScript function not found"
    return match.group(0)


def _evaluate_display_angles(html, angles):
    script = re.search(r"<script>(.*)</script>", html, flags=re.DOTALL)
    assert script, "dashboard script not found"
    js = "\n".join([
        _js_function(script.group(1), "norm360"),
        _js_function(script.group(1), "displayAngle"),
        f"console.log(JSON.stringify({json.dumps(angles)}.map(displayAngle)));",
    ])
    result = subprocess.run(
        ["node", "-e", js], check=True, capture_output=True, text=True
    )
    return json.loads(result.stdout)


def test_compass_pointer_uses_same_rotation_as_physical_labels():
    html = (ROOT / "main" / "wifi" / "web_page.html").read_text(encoding="utf-8")
    match = re.search(r"var rad=\(angle\+pointerOffset-(\d+)\)\*Math\.PI/180;", html)
    assert match, "compass pointer formula not found"

    offset_deg = 45 - int(match.group(1))
    positions = {
        0: (math.cos(math.radians(0 + offset_deg)), math.sin(math.radians(0 + offset_deg))),
        90: (math.cos(math.radians(90 + offset_deg)), math.sin(math.radians(90 + offset_deg))),
        180: (math.cos(math.radians(180 + offset_deg)), math.sin(math.radians(180 + offset_deg))),
        270: (math.cos(math.radians(270 + offset_deg)), math.sin(math.radians(270 + offset_deg))),
    }

    assert positions[0][0] < 0 and positions[0][1] < 0
    assert positions[90][0] > 0 and positions[90][1] < 0
    assert positions[180][0] > 0 and positions[180][1] > 0
    assert positions[270][0] < 0 and positions[270][1] > 0


def test_web_page_does_not_expose_zero_degree_calibration_button():
    html = (ROOT / "main" / "wifi" / "web_page.html").read_text(encoding="utf-8")
    assert "calibrate0" not in html
    assert "校准 0°" not in html


def test_dashboard_displays_unsigned_angles_and_no_distance():
    html = (ROOT / "main" / "wifi" / "web_page.html").read_text(encoding="utf-8")
    assert _evaluate_display_angles(html, [-90.0, 360.0, 315.0]) == [270.0, 0.0, 315.0]

    parser = _DashboardParser()
    parser.feed(html)
    assert "distVal" not in parser.ids
    assert "估算距离" not in "".join(parser.text)
