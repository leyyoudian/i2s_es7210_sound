"""Locate the newest installed MSVC BuildTools (vcvars64.bat + cl.exe).

Tests used to hardcode a VS2019 path; auto-detection keeps them working
across BuildTools upgrades (e.g. VS2019 -> VS2022).
"""
import pathlib

_SEARCH_ROOTS = (
    pathlib.Path("C:/Program Files (x86)/Microsoft Visual Studio"),
    pathlib.Path("C:/Program Files/Microsoft Visual Studio"),
)


def _find():
    year_dirs = []
    for root in _SEARCH_ROOTS:
        if root.is_dir():
            year_dirs.extend(p for p in root.iterdir() if p.is_dir())
    for year in sorted(year_dirs, reverse=True):
        build_tools = year / "BuildTools"
        vcvars = build_tools / "VC" / "Auxiliary" / "Build" / "vcvars64.bat"
        msvc_root = build_tools / "VC" / "Tools" / "MSVC"
        if not vcvars.exists() or not msvc_root.is_dir():
            continue
        for version in sorted(msvc_root.iterdir(), reverse=True):
            cl = version / "bin" / "Hostx64" / "x64" / "cl.exe"
            if cl.exists():
                return vcvars, cl
    raise FileNotFoundError(
        "MSVC BuildTools x64 not found; install 'Desktop development with C++'")


VCVARS, CL = _find()
