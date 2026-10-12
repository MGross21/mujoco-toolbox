"""Generate type stubs for the installed ``mujoco`` package into ``typings/``.

MuJoCo's wheels ship compiled pybind11 bindings without ``.pyi`` files, so
mypy sees every ``mujoco`` call as ``Any``. Rerun after changing the mujoco
version:

    uv run --python 3.13 python scripts/generate_stubs.py

Python 3.13 is required because docstring indentation in the output differs
between interpreter versions. ``--check`` exits non-zero if ``typings/`` is stale.
"""

from __future__ import annotations

import argparse
import ast
import filecmp
import tempfile
import keyword
import shutil
import subprocess
import sys
from pathlib import Path

OUT = Path(__file__).resolve().parent.parent / "typings"
PYTHON = (3, 13)


def _top_level(text: str) -> list[tuple[int, str]]:
    """Characters of ``text`` outside any brackets, with their indices."""
    out, depth = [], 0
    for i, ch in enumerate(text):
        if ch in "([{":
            depth += 1
        elif ch in ")]}":
            depth -= 1
        elif depth == 0:
            out.append((i, ch))
    return out


def _fix_line(line: str) -> str:
    """Make one line of pybind11-stubgen output valid Python.

    - Attributes named after keywords (``global``) cannot be declared; drop them.
    - Stubgen omits C++ defaults it cannot render, leaving required parameters
      after optional ones. Give those ``= ...``.
    """
    stripped = line.lstrip()
    name = stripped.split(":", 1)[0]
    if ":" in stripped and keyword.iskeyword(name):
        return ""
    if not stripped.startswith("def "):
        return line

    start = line.index("(") + 1
    depth = 1
    for end in range(start, len(line)):
        depth += {"(": 1, ")": -1}.get(line[end], 0)
        if depth == 0:
            break

    params = line[start:end]
    cuts = [i for i, ch in _top_level(params) if ch == ","]
    parts = [params[i + 1 : j] for i, j in zip([-1, *cuts], [*cuts, len(params)])]
    seen_default = False
    for k, part in enumerate(parts):
        if part.strip().startswith("*"):
            break
        if any(ch == "=" for _, ch in _top_level(part)):
            seen_default = True
        elif seen_default and part.strip() not in ("", "/"):
            parts[k] = f"{part} = ..."
    return f"{line[:start]}{','.join(parts)}{line[end:]}"


def generate(out: Path) -> None:
    shutil.rmtree(out / "mujoco", ignore_errors=True)
    subprocess.run(
        [
            sys.executable,
            "-m",
            "pybind11_stubgen",
            "mujoco",
            "--output-dir",
            str(out),
            "--ignore-invalid-expressions",
            r".*::.*",  # raw C++ types in mujoco docstrings
            "--ignore-unresolved-names",
            r"mjs\w+_",  # internal spec structs
            "--exit-code",
        ],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    for path in out.rglob("*.pyi"):
        lines = path.read_text().splitlines(keepends=True)
        text = "".join(_fix_line(line) for line in lines)
        ast.parse(text, filename=str(path))
        path.write_text(text)


def _stale(expected: Path, actual: Path) -> list[str]:
    exp = {p.relative_to(expected) for p in expected.rglob("*.pyi")}
    act = {p.relative_to(actual) for p in actual.rglob("*.pyi")}
    changed = [str(p) for p in exp & act if not filecmp.cmp(expected / p, actual / p, shallow=False)]
    return sorted(changed + [str(p) for p in exp ^ act])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="fail if typings/ is out of date")
    args = parser.parse_args()

    if sys.version_info[:2] != PYTHON:
        print(f"Run with Python {PYTHON[0]}.{PYTHON[1]}: uv run --python 3.13 python {__file__}", file=sys.stderr)
        return 2

    import mujoco

    if not args.check:
        generate(OUT)
        print(f"Generated stubs for mujoco {mujoco.__version__} in {OUT}")
        return 0

    with tempfile.TemporaryDirectory() as tmp:
        generate(Path(tmp))
        stale = _stale(Path(tmp), OUT)
    if stale:
        print(f"typings/ is out of date for mujoco {mujoco.__version__}:", *stale, sep="\n  ", file=sys.stderr)
        print("Regenerate: uv run --python 3.13 python scripts/generate_stubs.py", file=sys.stderr)
        return 1
    print(f"typings/ is up to date for mujoco {mujoco.__version__}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
