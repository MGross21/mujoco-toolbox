"""Generate type stubs for the installed ``mujoco`` package into ``typings/``.

MuJoCo's wheels ship compiled pybind11 bindings without ``.pyi`` files, so
mypy sees every ``mujoco`` call as ``Any``. Rerun after changing the mujoco
version:

    uv run python scripts/generate_stubs.py
"""

from __future__ import annotations

import ast
import keyword
import os
import re
import shutil
import subprocess
import sys
from importlib.metadata import version
from pathlib import Path

OUT = Path(__file__).resolve().parent.parent / "typings"
# Module constants such as HEADERS_DIR hold machine-specific paths; keep only their types.
CONSTANT = re.compile(r"^(?!__all__)(\w+: [^=]+?) = .+$")


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

    - Module-level constant values are machine-specific (paths, OS); keep the type.
    - Attributes named after keywords (``global``) cannot be declared; drop them.
    - Stubgen omits C++ defaults it cannot render, leaving required parameters
      after optional ones. Give those ``= ...``.
    """
    if match := CONSTANT.match(line.rstrip("\n")):
        return f"{match.group(1)}\n"
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
    parts = [params[i + 1 : j] for i, j in zip([-1, *cuts], [*cuts, len(params)], strict=True)]
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
    """Write fixed-up ``mujoco`` stubs into ``out``."""
    shutil.rmtree(out / "mujoco", ignore_errors=True)
    subprocess.run(  # noqa: S603
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
        env={**os.environ, "MUJOCO_GL": "glfw"},  # picks which GLContext mujoco re-exports
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    for path in out.rglob("*.pyi"):
        lines = path.read_text().splitlines(keepends=True)
        text = "".join(_fix_line(line) for line in lines)
        ast.parse(text, filename=str(path))
        path.write_text(text)


def main() -> int:
    """Regenerate ``typings/``."""
    generate(OUT)
    print(f"Generated stubs for mujoco {version('mujoco')} in {OUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
