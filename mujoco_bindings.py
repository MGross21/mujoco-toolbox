import os
import subprocess
import sys
import shutil
from pathlib import Path
import mujoco as mj

MUJOCO_REPO = "https://github.com/deepmind/mujoco.git"
CLONE_DIR = Path("mujoco")
PYTHON_DIR = CLONE_DIR / "python"
STUBS_DIR = Path("stubs")
# MUJOCO_VERSION_FOLDER = "mujoco" + getattr(mj, "__version__", "unknown").replace(".", "")
MUJOCO_VERSION_FOLDER = "mujoco330"

DEFAULT_MUJOCO_PATH = Path.home() / ".mujoco" / MUJOCO_VERSION_FOLDER

def run(cmd, cwd=None):
    print(f"\n> Running: {' '.join(cmd)}")
    try:
        subprocess.run(cmd, check=True, cwd=cwd)
    except subprocess.CalledProcessError as e:
        print(f"\n❌ Command failed: {' '.join(cmd)}")
        print(e)
        sys.exit(1)

def set_mujoco_path():
    if not DEFAULT_MUJOCO_PATH.exists():
        print(f"❌ Expected MuJoCo path not found: {DEFAULT_MUJOCO_PATH}")
        print("👉 Please download and extract MuJoCo to this path or update the script.")
        print("🔗 https://github.com/deepmind/mujoco/releases")
        sys.exit(1)

    os.environ["MUJOCO_PATH"] = str(DEFAULT_MUJOCO_PATH)
    print(f"✔️ MUJOCO_PATH set to: {DEFAULT_MUJOCO_PATH}")

    # Also set MUJOCO_PLUGIN_PATH
    plugin_path = DEFAULT_MUJOCO_PATH / "plugins"
    plugin_path.mkdir(exist_ok=True)
    os.environ["MUJOCO_PLUGIN_PATH"] = str(plugin_path)
    print(f"✔️ MUJOCO_PLUGIN_PATH set to: {plugin_path}")

def check_stubgen_installed():
    if not shutil.which("stubgen"):
        print("❌ `stubgen` not found. Please install it with:")
        print("   pip install mypy")
        sys.exit(1)

def clone_repo():
    if not CLONE_DIR.exists():
        run(["git", "clone", MUJOCO_REPO])
    else:
        print("✔️ MuJoCo repo already cloned.")

def install_editable():
    run(["pip", "install", "-e", "."], cwd=PYTHON_DIR)

def generate_stubs():
    STUBS_DIR.mkdir(exist_ok=True)
    run(["stubgen", "-m", "mujoco", "-o", str(STUBS_DIR)])

def print_success():
    print("\n🎉 All done!")
    print("✔️ MuJoCo cloned")
    print("✔️ Python bindings installed in editable mode")
    print(f"✔️ Stubs generated in `{STUBS_DIR}/mujoco.pyi`\n")
    print("👉 Add this to your `mypy.ini` or set `MYPYPATH`:\n")
    print(f"[mypy]\nmypy_path = {STUBS_DIR}\n")

if __name__ == "__main__":
    set_mujoco_path()
    check_stubgen_installed()
    clone_repo()
    install_editable()
    generate_stubs()
    print_success()
