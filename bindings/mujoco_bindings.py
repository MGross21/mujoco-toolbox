import os
import shutil
import subprocess
import sys
import urllib.request
import zipfile
from pathlib import Path

# ------------------------- CONFIG -------------------------
subprocess.run(["pip", "install", "mujoco"], check=False)
import mujoco

MUJOCO_VERSION = mujoco.__version__
MUJOCO_SHORT = MUJOCO_VERSION.replace(".", "")
MUJOCO_ZIP_URL = (
    f"https://github.com/deepmind/mujoco/releases/download/{MUJOCO_VERSION}/mujoco-{MUJOCO_VERSION}-windows-x86_64.zip"
)
MUJOCO_DEST = Path.home() / ".mujoco" / f"mujoco{MUJOCO_SHORT}"
MUJOCO_REPO = "https://github.com/deepmind/mujoco.git"
REPO_DIR = Path("mujoco")
PYTHON_BINDINGS = REPO_DIR / "python"
STUBS_DIR = Path("stubs")


# ------------------------- UTILITIES -------------------------
def run(cmd, check=True, cwd=None) -> None:
    print(f"\n> Running: {' '.join(cmd)}")
    try:
        subprocess.run(cmd, check=check, cwd=cwd, shell=True)
    except subprocess.CalledProcessError as e:
        if e.returncode == 2316632107:
            print("⚠️ Already installed or no upgrade found. Continuing...")
        else:
            print(f"❌ Command failed: {' '.join(cmd)}")
            print(e)
            sys.exit(e.returncode)


# ------------------------- SETUP FUNCTIONS -------------------------
def download_and_extract_zip() -> None:
    zip_path = Path("mujoco.zip")
    print(f"📥 Downloading MuJoCo {MUJOCO_VERSION}...")
    urllib.request.urlretrieve(MUJOCO_ZIP_URL, zip_path)
    print("📦 Extracting MuJoCo...")
    with zipfile.ZipFile(zip_path, "r") as zip_ref:
        zip_ref.extractall(MUJOCO_DEST.parent)
    zip_path.unlink()
    print(f"✔️ Extracted to: {MUJOCO_DEST}")


def install_tools() -> None:
    tools = {
        "cmake": [
            "winget",
            "install",
            "--id=Kitware.CMake",
            "--silent",
            "--accept-package-agreements",
            "--accept-source-agreements",
        ],
        "cl": [
            "winget",
            "install",
            "--id=Microsoft.VisualStudio.2022.BuildTools",
            "--silent",
            "--accept-package-agreements",
            "--accept-source-agreements",
        ],
        "stubgen": [sys.executable, "-m", "pip", "install", "mypy"],
    }
    for tool, install_cmd in tools.items():
        if not shutil.which(tool):
            print(f"📦 Installing {tool}...")
            run(install_cmd)
        else:
            print(f"✔️ {tool} already installed.")


def check_python_version() -> None:
    major, minor = sys.version_info[:2]
    if major != 3 or not (10 <= minor <= 12):
        print(f"❌ Python {major}.{minor} is not supported. Please use 3.10-3.12.")
        sys.exit(1)
    print(f"✔️ Python version {major}.{minor} is compatible.")


def setup_env_vars() -> None:
    plugin_path = MUJOCO_DEST / "plugins"
    plugin_path.mkdir(exist_ok=True)
    os.environ["MUJOCO_PATH"] = str(MUJOCO_DEST)
    os.environ["MUJOCO_PLUGIN_PATH"] = str(plugin_path)
    print(f"✔️ Set MUJOCO_PATH: {MUJOCO_DEST}")
    print(f"✔️ Set MUJOCO_PLUGIN_PATH: {plugin_path}")


def clone_mujoco() -> None:
    if not REPO_DIR.exists():
        print("📥 Cloning MuJoCo repo...")
        run(["git", "clone", MUJOCO_REPO])
    else:
        print("✔️ MuJoCo repo already cloned.")


def install_python_bindings() -> None:
    print("📦 Installing MuJoCo Python bindings (editable mode)...")
    run(["pip", "install", "-e", "python"], cwd=REPO_DIR)


def generate_stubs() -> None:
    print("🧠 Generating .pyi stub files...")
    STUBS_DIR.mkdir(exist_ok=True)
    run(["stubgen", "-m", "mujoco", "-o", str(STUBS_DIR)])


def print_finish() -> None:
    print("\n🎉 Setup complete!")
    print(f"✔️ MuJoCo engine at: {MUJOCO_DEST}")
    print(f"✔️ Python bindings installed from: {PYTHON_BINDINGS}")
    print(f"✔️ Stub files generated at: {STUBS_DIR}/mujoco.pyi")
    print("\n👉 Add this to your mypy.ini:")
    print(f"[mypy]\nmypy_path = {STUBS_DIR}")


# ------------------------- MAIN -------------------------
if __name__ == "__main__":
    check_python_version()
    install_tools()

    if not MUJOCO_DEST.exists():
        download_and_extract_zip()
    else:
        print(f"✔️ MuJoCo already exists at: {MUJOCO_DEST}")

    setup_env_vars()
    clone_mujoco()
    install_python_bindings()
    generate_stubs()
    print_finish()
    print("✔️ All done!")
