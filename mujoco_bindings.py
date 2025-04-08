import os
import sys
import subprocess
import shutil
import zipfile
from pathlib import Path
import urllib.request

# ------------------------- CONFIG -------------------------

MUJOCO_VERSION = "3.3.1"
MUJOCO_ZIP_URL = f"https://github.com/deepmind/mujoco/releases/download/{MUJOCO_VERSION}/mujoco-{MUJOCO_VERSION}-windows-x86_64.zip"
MUJOCO_DEST = Path.home() / ".mujoco" / "mujoco330" #f"mujoco{MUJOCO_VERSION.replace('.', '')}"
MUJOCO_REPO = "https://github.com/deepmind/mujoco.git"
REPO_DIR = Path("mujoco")
PYTHON_BINDINGS = REPO_DIR / "python"
STUBS_DIR = Path("stubs")

# ------------------------- UTILITIES -------------------------

def run(cmd, check=True, cwd=None):
    print(f"\n> Running: {' '.join(cmd)}")
    subprocess.run(cmd, check=check, cwd=cwd, shell=True)

def download_and_extract_zip(url, extract_to):
    zip_path = Path("mujoco.zip")
    print(f"📥 Downloading MuJoCo {MUJOCO_VERSION}...")
    urllib.request.urlretrieve(url, zip_path)
    print("📦 Extracting MuJoCo...")
    with zipfile.ZipFile(zip_path, 'r') as zip_ref:
        zip_ref.extractall(extract_to.parent)
    zip_path.unlink()
    print(f"✔️ Extracted to: {extract_to}")

def install_tools():
    # Install CMake and MSVC if not present
    if not shutil.which("cmake"):
        print("📦 Installing CMake via winget...")
        run(["winget", "install", "--id=Kitware.CMake", "--silent", "--accept-package-agreements", "--accept-source-agreements"])
    else:
        print("✔️ CMake already installed.")

    if not shutil.which("cl"):
        print("📦 Installing Visual C++ Build Tools via winget...")
        run(["winget", "install", "--id=Microsoft.VisualStudio.2022.BuildTools", "--silent", "--accept-package-agreements", "--accept-source-agreements"])
    else:
        print("✔️ MSVC already installed.")

    if not shutil.which("stubgen"):
        print("📦 Installing mypy for stubgen...")
        run([sys.executable, "-m", "pip", "install", "mypy"])

def check_python_version():
    major, minor = sys.version_info[:2]
    if major != 3 or not (10 <= minor <= 13):
        print(f"❌ Unsupported Python version: {major}.{minor}")
        print("👉 Please install Python 3.10 to 3.12 from https://www.python.org/downloads/")
        sys.exit(1)

# ------------------------- MAIN SETUP -------------------------

def setup_env_vars():
    os.environ["MUJOCO_PATH"] = str(MUJOCO_DEST)
    plugin_path = MUJOCO_DEST / "plugins"
    plugin_path.mkdir(exist_ok=True)
    os.environ["MUJOCO_PLUGIN_PATH"] = str(plugin_path)
    print(f"✔️ Set MUJOCO_PATH: {MUJOCO_DEST}")
    print(f"✔️ Set MUJOCO_PLUGIN_PATH: {plugin_path}")

def clone_mujoco():
    if not REPO_DIR.exists():
        print("📥 Cloning MuJoCo repository...")
        run(["git", "clone", MUJOCO_REPO])
    else:
        print("✔️ MuJoCo repo already cloned.")

def install_python_bindings():
    print("📦 Installing MuJoCo Python bindings in editable mode...")
    run(["pip", "install", "-e", "."], cwd=PYTHON_BINDINGS)

def generate_stubs():
    print("🧠 Generating .pyi stub files...")
    STUBS_DIR.mkdir(exist_ok=True)
    run(["stubgen", "-m", "mujoco", "-o", str(STUBS_DIR)])

def print_finish():
    print("\n🎉 Setup complete!")
    print(f"✔️ MuJoCo engine in: {MUJOCO_DEST}")
    print(f"✔️ Python bindings installed from: {PYTHON_BINDINGS}")
    print(f"✔️ Type stubs saved to: {STUBS_DIR}/mujoco.pyi")
    print("👉 Add this to your mypy.ini:")
    print(f"[mypy]\nmypy_path = {STUBS_DIR}")

# ------------------------- EXECUTION -------------------------

if __name__ == "__main__":
    check_python_version()
    install_tools()

    if not MUJOCO_DEST.exists():
        download_and_extract_zip(MUJOCO_ZIP_URL, MUJOCO_DEST)
    else:
        print(f"✔️ MuJoCo already extracted at: {MUJOCO_DEST}")

    setup_env_vars()
    clone_mujoco()
    install_python_bindings()
    generate_stubs()
    print_finish()
