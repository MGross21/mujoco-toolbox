import subprocess
import platform
import os

def check_installed(tool: str, auto_install: bool = False):
    """Ensure a CLI tool is installed, optionally installing it if missing."""
    def install_tool(tool: str):
        system = platform.system()
        print(f"Installing {tool}...")
        try:
            if system == "Windows":
                subprocess.run(["winget", "install", "--id", "Gyan.FFmpeg", "--source", "winget"], check=True)
            elif system == "Linux":
                subprocess.run(["sudo", "apt", "update"], check=True)
                subprocess.run(["sudo", "apt", "install", "-y", tool], check=True)
            elif system == "Darwin":  # macOS
                subprocess.run(["brew", "install", tool], check=True)
            else:
                raise EnvironmentError(f"Unsupported platform: {system}")
        except Exception as e:
            raise RuntimeError(f"{tool} installation failed: {e}")

    try:
        subprocess.run([tool, "-version"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
    except (FileNotFoundError, subprocess.CalledProcessError, OSError):
        print(f"{tool} is not installed or not found in PATH.")
        if auto_install or os.getenv("MJTB_AUTO_INSTALL") == "1":
            install_tool(tool)
        else:
            user = input(f"Would you like to install {tool}? [y/n]: ").strip().lower()
            if user == "y":
                install_tool(tool)
            else:
                raise EnvironmentError(
                    f"{tool} not found. Refer to: https://github.com/MGross21/mujoco-toolbox#extra-packages\n"
                )