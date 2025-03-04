![Build](https://github.com/MGross21/mujoco-toolbox/actions/workflows/ci.yml/badge.svg)
![Python](https://img.shields.io/badge/python-3.10%20|%203.11%20|%203.12%20|%203.13-blue)
![License](https://img.shields.io/github/license/MGross21/mujoco-toolbox)
[![PyPI](https://github.com/MGross21/mujoco-toolbox/actions/workflows/publish.yml/badge.svg)](https://github.com/MGross21/mujoco-toolbox/actions/workflows/publish.yml)

# Mujoco Toolbox

Streamlines the MuJoCo Physics Simulator

## PyPI Package

```bash
pip install -U mujoco-toolbox
```

## For Local Install

*Navigate to the Directory that you wish to place the package*

<details>
<summary><b>JUPYTER NOTEBOOK</b></summary>
</br>
*Place Commands Inside Cell*

One-Time Install:

```bash
!git clone https://github.com/MGross21/mujoco-toolbox
!pip install -U ./mujoco-toolbox
```

Check for Updates:

```bash
!git pull origin main
```

Install/Check for Updates:

```python
folder = "mujoco-toolbox"
repo_url = f"https://github.com/MGross21/{folder}"

!if [ -d "{folder}" ]; then cd {folder} && git pull && cd ..; else git clone {repo_url}; fi
!pip install -U ./{folder}
```

</details>
</br>

<details>
<summary><b>BASH WINDOW</b></summary>
</br>
One-Time Install:

```bash
git clone https://github.com/MGross21/mujoco-toolbox
pip install -U ./mujoco-toolbox
```

Check for Updates:
```bash
git pull origin main
```

Install/Check for Updates:

```bash
# Define the repo directory and remote URL
REPO_DIR="mujoco-toolbox"
REPO_URL="https://github.com/MGross21/mujoco-toolbox"
BRANCH="main"  # Change this to 'master' if needed

# Check if the repository exists and is up-to-date
if [ -d "$REPO_DIR" ]; then
  echo "Repository exists. Pulling latest changes..."
  cd "$REPO_DIR"
  git fetch origin  # Get latest changes without checking out the branch
  LOCAL_HASH=$(git rev-parse HEAD)
  REMOTE_HASH=$(git rev-parse origin/$BRANCH)

  if [ "$LOCAL_HASH" != "$REMOTE_HASH" ]; then
    echo "Local repository is outdated. Updating..."
    git pull origin $BRANCH
  else
    echo "Local repository is up to date."
  fi
else
  echo "Repository not found. Cloning..."
  git clone $REPO_URL
  cd "$REPO_DIR"
fi

# Install or update the package
pip install -U ./mujoco-toolbox
```

</details>

## Extra Packages

<details>
<summary><b>FFMPEG</b></summary>

</br>

*Required for [mediapy](https://google.github.io/mediapy/mediapy.html) dependency*

**Windows**

```bash
winget install ffmpeg
ffmpeg -version
```

**Linux**

```bash
sudo apt update && sudo apt install ffmpeg
ffmpeg -version
```

**MacOS**

*Using Homebrew*

```bash
brew install ffmpeg
ffmpeg -version
```

*Using MacPorts*

```bash
sudo port install ffmpeg
ffmpeg -version
```

</details>

## To Run in Scripts

```python
import mujoco_toolbox as mjtb
from mujoco_toolbox import Wrapper

# Optional Global Variables
mjtb.VERBOSITY = True # Additional Performance and Debugging Print Statements (Default=False)
mjtb.CAPTURE_PARAMETERS = "all" # MjData Simulation Parameters (Default = ['time', 'qpos', 'qvel', 'act', 'qacc', 'xpos', 'xquat', 'xmat', 'ctrl', 'sensordata']) 

# Example Code
Wrapper("path/to/your/xml").runSim(render=True).renderMedia()
```

## Pre-Made Controllers

```python
import mujoco_toolbox as mjtb

mjtb.sineController(m,d,**kwargs)
mjtb.cosineController(m,d,**kwargs)
mjtb.stepController(m,d,**kwargs)
mjtb.randomController(m,d,**kwargs)

# Wrapper can use custom controllers as well!
```
