![Build](https://github.com/MGross21/mujoco-toolbox/actions/workflows/ci.yml/badge.svg)
![Python](https://img.shields.io/badge/python-3.10%20|%203.11%20|%203.12%20|%203.13-blue)
![License](https://img.shields.io/github/license/MGross21/mujoco-toolbox)
[![PyPI](https://github.com/MGross21/mujoco-toolbox/actions/workflows/publish.yml/badge.svg)](https://github.com/MGross21/mujoco-toolbox/actions/workflows/publish.yml)

# Mujoco Toolbox

Streamlines the MuJoCo Physics Simulator

## For Local Install

Navigate to the Directory that you wish to place the package

```bash
git clone https://github.com/MGross21/mujoco-toolbox
pip install -e ./mujoco-toolbox/
```

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
import mujoco_tools as mjtb
from mujoco_tools import Wrapper

# Sample
Wrapper("path/to/your/xml").runSim(render=True).renderMedia()
```

## Pre-Made Controllers

* Sine
* Cosine
* Single Step
* Random
