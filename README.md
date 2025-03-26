![Build](https://github.com/MGross21/mujoco-toolbox/actions/workflows/ci.yml/badge.svg)
![Python](https://img.shields.io/badge/python-3.10%20|%203.11%20|%203.12%20|%203.13-blue)
![License](https://img.shields.io/github/license/MGross21/mujoco-toolbox)
[![PyPI](https://github.com/MGross21/mujoco-toolbox/actions/workflows/publish.yml/badge.svg)](https://github.com/MGross21/mujoco-toolbox/actions/workflows/publish.yml)
[![Docs](https://github.com/MGross21/mujoco-toolbox/actions/workflows/docs.yml/badge.svg)](https://github.com/MGross21/mujoco-toolbox/actions/workflows/docs.yml)

# Mujoco Toolbox

Streamlines the MuJoCo Physics Simulator

## Installation

*Add `-U` flag to upgrade pre-existing library*

### PyPI Package

[![PyPI version](https://img.shields.io/pypi/v/mujoco-toolbox?labelColor=333333&color=%23800080)](https://pypi.org/project/mujoco-toolbox/)

```bash
pip install mujoco-toolbox
```

### GitHub Package

[![GitHub release](https://img.shields.io/github/v/release/MGross21/mujoco-toolbox?label=github&labelColor=333333&color=%23800080)](https://github.com/MGross21/mujoco-toolbox/releases)

```bash
pip install git+https://github.com/MGross21/mujoco-toolbox.git@main
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

## Example Script

*Bare minimum to run MuJoCo simulation and display result*

```python
import mujoco_toolbox as mjtb

mjtb.Wrapper("path/to/your/xml").run(render=True).save()
```

## Pre-Made Controllers

```python
from mujoco_toolbox.controller import (
    cosine_controller,
    random_controller,
    live_controller,
    sine_controller,
    step_controller,
)

# Wrapper can use custom controllers as well!
```

## File Support

### XML / MJCF (Native)

![Glovebox](https://github.com/MGross21/mujoco-toolbox/blob/main/assets/images/glovebox_sample.png)

### URDF

![UR5](https://github.com/MGross21/mujoco-toolbox/blob/main/assets/images/ur5_render_no_gui.png)
