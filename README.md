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

# Shorthand Bypass
# Will use run(render=True) and warning will be displayed 
mjtb.Wrapper("path/to/your/xml").save()
```

## Controllers

### Pre-Made

```python
from mujoco_toolbox.controllers import (
    cos,
    random,
    real_time,
    sin,
    step,
)
```

### Custom

```python

def foo(model: MjModel, data: MjData,**kwargs):
    # Perform logic based on model/data objects
    # ie. PID Controller
```

## Instantiating a Digital Twin

```python
import mujoco_toolbox as mjtb
from mujoco_toolbox.controllers import real_time

with mjtb.Wrapper("path/to/xml", controller=real_time) as digitaltwin:
    digitaltwin.liveView(show_menu=False) # Open the simulation window
    while True:
        digitaltwin.controller(digitaltwin.model, digitaltwin.data, {"Mujoco MjData Object": value})
```

See `MjData` objects [here](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjdata)

## File Support

### XML / MJCF (Native)

```python
import mujoco_toolbox as mjtb

mjtb.Wrapper("path/to/xml")
```

![Glovebox](https://github.com/MGross21/mujoco-toolbox/blob/main/assets/images/glovebox_sample.png)

### URDF

```python
import mujoco_toolbox as mjtb

mjtb.Wrapper("path/to/urdf", meshdir="path/to/mesh/files")
```

![UR5](https://github.com/MGross21/mujoco-toolbox/blob/main/assets/images/ur5_render_no_gui.png)

## Merging Capabilities

```python
from mujoco_toolbox import Builder, Wrapper

Obj = Builder("path/to/xml_1") + Builder("path/to/xml_2") + ...
# OR
Obj = Builder("path/to/xml_1","path/to/xml_2", ... )

# Then
Wrapper(Obj.xml)

```

![Humanoid in Box](https://github.com/MGross21/mujoco-toolbox/blob/main/assets/images/human_in_box.png)
