<h1 align="center">
<img src="https://raw.githubusercontent.com/MGross21/mujoco-toolbox/main/assets/images/mjtb_logo_transparent.png" width="400">
</h1><br>

![Build](https://github.com/MGross21/mujoco-toolbox/actions/workflows/ci.yml/badge.svg)
![Python](https://img.shields.io/badge/python-3.9%20|%203.10%20|%203.11%20|%203.12%20|%203.13-blue)
![License](https://img.shields.io/github/license/MGross21/mujoco-toolbox)
[![PyPI](https://github.com/MGross21/mujoco-toolbox/actions/workflows/publish.yml/badge.svg)](https://github.com/MGross21/mujoco-toolbox/actions/workflows/publish.yml)
[![Docs](https://github.com/MGross21/mujoco-toolbox/actions/workflows/docs.yml/badge.svg)](https://github.com/MGross21/mujoco-toolbox/actions/workflows/docs.yml)

A Modern Simulation Wrapper for Google DeepMind’s MuJoCo

> **⚠️ WARNING**  
> This package is currently in its zero-release stage. Class methods and APIs may change without prior notice. Please review the documentation and changelog after each update to stay informed about any modifications.

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


### Adding to Project Dependencies
<details>

<summary><b>Click to Expand</b></summary><br>


Place the following in your `requirements.txt` or `pyproject.toml` file.

### PyPI

Expect less frequent, stable releases.

```
mujoco-toolbox
```

### Github

Expect frequent rolling releases.

```
git+https://github.com/MGross21/mujoco-toolbox.git@main#egg=mujoco-toolbox
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

*Debian/Ubuntu*

```bash
sudo apt update && sudo apt install ffmpeg
ffmpeg -version
```

*Arch Linux*

```bash
sudo pacman -Syu ffmpeg
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

mjtb.Simulation("path/to/your/xml").run(render=True).save()
```

## Controllers

### Pre-Made Controllers

The following controllers are available out-of-the-box:

- `sin`
- `cos`
- `step`
- `random`
- `real_time` <sub>(recommended controller for digital twins)</sub>

You can import them as follows:

```python
import mujoco_toolbox.controllers as ctrl
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

with mjtb.Simulation("path/to/xml", controller=real_time) as digitaltwin:
    digitaltwin.launch(show_menu=False) # Open the simulation window
    while True:
        digitaltwin.controller(digitaltwin.model, digitaltwin.data, {"mjdata_kwargs": value})
```

See `MjData` objects [here](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjdata)

## File Support

### XML / MJCF (Native)

```python
import mujoco_toolbox as mjtb

mjtb.Simulation("path/to/xml").show()
```

<img src="https://raw.githubusercontent.com/MGross21/mujoco-toolbox/main/assets/images/ur5_vention.png" alt="UR5/Vention" width="300">

### URDF

```python
import mujoco_toolbox as mjtb

mjtb.Simulation("path/to/urdf", meshdir="path/to/mesh/files").show()  # supports *.stl or *.obj
```

<img src="https://raw.githubusercontent.com/MGross21/mujoco-toolbox/main/assets/images/ur5_render_no_gui.png" alt="UR5" width="300">

## Merging Capabilities

Supports full `<mujoco>...</mujoco>` and `<robot>...</robot>` structure as well as complete sub-tree structures.

```python
import mujoco_toolbox as mjtb

# Merges: XML & URDF Files, XML & URDF Strings, Sub Tree Structures
mjtb.Simulation("path/to/xml_1", string_xml_var, ..., "path/to/xml_n").show()

```

> **⚠️ WARNING**  
> Duplicate sub-tree items with the same name will cause MuJoCo to throw a `FatalError`.

<img src="https://raw.githubusercontent.com/MGross21/mujoco-toolbox/main/assets/images/human_in_box.png" alt="Humanoid in Box" width="300">
