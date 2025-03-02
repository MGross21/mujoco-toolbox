[![Python 3.10](https://github.com/MGross21/mujoco-toolbox/actions/workflows/ci.yml/badge.svg?branch=main&event=push&matrix=python-version=3.10)](https://github.com/MGross21/mujoco-toolbox/actions/workflows/ci.yml)  [![Python 3.11](https://github.com/MGross21/mujoco-toolbox/actions/workflows/ci.yml/badge.svg?branch=main&event=push&matrix=python-version=3.11)](https://github.com/MGross21/mujoco-toolbox/actions/workflows/ci.yml)    [![Python 3.12](https://github.com/MGross21/mujoco-toolbox/actions/workflows/ci.yml/badge.svg?branch=main&event=push&matrix=python-version=3.12)](https://github.com/MGross21/mujoco-toolbox/actions/workflows/ci.yml)  [![Python 3.13](https://github.com/MGross21/mujoco-toolbox/actions/workflows/ci.yml/badge.svg?branch=main&event=push&matrix=python-version=3.13)](https://github.com/MGross21/mujoco-toolbox/actions/workflows/ci.yml)

# Streamlines the MuJoCo Physics Simulator

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
