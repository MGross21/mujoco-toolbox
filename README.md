# Streamlines the Mujoco Physics Simulator

## For Local Install

Navigate to the Directory that you wish to place the package

```bash
git clone https://github.com/MGross21/mujoco-toolbox
pip install -e ./mujoco-toolbox/
```

## To Run in Scripts

```python
import mujoco_tools as mjtb
from mujoco_tools import Wrapper

# Sample
Wrapper("path/to/your/xml").runSim(render=True).renderMedia()
```

## Pre-Made Controllers

As of February 28th, 2025:

```python
mjtb.sineController
mjtb.sineController
mjtb.randomController
# Any other custom controller can be used as well
```
