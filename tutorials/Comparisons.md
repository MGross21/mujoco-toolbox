# Comparisons

This document compares MuJoCo recommended way to perform certain actions vs this package, MuJoCo Toolbox.

## Launching Model in GUI

### MuJoCo

```python
import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("path/to/urdf")
data = mujoco.MjData(model)
viewer.launch(model, data)
```

### Mujoco Toolbox

```python
import mujoco_toolbox as mjtb

mjtb.Wrapper("path/to/urdf").launch()
```

## Running Sim and Capturing Data

Lets set some constants across both for simplicity:

```python
DURATION = 10
DATA_RATE = 100
```

### MuJoCo

```python
import mujoco

model = mujoco.MjModel.from_xml_path("path/to/urdf")
data = mujoco.MjData(model)
mujoco.mj_resetData(model, data)

t = []
xyz = []
qpos = []
qvel = []
act = []
qacc = []
xpos = []
xquat = []
xmat = []
ctrl = []
sensordata = []

while data.time < DURATION:
    mujoco.mj_step(model, data)

    if len(xyz) < data.time * DATA_RATE:
        t.append(data.time)
        xyz.append(data.xpos.copy())
        qpos.append(data.qpos.copy())
        qvel.append(data.qvel.copy())
        act.append(data.act.copy())
        qacc.append(data.qacc.copy())
        xpos.append(data.xpos.copy())
        xquat.append(data.xquat.copy())
        xmat.append(data.xmat.copy())
        ctrl.append(data.ctrl.copy())
        sensordata.append(data.sensordata.copy())
```

### Mujoco Toolbox

```python
import mujoco_toolbox as mjtb

sim = mjtb.Wrapper("path/to/urdf", DURATION, DATA_RATE).run()
sim.captured_data # Dictionary holding data, ie. ["qpos"] = [data]
```

Under the hood, certain class variable have the following defaults:

```python
mjtb.wrapper.CAPTURE_PARAMETERS = {"xpos","qpos","qvel","act","ctrl","qacc","xpos","xquat","xmat","ctrl","sensordata"}
```
