from mujoco_toolbox import Wrapper, Builder, glovebox, WORLD_ASSETS
import mujoco

IN_TO_M = 0.0254

# Glovebox dimensions
width = 75 # in
depth = 60 # in
height = 40 # in

build = Builder("tests/models/UR5e/ur5e.xml", WORLD_ASSETS, glovebox(width=width*IN_TO_M, height=height*IN_TO_M, depth=depth*IN_TO_M, pos_y=0.4))

with Wrapper(build) as env:
    print(env.model.qpos0)
    mujoco.mj_resetDataKeyframe(env._model, env._data, 0)

    # print(env.body_names)
    env.launch(show_menu=False)