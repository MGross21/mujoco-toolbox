import mujoco_toolbox as mjtb
import os

# First Model (string1) - A simple model with an asset and a body (plane)
string1 = """
<mujoco>
  <asset>
    <texture name="skybox" builtin="gradient" rgb1=".3 .5 .7" rgb2="0 0 0" width="32" height="512"/>
    <material name="skybox_material" texture="skybox" texuniform="true" rgba="1 1 1 1"/>
  </asset>
  <worldbody>
    <body name="floor" pos="0 0 0">
      <geom type="plane" size="5 5 0.1" material="skybox_material"/>
    </body>
  </worldbody>
</mujoco>
"""

# Second Model (string2) - A model with a box and sphere with materials
string2 = """
<mujoco>
  <asset>
    <texture name="body_texture" type="cube" builtin="flat" width="128" height="128" rgb1="0.8 0.6 0.4" rgb2="0.8 0.6 0.4"/>
    <material name="body_material" texture="body_texture" texuniform="true" rgba="0.8 0.6 .4 1"/>
  </asset>
  <worldbody>
    <body name="box" pos="0 0.5 0">
      <geom type="box" size="0.5 0.5 0.5" material="body_material"/>
    </body>
    <body name="sphere" pos="2 2 0">
      <geom type="sphere" size="0.5" material="body_material"/>
    </body>
  </worldbody>
</mujoco>
"""

# Third Model (string3) - Adding light and camera
string3 = """
<mujoco>
  <worldbody>
    <light name="light1" pos="0 0 10" dir="0 0 -1" diffuse="1 1 1" specular="0.5 0.5 0.5"/>
    <camera name="main_camera" pos="5 -5 5" euler="0 0.7 0"/>
  </worldbody>
</mujoco>
"""

# Fourth Model (string4) - Simulation settings (timestep and gravity)
string4 = """
<mujoco>
  <option timestep="0.002" gravity="0 0 -9.81"/>
</mujoco>
"""


def main() -> None:
    builder = mjtb.Builder(string1)\
            + mjtb.Builder(string2)\
            + mjtb.Builder(string3)\
            + mjtb.Builder(string4)

    assert builder is not None, "Builder object is empty"

    args = [string1, string2, string3, string4]

    builder2 = mjtb.Builder(*args)

    assert builder2 is not None, "Builder2 object is empty"

    if mjtb.GUI_ENABLED:
        mjtb.Wrapper(builder.xml).liveView(show_menu=False)
        mjtb.Wrapper(builder2.xml).liveView(show_menu=False)

if __name__ == "__main__":
    main()
    mjtb.utils._print_success(f"{os.path.basename(__file__)} Tests passed!\n")
