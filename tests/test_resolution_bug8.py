
model = """
<mujoco>
    <visual>
        <global offwidth="800" offheight="600" />
    </visual>
    <option>
        <flag gravity="enable" contact="enable" />
    </option>
    <compiler angle="degree"/>
    <worldbody>
        <light name="top" pos="0 0 1"/>
        <body name="floor" pos="0 0 0">
            <geom name="floor" pos="0 -.5 0" size="1 1 .1" type="plane" rgba="1 0.83 0.61 0.5"/>
        </body>
        <body name="A" pos="0 0 0.5" axisangle="0 1 0 0">
            <joint name="j1" type="free" pos="0 0 0"/>
            <geom type="sphere" size=".105" pos="0 0 0" rgba="0 0 1 1" mass="0.25"/>
            <body name="leg" pos="0 0 -.25">
                <geom type="cylinder" size=".01 .25" pos="0 0 0" rgba="1 0 0 1" mass="0"/>
                <body name="foot" pos="0 0 -.25">
                    <joint name="ankle" type="ball" pos="0 0 0" stiffness="10" damping="1" />
                    <geom type="box" size="0.1 0.1 .01" pos="0.025 0 0" rgba="0 1 0 1" mass="0.1"/>
                    <!-- Add site for force application -->
                    <site name="force_site" pos="0 0 1"/>
                </body>
            </body>
        </body>
    </worldbody>
    <equality>
        <weld name="weld1" body1="floor" body2="foot" anchor="0 0 0"/>
    </equality>
</mujoco>

"""
# Fixing: https://github.com/MGross21/mujoco-toolbox/issues/8
# This test ensures that the resolution is set correctly when resolution
# is set in xml but not in wrapper
import numpy as np

from mujoco_toolbox import Wrapper

out = Wrapper(model)

assert out.resolution == (800, 600), "Resolution does not match expected value."
assert np.array_equal(out.gravity, np.array([0, 0, -9.81])), "Gravity does not match expected value."

out.run(render=True).save()
