import mujoco_toolbox as mjtb
from mujoco_toolbox import Computer


xml = f"""
<mujoco>
    {mjtb.WORLD_ASSETS}
    <worldbody>
        <body name="floor" pos="0 0 0">
            <geom type="plane" size="5 5 0.1" material="grid"/>
        </body>
    </worldbody>
    <worldbody>
        <light name="light" pos="0 0 25" dir="0 0 -1" diffuse="1 1 1" specular="0.5 0.5 0.5" />
        <camera name="main" pos="0 -5 15" euler="0 0.7 0"/>
    </worldbody>
</mujoco>
"""


gb_width = 1.25
gb_depth = 0.75
gb_height = 1.0
glass_thickness = 0.05

glovebox = f"""
<mujoco>
    <asset>
        <material name="glass" rgba="1 1 1 0.2"/>
    </asset>
    <worldbody>
        <body name="walls" pos="0 0 0">
            <geom type="box" size="{gb_width/2} {glass_thickness/2} {gb_height/2}" material="glass" pos="0 {gb_depth/2+glass_thickness/2} {gb_height/2}"/>
            <geom type="box" size="{gb_width/2} {glass_thickness/2} {gb_height/2}" material="glass" pos="0 {-gb_depth/2-glass_thickness/2} {gb_height/2}"/>
            <geom type="box" size="{glass_thickness/2} {gb_depth/2} {gb_height/2}" material="glass" pos="{ gb_width/2-glass_thickness/2} 0 {gb_height/2}"/>
            <geom type="box" size="{glass_thickness/2} {gb_depth/2} {gb_height/2}" material="glass" pos="{-gb_width/2+glass_thickness/2} 0 {gb_height/2}"/>
            <geom type="plane" size="{gb_width/2} {gb_depth/2+glass_thickness} {glass_thickness/2}" material="glass" pos="0 0 {gb_height}"/>
        </body>
    </worldbody>
</mujoco>
"""

import os
model_dir = os.path.abspath(os.path.join("models", "UR5"))
urdf = os.path.join(model_dir, "ur5.urdf")
meshes = os.path.join(model_dir, "meshes", "collision")

w = mjtb.Wrapper(urdf, meshdir=meshes)
w.xml = (mjtb.Builder(xml) + mjtb.Builder(glovebox) + mjtb.Builder(w.xml)).xml

print(w.xml)

w.reload()
if Computer.GUI_ENABLED:
    w.liveView(show_menu=False)