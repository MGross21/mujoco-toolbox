WORLD_ASSETS = """
  <asset>
    <texture type="skybox" builtin="gradient" rgb1=".3 .5 .7" rgb2="0 0 0" width="32" height="512"/>
    <texture name="body" type="cube" builtin="flat" mark="cross" width="128" height="128" rgb1="0.8 0.6 0.4" rgb2="0.8 0.6 0.4" markrgb="1 1 1"/>
    <material name="body" texture="body" texuniform="true" rgba="0.8 0.6 .4 1"/>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1=".1 .2 .3" rgb2=".2 .3 .4"/>
    <material name="grid" texture="grid" texrepeat="1 1" texuniform="true" reflectance=".2"/>
    <texture name="background" type="2d" builtin="flat" width="256" height="256" rgb1="1 1 1"/>
    <material name="background" texture="background" texuniform="true" rgba="1 1 1 1"/>
  </asset>
"""

def GloveBox(width=1.25, depth=0.75, height=1.0, glass_thickness=0.05) -> str:
    return f"""
<mujoco>
    <asset>
        <material name="glass" rgba="1 1 1 0.2"/>
    </asset>
    <worldbody>
        <body name="walls" pos="0 0 0">
            <geom type="box" size="{width/2} {glass_thickness/2} {height/2}" material="glass" pos="0 {depth/2+glass_thickness/2} {height/2}"/>
            <geom type="box" size="{width/2} {glass_thickness/2} {height/2}" material="glass" pos="0 {-depth/2-glass_thickness/2} {height/2}"/>
            <geom type="box" size="{glass_thickness/2} {depth/2} {height/2}" material="glass" pos="{width/2-glass_thickness/2} 0 {height/2}"/>
            <geom type="box" size="{glass_thickness/2} {depth/2} {height/2}" material="glass" pos="{-width/2+glass_thickness/2} 0 {height/2}"/>
            <geom type="plane" size="{width/2} {depth/2+glass_thickness} {glass_thickness/2}" material="glass" pos="0 0 {height}"/>
        </body>
    </worldbody>
</mujoco>
"""
