from mujoco_toolbox import COMPUTER, WORLD_ASSETS, Builder, glovebox, Wrapper, print_success
import os

####################
# TESTING DESCRIPTION:
# Builder:
# - World
# - GloveBox
# - UR5
# Wrapper:
# - Live view
####################

world = f"""
<mujoco>
    {WORLD_ASSETS}
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
def main() -> None:
    model_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "models", "UR5"))
    urdf = os.path.join(model_dir, "ur5.urdf")
    meshes = os.path.join(model_dir, "meshes", "collision")

    humanoid = os.path.join(os.path.dirname(__file__), "models", "humanoid.xml")

    # w = mjtb.Wrapper(urdf, meshdir=meshes)
    # w.xml = (mjtb.Builder(mjtb.GloveBox()) + mjtb.Builder(w.xml) + mjtb.Builder(humanoid)).xml

    # w.reload()
    if COMPUTER.GUI_ENABLED:
        Wrapper(Builder(humanoid, glovebox(5, 5, 5)).xml).liveView(show_menu=False)

if __name__ == "__main__":
    main()
    print_success(f"{__file__} Tests passed!\n")
