"""Reusable world and scene XML for building models."""

from pathlib import Path

WORLD_ASSETS = Path(__file__).parent.joinpath("templates", "world.xml").read_text()
"""Pre-made world assets for MuJoCo simulation:
`skybox`, `grid`, `body`, and `background` textures."""


def glovebox(
    *,
    width: float = 1.25,
    depth: float = 0.75,
    height: float = 1.0,
    glass_thickness: float = 0.05,
    pos_x: float = 0,
    pos_y: float = 0,
) -> str:
    """Create a glovebox with the given dimensions (in meters)."""
    w, d, h, t = width / 2, depth / 2, height / 2, glass_thickness / 2
    return f"""<mujoco>
    <asset>
        <material name="glass" rgba="1 1 1 0.2"/>
    </asset>
    <worldbody>
        <light name="glovebox_light" pos="0 0 {height + 5 * glass_thickness}" dir="0 0 -1"
               diffuse="5 5 5" specular="2 2 2" directional="true" cutoff="180"/>
        <body name="camera_target" pos="0 0 0">
            <geom type="sphere" size="0.001" rgba="0 0 0 0" density="0"/>
        </body>
        <body name="glovebox" pos="{pos_x} {pos_y} 0">
            <geom type="box" size="{w} {t} {h}" material="glass" pos="0 {d + t} {h}"/>
            <geom type="box" size="{w} {t} {h}" material="glass" pos="0 {-d - t} {h}"/>
            <geom type="box" size="{t} {d} {h}" material="glass" pos="{w - t} 0 {h}"/>
            <geom type="box" size="{t} {d} {h}" material="glass" pos="{-w + t} 0 {h}"/>
            <geom type="box" size="{w} {d + glass_thickness} {t}" material="glass" pos="0 0 {height + t}"/>
            <site name="center_floor" pos="0 0 0" size="0.01" rgba="1 0 0 1"/>
        </body>
        <camera name="glovebox_cam" pos="{width} {depth} {height - glass_thickness}"
                mode="track" target="camera_target" fovy="90"/>
    </worldbody>
</mujoco>
"""
