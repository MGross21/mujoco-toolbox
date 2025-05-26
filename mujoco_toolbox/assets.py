from pathlib import Path

from jinja2 import Template

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
    return Template(
        Path(__file__).parent.joinpath("templates", "glovebox.xml").read_text(),
    ).render(
        width=width,
        depth=depth,
        height=height,
        glass_thickness=glass_thickness,
        pos_x=pos_x,
        pos_y=pos_y,
    )
