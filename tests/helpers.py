from pathlib import Path
from typing import Self

import numpy as np

MODELS = Path(__file__).parent / "models"
BOX_AND_LEG = MODELS / "box_and_leg.xml"
DROPPED_BOX = """
<mujoco>
    <worldbody>
        <geom type="plane" size="1 1 .1"/>
        <body pos="0 0 .5"><freejoint/><geom type="box" size=".1 .1 .1"/></body>
    </worldbody>
</mujoco>
"""


class FakeRenderer:
    """Stands in for ``mujoco.Renderer``; each frame is filled with its index."""

    def __init__(self, _model: object, height: int, width: int, max_geom: int) -> None:
        self.shape = (height, width, 3)
        self.max_geom = max_geom
        self.count = 0

    def __enter__(self) -> Self:
        return self

    def __exit__(self, *_: object) -> None:
        pass

    def update_scene(self, _data: object, _camera: object) -> None:
        pass

    def render(self, *, out: np.ndarray) -> np.ndarray:
        out[...] = self.count % 256
        self.count += 1
        return out
