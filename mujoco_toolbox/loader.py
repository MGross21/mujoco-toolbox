import logging
from pathlib import Path
from typing import TYPE_CHECKING, Any, Union

import defusedxml.ElementTree as ET
import mujoco

from .builder import Builder # type checking

logging.basicConfig(level=logging.DEBUG)

class Loader:
    """Handles loading of MuJoCo models from XML or URDF files."""

    def __init__(self, builder: Union[str, "Builder"], **kwargs: Any) -> None:
        self._builder = builder if isinstance(builder, Builder) else Builder(builder)
        self.xml = str(self._builder)

        try:
            self._model = mujoco.MjModel.from_xml_string(self.xml)
        except mujoco.Error as e:
            raise ValueError(f"Failed to load model due to Mujoco error: {e}") from e
        except Exception as e:
            raise ValueError(f"Failed to load model: {e}") from e

    @property
    def model(self) -> mujoco.MjModel:
        if not hasattr(self, '_model'):
            raise RuntimeError("Model has not been initialized properly.")
        return self._model

    def validate_meshes(self) -> None:
        root = ET.fromstring(self.xml)
        meshdir = root.find("compiler").get("meshdir", "meshes/")
        for asset in root.findall(".//mesh"):
            file_path = Path(meshdir) / asset.get("file", "")
            if not file_path.exists():
                raise FileNotFoundError(f"Mesh file not found: {file_path}")

    def __str__(self) -> str:
        return self.xml

    def __repr__(self) -> str:
        return f"Loader(Initialized={hasattr(self, '_model')})"

    def __len__(self) -> int:
        return len(ET.fromstring(self.xml))