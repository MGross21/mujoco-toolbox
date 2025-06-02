from pathlib import Path
from typing import Any, Union

import defusedxml.ElementTree as ET
import mujoco

from .builder import Builder  # type checking


class Loader:
    """Handles loading of MuJoCo models from XML or URDF files."""

    def __init__(self, builder: Union[str, "Builder"], **kwargs: Any) -> None:
        self._builder = builder if isinstance(builder, Builder) else Builder(builder)
        self.xml = str(self._builder)

        try:
            self._model = mujoco.MjModel.from_xml_string(self.xml)
        except Exception:
            raise

    @property
    def model(self) -> mujoco.MjModel:
        if not hasattr(self, "_model"):
            msg = "Model has not been initialized properly."
            raise RuntimeError(msg)
        return self._model

    def validate_meshes(self) -> None:
        root = ET.fromstring(self.xml)
        compiler = root.find(".//compiler")
        meshdir = compiler.get("meshdir", "meshes/") if compiler is not None else "meshes/"
        for asset in root.findall(".//mesh"):
            file_path = Path(meshdir) / asset.get("file", "")
            if not file_path.exists():
                msg = f"Mesh file not found: {file_path}"
                raise FileNotFoundError(msg)

    def __str__(self) -> str:
        return self.xml

    def __repr__(self) -> str:
        return f"Loader(Initialized={hasattr(self, '_model')})"
