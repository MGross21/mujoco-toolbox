import os
import xml.etree.ElementTree as StdET
from functools import lru_cache
from pathlib import Path
from typing import TYPE_CHECKING, Any, Union

import defusedxml.ElementTree as ET  # Secure parser
import mujoco
import trimesh

if TYPE_CHECKING:
    from .builder import Builder


class Loader:
    """Handles loading of MuJoCo models from XML or URDF files."""

    def __init__(self, xml: Union[str, "Builder"], meshdir: str = "meshes/") -> None:
        """Initialize the loader with XML content and mesh directory."""
        self.meshdir = meshdir
        self.xml = ""
        self._model = None

        if not isinstance(xml, str):
            xml = str(xml)

        self._load_model(xml)

    @property
    def model(self) -> mujoco.MjModel:
        """Get the loaded MuJoCo model."""
        if self._model is None:
            raise ValueError("No model loaded")
        return self._model

    def _load_model(self, xml: str, **kwargs: Any) -> None:
        """Load a MuJoCo model from a file or string."""
        path = Path(xml)
        if path.exists():
            extension = path.suffix.lower()[1:]
            if extension == "xml":
                self._load_xml_file(str(path.absolute()), **kwargs)
                return
            elif extension == "urdf":
                self._load_urdf_file(str(path.absolute()), **kwargs)
                return
            else:
                raise ValueError(f"Unsupported file extension: '{extension}'. Must be .xml or .urdf")

        try:
            root = ET.fromstring(xml)

            if root.tag == "robot":
                self._load_urdf_string(root, **kwargs)
            elif root.tag == "mujoco":
                self._load_xml_string(xml, **kwargs)
            else:
                raise ValueError(f"Unsupported root tag: <{root.tag}>. Must be <mujoco> or <robot>.")
        except ET.ParseError as e:
            raise ValueError(f"Invalid XML string: {e}") from e

    def _load_xml_file(self, xml_path: str, **kwargs) -> None:
        """Load a MuJoCo model from a valid MJCF XML file."""
        try:
            with open(xml_path, encoding="utf-8") as f:
                xml_content = f.read()

            if template := kwargs.get("template"):
                try:
                    xml_content = xml_content.format(**template)
                except KeyError as e:
                    raise ValueError(f"Template key error: {e}") from e

            self.xml = xml_content
            self._model = mujoco.MjModel.from_xml_string(xml_content)
        except FileNotFoundError:
            raise FileNotFoundError(f"XML file not found: {xml_path}")
        except (mujoco.FatalError, ValueError, TypeError) as e:
            raise ValueError(f"MuJoCo error loading XML: {e}") from e

    def _load_urdf_file(self, urdf_path: str, **kwargs: Any) -> None:
        """Load a URDF file and convert it to MJCF."""
        try:
            robot = ET.parse(urdf_path).getroot()
            self._load_urdf_string(robot, urdf_path=urdf_path, **kwargs)
        except ET.ParseError as e:
            raise ValueError(f"Error parsing URDF file: {e}") from e
        except Exception as e:
            raise ValueError(f"Failed to process URDF file: {e}") from e

    def _load_urdf_string(self, robot: StdET.Element, **kwargs: Any) -> None:
        """Convert a URDF ElementTree into a MJCF model."""
        try:
            mujoco_root = StdET.Element("mujoco")

            mesh_base_dir = Path(kwargs.get("urdf_path", self.meshdir))
            if not mesh_base_dir.is_absolute():
                mesh_base_dir = Path.cwd() / self.meshdir

            if not mesh_base_dir.exists():
                raise FileNotFoundError(f"Mesh directory not found: {mesh_base_dir}")

            compiler_attrs = {
                "meshdir": str(mesh_base_dir),
                "balanceinertia": kwargs.get("balanceinertia", "true"),
                "discardvisual": kwargs.get("discardvisual", "true"),
            }
            StdET.SubElement(mujoco_root, "compiler", **compiler_attrs)

            asset_tag = StdET.SubElement(mujoco_root, "asset")
            self._process_meshes(mesh_base_dir, asset_tag)

            self._generate_actuators_from_joints(robot, mujoco_root)

            worldbody = StdET.SubElement(mujoco_root, "worldbody")
            worldbody.append(robot)

            self.xml = StdET.tostring(mujoco_root, encoding="unicode").replace(".dae", ".stl")
            self._model = mujoco.MjModel.from_xml_string(self.xml)
        except Exception as e:
            raise ValueError(f"Failed to process URDF string: {e}") from e

    def _load_xml_string(self, xml: str, **kwargs: Any) -> None:
        """Load a MuJoCo model from a raw MJCF XML string."""
        try:
            if template := kwargs.get("template"):
                try:
                    xml = xml.format(**template)
                except KeyError as e:
                    raise ValueError(f"Template key error: {e}") from e

            self.xml = xml
            self._model = mujoco.MjModel.from_xml_string(xml)
        except (mujoco.FatalError, ValueError, TypeError) as e:
            raise ValueError(f"MuJoCo error loading XML string: {e}") from e

    def _process_meshes(self, mesh_base_dir: Path, asset_tag: StdET.Element) -> None:
        """Add STL and DAE meshes from a directory to the <asset> block."""
        processed_meshes = set()
        for stl_path in mesh_base_dir.glob("**/*.stl"):
            self._add_mesh_to_assets(mesh_base_dir, stl_path, asset_tag, processed_meshes)
        for dae_path in mesh_base_dir.glob("**/*.dae"):
            try:
                stl_path = Path(self._convert_dae_to_stl(str(dae_path)))
                self._add_mesh_to_assets(mesh_base_dir, stl_path, asset_tag, processed_meshes)
            except ValueError:
                continue

    def _add_mesh_to_assets(
        self,
        base_dir: Path,
        mesh_path: Path,
        asset_tag: StdET.Element,
        processed_meshes: set[str]
    ) -> None:
        """Insert mesh entry into MJCF asset block."""
        rel_path = mesh_path.relative_to(base_dir)
        rel_dir = str(rel_path.parent).replace(os.sep, "_")
        mesh_name = f"{mesh_path.stem}_{rel_dir}" if rel_dir != "." else mesh_path.stem

        if mesh_name in processed_meshes:
            return

        processed_meshes.add(mesh_name)
        StdET.SubElement(asset_tag, "mesh", name=mesh_name, file=str(rel_path))

    def _generate_actuators_from_joints(self, robot: StdET.Element, mujoco_tag: StdET.Element) -> None:
        """Create simple motors for each movable joint in the URDF."""
        actuator_tag = StdET.SubElement(mujoco_tag, "actuator")
        for joint in robot.findall(".//joint"):
            joint_type = joint.get("type", "").lower()
            joint_name = joint.get("name", "")
            if joint_type in ("revolute", "prismatic", "continuous") and joint_name:
                StdET.SubElement(
                    actuator_tag,
                    "motor",
                    name=f"motor_{joint_name}",
                    joint=joint_name,
                    gear="1",
                    ctrlrange="-1 1"
                )

    @staticmethod
    @lru_cache(maxsize=32)
    def _convert_dae_to_stl(dae_path: str) -> str:
        """Convert a DAE file to STL, only if needed."""
        dae_path = Path(dae_path)
        stl_path = dae_path.with_suffix(".stl")

        if not stl_path.exists() or dae_path.stat().st_mtime > stl_path.stat().st_mtime:
            try:
                mesh = trimesh.load_mesh(str(dae_path))
                mesh.export(str(stl_path))
            except Exception as e:
                raise ValueError(f"Error converting {dae_path.name} to STL: {e}") from e

        return str(stl_path)

    def reload(self, new_xml: str | None = None) -> mujoco.MjModel:
        """Reload model from stored or new XML string."""
        try:
            xml_to_use = new_xml if new_xml is not None else self.xml
            if not xml_to_use:
                raise ValueError("No XML available for reloading")

            self._model = mujoco.MjModel.from_xml_string(xml_to_use)
            if new_xml is not None:
                self.xml = new_xml

            return self._model
        except Exception as e:
            raise ValueError(f"Failed to reload model: {e}") from e