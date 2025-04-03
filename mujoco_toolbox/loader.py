import os
from pathlib import Path
from functools import lru_cache
from typing import Any, Dict, List, Optional, Set, Tuple, Union
import mujoco
import trimesh

# Use defusedxml for secure XML parsing
import defusedxml.ElementTree as ET  # For secure parsing
import xml.etree.ElementTree as StdET  # For element creation and modification

from .builder import (Builder, 
    Element, ElementTree # for type hinting
)

class Loader:
    """Handles loading of MuJoCo models from XML or URDF files."""
    
    def __init__(self, xml: Union[str, 'Builder'], meshdir: str = "meshes/"):
        """Initialize the loader with XML content and mesh directory."""
        self.meshdir = meshdir
        self.xml = ""
        self._model = None
        
        # Handle Builder objects
        if hasattr(xml, '__str__') and not isinstance(xml, str):
            xml = str(xml)
            
        self._load_model(xml)
    
    @property
    def model(self) -> mujoco.MjModel:
        """Get the loaded MuJoCo model."""
        if self._model is None:
            raise ValueError("No model loaded")
        return self._model
    
    def _load_model(self, xml: str, **kwargs: Any) -> None:
        """Load a MuJoCo model from an XML file or a string."""
        try:
            # Check if it's a file path
            path = Path(xml)
            if path.exists():
                extension = path.suffix.lower()[1:]
                if extension == "xml":
                    self._load_xml_file(str(path.absolute()), **kwargs)
                elif extension == "urdf":
                    self._load_urdf_file(str(path.absolute()), **kwargs)
                else:
                    raise ValueError(f"Unsupported file extension: '{extension}'. Provide an XML or URDF file.")
                return
        except (TypeError, ValueError):
            pass  # Not a valid path, try as XML string
            
        # Try as XML string
        try:
            ET.fromstring(xml)  # Validate XML with secure parser
            self._load_xml_string(xml, **kwargs)
        except ET.ParseError as e:
            raise ValueError(f"Invalid XML string: {e}") from e
    
    def _load_xml_file(self, xml_path: str, **kwargs) -> None:
        """Load a MuJoCo model from an XML file."""
        try:
            with open(xml_path, 'r', encoding='utf-8') as f:
                xml_content = f.read()
            
            # Apply template if provided
            template = kwargs.get("template")
            if template:
                try:
                    xml_content = xml_content.format(**template)
                except KeyError as e:
                    raise ValueError(f"Template key error: {e}") from e
            
            self.xml = xml_content
            self._model = mujoco.MjModel.from_xml_string(xml_content)
        except FileNotFoundError as e:
            raise FileNotFoundError(f"XML file not found: {xml_path}") from e
        except mujoco.FatalError as e:
            raise ValueError(f"MuJoCo error loading XML: {e}") from e
    
    def _load_urdf_file(self, urdf_path: str, **kwargs: Any) -> None:
        """Process and load a URDF file for use with MuJoCo."""
        try:
            # Parse URDF with secure XML parser
            robot = ET.parse(urdf_path).getroot()
            
            # Create MuJoCo element (using StdET for creation)
            mujoco_tag = StdET.Element("mujoco")
            
            # Resolve mesh directory path
            mesh_base_dir = Path(self.meshdir)
            if not mesh_base_dir.is_absolute():
                urdf_dir = Path(urdf_path).parent.absolute()
                mesh_base_dir = urdf_dir / mesh_base_dir
            
            if not mesh_base_dir.exists():
                raise FileNotFoundError(f"Mesh directory not found: {mesh_base_dir}")
            
            # Add compiler tag
            compiler_attrs = {
                "meshdir": str(mesh_base_dir),
                "balanceinertia": kwargs.get("balanceinertia", "true"),
                "discardvisual": kwargs.get("discardvisual", "true"),
            }
            StdET.SubElement(mujoco_tag, "compiler", **compiler_attrs)
            
            # Create asset tag for meshes
            asset_tag = StdET.SubElement(mujoco_tag, "asset")
            
            # Process mesh directories
            self._process_meshes(mesh_base_dir, asset_tag)
            
            # Generate actuators from URDF joints
            self._generate_actuators_from_joints(robot, mujoco_tag)
            
            # Insert MuJoCo tag at beginning of robot tag
            robot.insert(0, mujoco_tag)
            
            # Convert to string and replace DAE with STL references
            self.xml = StdET.tostring(robot, encoding="unicode").replace(".dae", ".stl")
            
            # Load model
            self._model = mujoco.MjModel.from_xml_string(self.xml)
        except FileNotFoundError:
            raise
        except ET.ParseError as e:
            raise ValueError(f"Error parsing URDF file: {e}") from e
        except Exception as e:
            raise ValueError(f"Failed to process URDF file: {e}") from e
    
    def _process_meshes(self, mesh_base_dir: Path, asset_tag: StdET.Element) -> None:
        """Process all mesh files in the mesh directory."""
        processed_meshes = set()
        for stl_path in mesh_base_dir.glob("**/*.stl"):
            self._add_mesh_to_assets(mesh_base_dir, stl_path, asset_tag, processed_meshes)
        for dae_path in mesh_base_dir.glob("**/*.dae"):
            try:
                stl_path = Path(self._convert_dae_to_stl(str(dae_path)))
                self._add_mesh_to_assets(mesh_base_dir, stl_path, asset_tag, processed_meshes)
            except ValueError:
                continue
    
    def _add_mesh_to_assets(self, base_dir: Path, mesh_path: Path, asset_tag: StdET.Element, processed_meshes: Set[str]) -> None:
        """Add a mesh file to the asset tag."""
        rel_path = mesh_path.relative_to(base_dir)
        rel_dir = str(rel_path.parent).replace(os.sep, '_')
        mesh_name = f"{mesh_path.stem}_{rel_dir}" if rel_dir != '.' else mesh_path.stem
        if mesh_name in processed_meshes:
            return
        processed_meshes.add(mesh_name)
        StdET.SubElement(asset_tag, "mesh", name=mesh_name, file=str(rel_path))
    
    def _generate_actuators_from_joints(self, robot: StdET.Element, mujoco_tag: StdET.Element) -> None:
        """Generate actuators for the movable joints in the URDF."""
        actuator_tag = StdET.SubElement(mujoco_tag, "actuator")
        for joint in robot.findall(".//joint"):
            joint_type = joint.get("type", "").lower()
            joint_name = joint.get("name", "")
            if joint_type in ("revolute", "prismatic", "continuous") and joint_name:
                StdET.SubElement(actuator_tag, "motor", name=f"motor_{joint_name}", joint=joint_name, gear="1", ctrlrange="-1 1")
    
    @staticmethod
    @lru_cache(maxsize=32)  # Cache expensive mesh conversions
    def _convert_dae_to_stl(dae_path: str) -> str:
        """Convert DAE file to STL if it doesn't already exist."""
        dae_path = Path(dae_path)
        stl_path = dae_path.with_suffix('.stl')
        
        # Only convert if STL doesn't exist or is older than DAE
        if not stl_path.exists() or dae_path.stat().st_mtime > stl_path.stat().st_mtime:
            try:
                mesh = trimesh.load_mesh(str(dae_path))
                mesh.export(str(stl_path))
            except Exception as e:
                raise ValueError(f"Error converting {dae_path.name} to STL: {e}") from e
                
        return str(stl_path)
    
    def _load_xml_string(self, xml: str, **kwargs: Any) -> None:
        """Load a MuJoCo model from an XML string."""
        try:
            # Apply template if provided
            template = kwargs.get("template")
            if template:
                try:
                    xml = xml.format(**template)
                except KeyError as e:
                    raise ValueError(f"Template key error: {e}") from e
            
            self.xml = xml
            self._model = mujoco.MjModel.from_xml_string(xml)
        except mujoco.FatalError as e:
            raise ValueError(f"MuJoCo error loading XML string: {e}") from e
    
    def reload(self, new_xml: Optional[str] = None) -> mujoco.MjModel:
        """Reload the model from XML string."""
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
