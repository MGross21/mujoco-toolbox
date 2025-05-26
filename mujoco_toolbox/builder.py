import os
import xml.etree.ElementTree as StdET
from collections.abc import Sequence
from io import BytesIO
from pathlib import Path
from typing import Union
from xml.etree.ElementTree import Element, ElementTree

import defusedxml.ElementTree as ET


class Builder:
    """A class to build and manipulate MuJoCo XML models."""

    def __init__(self, *inputs: str, meshdir: str = "meshes/") -> None:
        if not inputs:
            msg = "Input is required to initialize the Builder"
            raise ValueError(msg)
        if not all(isinstance(i, str) for i in inputs):
            msg = "Input must be an XML string or a file path"
            raise TypeError(msg)
        self.meshdir = meshdir
        self.tree, self.root = self._parse_input(inputs[0])
        for other in inputs[1:]:
            self += Builder(other, meshdir=meshdir)

    @staticmethod
    def merge(inputs: Sequence[Union[str, "Builder"]], meshdir: str = "meshes/") -> "Builder":
        """Merge multiple Builder objects and/or XML strings into one Builder.

        Args:
            inputs: Sequence of Builder objects and/or XML strings or file paths.
            meshdir: Mesh directory (default: "meshes/").

        Returns:
            Merged Builder instance.

        Raises:
            ValueError: If no inputs are provided.

        """
        if not inputs:
            msg = "No inputs provided for merging."
            raise ValueError(msg)
        builders = [i for i in inputs if isinstance(i, Builder)]
        strings = [i for i in inputs if isinstance(i, str)]
        if builders:
            builder = sum(builders[1:], builders[0])
            if strings:
                builder += Builder(*strings, meshdir=meshdir)
        else:
            builder = Builder(*strings, meshdir=meshdir)
        return builder

    def _parse_input(self, xml_input: str) -> tuple[ElementTree, Element]:
        # Parse XML from string or file
        if xml_input.strip().startswith("<"):
            root = ET.fromstring(xml_input)
        else:
            path = Path(xml_input)
            if not path.exists():
                msg = f"File not found: {xml_input}"
                raise FileNotFoundError(msg)
            root = ET.parse(path).getroot()

        # If root is <robot>, ensure <mujoco> child exists (not as wrapper)
        if root.tag == "robot":
            mujoco_tag = root.find("mujoco")
            if mujoco_tag is None:
                mujoco_tag = StdET.Element("mujoco")
                # Insert <mujoco> as first child (after comments, if any)
                insert_idx = 0
                for idx, child in enumerate(list(root)):
                    if not isinstance(child.tag, str) or child.tag.startswith("#"):
                        insert_idx = idx + 1
                    else:
                        break
                root.insert(insert_idx, mujoco_tag)
            # Ensure <compiler> exists under <mujoco>
            compiler_tag = mujoco_tag.find("compiler")
            if compiler_tag is None:
                compiler_tag = StdET.Element("compiler", {
                    "angle": "radian",
                    "meshdir": self.meshdir,
                    "balanceinertia": "true",
                    "discardvisual": "true",
                })
                mujoco_tag.insert(0, compiler_tag)
            return self._to_safe_tree(root), root

        # If root is <mujoco>, ensure <compiler> exists
        if root.tag == "mujoco":
            compiler_tag = root.find("compiler")
            if compiler_tag is None:
                compiler_tag = StdET.Element("compiler", {
                    "angle": "radian",
                    "meshdir": self.meshdir,
                    "balanceinertia": "true",
                    "discardvisual": "true",
                })
                root.insert(0, compiler_tag)
            return self._to_safe_tree(root), root

        # If root is neither <robot> nor <mujoco>, wrap in <mujoco> and inject <compiler>
        mujoco_tag = StdET.Element("mujoco")
        mujoco_tag.append(root)
        compiler_tag = mujoco_tag.find("compiler")
        if compiler_tag is None:
            compiler_tag = StdET.Element("compiler", {
                "angle": "radian",
                "meshdir": self.meshdir,
                "balanceinertia": "true",
                "discardvisual": "true",
            })
            mujoco_tag.insert(0, compiler_tag)
        return self._to_safe_tree(mujoco_tag), mujoco_tag

    def _to_safe_tree(self, root: Element) -> ElementTree:
        xml_string = StdET.tostring(root)
        return ET.parse(BytesIO(xml_string))

    def __add__(self, other: Union[str, "Builder"]) -> "Builder":
        if isinstance(other, str):
            _, other_root = Builder(other, meshdir=self.meshdir)._parse_input(other)
        elif isinstance(other, Builder):
            other_root = other.root
        else:
            msg = "Can only merge with str or Builder"
            raise TypeError(msg)

        # Determine merge context: MJCF or URDF
        if self.root.tag == "robot":
            mujoco_self = self.root.find("mujoco")
            mujoco_other = other_root.find("mujoco") if other_root.tag == "robot" else other_root if other_root.tag == "mujoco" else None
            if mujoco_self is not None and mujoco_other is not None:
                self._merge_mujoco_tags(mujoco_self, mujoco_other)
        elif self.root.tag == "mujoco":
            mujoco_self = self.root
            mujoco_other = other_root.find("mujoco") if other_root.tag == "robot" else other_root if other_root.tag == "mujoco" else None
            if mujoco_other is not None:
                self._merge_mujoco_tags(mujoco_self, mujoco_other)
        else:
            # Fallback: merge at root
            self._merge_tag("asset", self.root, other_root)
            self._merge_tag("worldbody", self.root, other_root)
        return self

    def _merge_mujoco_tags(self, mujoco_self: Element, mujoco_other: Element) -> None:
        # Merge all relevant tags under <mujoco>
        for tag in [
            "asset", "worldbody", "camera", "light", "contact", "equality",
            "sensor", "actuator", "default", "tendon", "include",
        ]:
            self._merge_tag(tag, mujoco_self, mujoco_other)

    def _merge_tag(self, tag: str, root1: Element, root2: Element) -> None:
        s1, s2 = root1.find(tag), root2.find(tag)
        if s1 is None and s2 is not None:
            s1 = StdET.SubElement(root1, tag)
        if s1 is not None and s2 is not None:
            for el in list(s2):
                s1.append(el)

    def save(self, file_path: str) -> str:
        if self.tree is not None:
            self._indent_xml(self.root)
            self.tree.write(file_path, encoding="utf-8", xml_declaration=True)
        else:
            msg = "No model loaded. Cannot save."
            raise ValueError(msg)
        return os.path.abspath(file_path)

    def _indent_xml(self, elem: Element, level: int = 0) -> None:
        i = "\n" + level * "  "
        if len(elem):
            if not elem.text or not elem.text.strip():
                elem.text = i + "  "
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
            for sub_elem in elem:
                self._indent_xml(sub_elem, level + 1)
            if not elem[-1].tail or not elem[-1].tail.strip():
                elem[-1].tail = i
        elif level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i

    def __str__(self) -> str:
        return StdET.tostring(self.root, encoding="unicode", method="xml")

    def __repr__(self) -> str:
        return self.__str__()

    def __len__(self) -> int:
        return len(self.root) if self.root is not None else 0

    def __radd__(self, other: Union[str, "Builder"]) -> "Builder":
        return self.__add__(other)
