import os
import xml.etree.ElementTree as StdET
from collections.abc import Sequence
from io import BytesIO
from pathlib import Path
from typing import Union
from xml.etree.ElementTree import Element, ElementTree

import defusedxml.ElementTree as ET
import warnings

class Builder:
    """A class to build and manipulate MuJoCo XML models."""

    def __init__(self, *inputs: str) -> None:
        """
        Initialize a Builder from one or more XML strings or file paths.

        Args:
            *inputs: One or more XML strings or file paths.
        Raises:
            ValueError: If no inputs are provided.
            TypeError: If any input is not a string.
        """
        if not inputs:
            raise ValueError("Input is required to initialize the Builder")
        if not all(isinstance(i, str) for i in inputs):
            raise TypeError("Input must be an XML string or a file path")
        self.tree, self.root = self._parse_input(inputs[0])
        for other in inputs[1:]:
            self += Builder(other)

    @classmethod
    def merge(cls, items):
        """
        Merge multiple XML strings or Builder instances into a single Builder.
        Ensure `<compiler>` tags are merged correctly.
        """
        import xml.etree.ElementTree as ET  # Use standard library for XML handling
        roots = []
        for item in items:
            if isinstance(item, str):
                roots.append(Builder(item).root)
            elif isinstance(item, Builder):
                roots.append(item.root)
            else:
                raise TypeError(
                    "All items to merge must be XML strings or Builder instances"
                )
        merged_tree = ET.Element("robot")  # Use a generic root element instead of <mujoco>
        compiler_tags = []
        for root in roots:
            for child in root:
                if child.tag == "compiler":
                    compiler_tags.append(child)
                else:
                    merged_tree.append(child)
        # Warn if multiple <compiler> tags are found
        if len(compiler_tags) > 1:
            warnings.warn("Multiple <compiler> tags found. Only the first will be retained.")
        # Merge compiler tags if any
        if compiler_tags:
            merged_tree.append(compiler_tags[0])
        return cls(ET.tostring(merged_tree, encoding="unicode"))

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
            return self._to_safe_tree(root), root

        # If root is <mujoco>, return as-is
        if root.tag == "mujoco":
            return self._to_safe_tree(root), root

        # If root is neither <robot> nor <mujoco>, wrap in <robot>
        robot_tag = StdET.Element("robot")
        robot_tag.append(root)
        return self._to_safe_tree(robot_tag), robot_tag

    def _merge_root(self, other_root: Element) -> None:
        # Merge <mujoco> or <robot> roots, or fallback
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
            self._merge_tag("asset", self.root, other_root)
            self._merge_tag("worldbody", self.root, other_root)

    def _to_safe_tree(self, root: Element) -> ElementTree:
        xml_string = StdET.tostring(root)
        return ET.parse(BytesIO(xml_string))

    def __add__(self, other: Union[str, "Builder"]) -> "Builder":
        """
        Add (merge) another Builder or XML string into this Builder.
        """
        if isinstance(other, str):
            _, other_root = Builder(other)._parse_input(other)
        elif isinstance(other, Builder):
            other_root = other.root
        else:
            raise TypeError("Can only merge with str or Builder")
        self._merge_root(other_root)
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
        # Only count user-supplied top-level tags, not injected <compiler>
        if self.root is None:
            return 0
        return len([el for el in self.root if el.tag != "compiler"])

    def __radd__(self, other: Union[str, "Builder"]) -> "Builder":
        """
        Right-add (merge) another Builder or XML string into this Builder.
        """
        return self.__add__(other)
