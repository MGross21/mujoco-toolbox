"""Merge MJCF and URDF sources into one MuJoCo model."""

from __future__ import annotations

import sys
from pathlib import Path
from xml.etree.ElementTree import Element, ElementTree, SubElement, indent, tostring

if sys.version_info >= (3, 11):
    from typing import Self
else:
    from typing_extensions import Self

from typing import TYPE_CHECKING

import defusedxml.ElementTree as ET

if TYPE_CHECKING:
    from collections.abc import Sequence

_MERGED_TAGS = (
    "asset",
    "worldbody",
    "camera",
    "light",
    "contact",
    "equality",
    "sensor",
    "actuator",
    "default",
    "tendon",
    "include",
)


class Builder:
    """Combine MJCF and URDF sources into a single MuJoCo model.

    Sources can be file paths or XML strings, either full documents
    (``<mujoco>``/``<robot>``) or bare sub-trees. Builders combine with ``+``,
    appending the second model's assets, bodies, actuators, sensors, etc.

    Example:
        >>> model = Builder("robot.urdf", meshdir="meshes/") + "<worldbody>...</worldbody>"
        >>> model.save("combined.xml")

    Args:
        *inputs: XML file paths or strings, merged in order.
        meshdir: Mesh directory written to ``<compiler meshdir>`` for URDF inputs.

    Raises:
        ValueError: If no input is given.
        TypeError: If an input is not a string.

    """

    def __init__(self, *inputs: str, meshdir: str = "meshes/") -> None:
        if not inputs:
            msg = "Input is required to initialize the Builder"
            raise ValueError(msg)
        if not all(isinstance(i, str) for i in inputs):
            msg = "Input must be an XML string or a file path"
            raise TypeError(msg)
        self.meshdir = meshdir
        self.root, self._mujoco = self._parse_input(inputs[0])
        for other in inputs[1:]:
            self += Builder(other, meshdir=meshdir)

    @staticmethod
    def merge(inputs: Sequence[str | Builder], meshdir: str = "meshes/") -> Builder:
        """Merge Builders and XML strings or file paths into one Builder.

        Builders are merged first, in order, followed by all strings.

        Args:
            inputs: Builders and/or XML strings or file paths.
            meshdir: Mesh directory for URDF inputs.

        Returns:
            The first Builder, extended with the rest.

        Raises:
            ValueError: If no inputs are provided.

        """
        if not inputs:
            msg = "No inputs provided for merging."
            raise ValueError(msg)
        builders = [i for i in inputs if isinstance(i, Builder)]
        strings = [i for i in inputs if isinstance(i, str)]
        if strings:
            builders.append(Builder(*strings, meshdir=meshdir))
        return sum(builders[1:], builders[0])

    def _parse_input(self, xml_input: str) -> tuple[Element, Element]:
        is_xml = xml_input.strip().startswith("<")
        if not is_xml and not Path(xml_input).exists():
            msg = f"File not found: {xml_input}"
            raise FileNotFoundError(msg)
        root = ET.fromstring(xml_input if is_xml else Path(xml_input).read_bytes())

        if root.tag not in ("robot", "mujoco"):
            wrapper = Element("mujoco")
            wrapper.append(root)
            root = wrapper
        mujoco_tag = root.find("mujoco") if root.tag == "robot" else root
        if mujoco_tag is None:
            mujoco_tag = Element("mujoco")
            root.insert(0, mujoco_tag)
        if mujoco_tag.find("compiler") is None:
            compiler = Element(
                "compiler", angle="radian", meshdir=self.meshdir, balanceinertia="true", discardvisual="true"
            )
            mujoco_tag.insert(0, compiler)
        return root, mujoco_tag

    def __add__(self, other: str | Builder) -> Self:
        if isinstance(other, str):
            other = Builder(other, meshdir=self.meshdir)
        elif not isinstance(other, Builder):
            msg = "Can only merge with str or Builder"
            raise TypeError(msg)

        for tag in _MERGED_TAGS:
            src = other._mujoco.find(tag)
            if src is None:
                continue
            dst = self._mujoco.find(tag)
            if dst is None:
                dst = SubElement(self._mujoco, tag)
            dst.extend(list(src))
        return self

    def save(self, file_path: str) -> str:
        """Write the model as indented XML.

        Args:
            file_path: Output path.

        Returns:
            Absolute path of the written file.

        """
        indent(self.root)
        ElementTree(self.root).write(file_path, encoding="utf-8", xml_declaration=True)
        return str(Path(file_path).resolve())

    def __str__(self) -> str:
        return tostring(self.root, encoding="unicode")

    __repr__ = __str__

    def __len__(self) -> int:
        return len(self.root)

    def __radd__(self, other: str | Builder) -> Self:
        return self.__add__(other)
