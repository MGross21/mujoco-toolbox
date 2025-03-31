import os
import xml.etree.ElementTree as StdET  # For element creation
from io import BytesIO
from pathlib import Path
from typing import Union
from xml.etree.ElementTree import (  # Import for type annotations
    Element,
    ElementTree,
)

import defusedxml.ElementTree as ET  # For secure parsing


class Builder:
    """Builder class for loading, merging, and saving MuJoCo
    models from XML strings or file paths.

    Attributes:
        xml (str): XML string of the model.

    Examples:
        ```python
        # Load a model
        builder1 = Builder("path/to/model1.xml")
        # Merge two models from two object instances
        builder2 = Builder("path/to/model2.xml")
        merged_builder = builder1 + builder2
        merged_builder.save("path/to/merged_model.xml")
        # Merge two models from two file paths
        builder = Builder("path/to/model1.xml", "path/to/model2.xml")
        builder.save("path/to/merged_model.xml")
        ```

    """

    def __init__(self, *args: str) -> None:
        """Initialize with XML strings or file paths.

        Args:
            *args: XML strings or file paths.

        Raises:
            TypeError: If any input is not a string.
            ValueError: If no input is provided.

        """
        for arg in args:
            if not isinstance(arg, str):
                msg = "Input must be an XML string or a file path"
                raise TypeError(msg)

        if len(args) == 0:
            msg = "Input is required to initialize the Builder"
            raise ValueError(msg)

        if len(args) > 1:
            # Merge multiple Builder instances
            self.tree, self.root = self._load_model(args[0])
            for additional_arg in args[1:]:
                additional_builder = Builder(additional_arg)
                self.__add__(additional_builder)
        else:
            xml_input = args[0]
            self.tree, self.root = self._load_model(xml_input)

    def _create_element_tree(self, root: Element) -> ElementTree:
        """Create an ElementTree from a root element in a defusedxml-safe way.

        Args:
            root: Root element for the tree.

        Returns:
            ElementTree: A new ElementTree with the given root.

        """
        # Convert to string and parse back to create a tree
        xml_string = StdET.tostring(root)
        return ET.parse(BytesIO(xml_string))

    def _load_model(self, xml_input: str) -> tuple[ElementTree, Element]:
        """Load a MuJoCo model from a file or XML string.

        Args:
            xml_input: XML string or file path.

        Returns:
            tuple: ElementTree and root Element.

        Raises:
            TypeError: If input is not a string.
            ValueError: If XML is invalid.
            FileNotFoundError: If file path doesn't exist.

        """
        if not isinstance(xml_input, str):
            msg = "Input must be either an XML string or a file path"
            raise TypeError(msg) from None

        # Check if input is an XML string
        if xml_input.strip().startswith("<"):
            # Try parsing as XML string
            try:
                root = ET.fromstring(xml_input)
                # Create tree using our safe method
                tree = self._create_element_tree(root)

                # If it's a URDF (robot tag), wrap it in mujoco
                if root.tag == "robot":
                    return self._wrap_urdf_in_mujoco(tree)

                # If root is not mujoco, wrap it
                if root.tag != "mujoco":
                    mujoco_elem = StdET.Element("mujoco")
                    mujoco_elem.append(root)
                    # Create tree using our safe method
                    tree = self._create_element_tree(mujoco_elem)

                return tree, tree.getroot()
            except ET.ParseError:
                msg = "Invalid XML string provided"
                raise ValueError(msg) from None
        else:
            # Treat input as file path
            if not Path(xml_input).exists():
                msg = f"File not found: {xml_input}"
                raise FileNotFoundError(msg) from None

            # Handle URDF files
            if xml_input.lower().endswith(".urdf"):
                return self._wrap_urdf_in_mujoco(ET.parse(xml_input))

            # Parse regular XML file
            try:
                tree = ET.parse(xml_input)
                root = tree.getroot()

                # If root is not mujoco, wrap it
                if root.tag != "mujoco":
                    mujoco_elem = StdET.Element("mujoco")
                    mujoco_elem.append(root)
                    # Create tree using our safe method
                    tree = self._create_element_tree(mujoco_elem)

                return tree, tree.getroot()
            except ET.ParseError:
                msg = f"Invalid XML file: {xml_input}"
                raise ValueError(msg) from None

    def _wrap_urdf_in_mujoco(
        self, urdf_input: str | ElementTree,
    ) -> tuple[ElementTree, Element]:
        """Wrap a URDF inside <mujoco> tags for MuJoCo compatibility.

        Args:
            urdf_input: File path, ElementTree, or XML string.

        Returns:
            tuple: ElementTree and root Element.

        Raises:
            TypeError: If input is not a string or ElementTree.
            ValueError: If URDF is invalid.

        """
        # Handle string input (could be a file path or XML string)
        if isinstance(urdf_input, str):
            # Check if it's an XML string
            if urdf_input.strip().startswith("<"):
                try:
                    root = ET.fromstring(urdf_input)
                    # Create tree using our safe method
                    tree = self._create_element_tree(root)
                except ET.ParseError:
                    msg = "Invalid URDF XML string provided"
                    raise ValueError(msg) from None
            else:
                # It's a file path
                try:
                    tree = ET.parse(urdf_input)
                except Exception:
                    msg = f"Could not parse URDF file: {urdf_input}"
                    raise ValueError(msg) from None
        # Handle ElementTree input using duck typing
        elif hasattr(urdf_input, "getroot") and callable(urdf_input.getroot):
            # Handle both standard ElementTree and defusedxml ElementTree
            tree = urdf_input  # type: ignore
        else:
            msg = "URDF input must be a string or ElementTree-like object with getroot() method"
            raise TypeError(msg)

        root = tree.getroot()

        # If root is already present in a mujoco element
        if root.tag == "mujoco":
            return tree, root

        # Create mujoco element and append the URDF root
        mujoco_elem = StdET.Element("mujoco")
        mujoco_elem.append(root)

        # Create a new tree with the <mujoco> root using our safe method
        tree = self._create_element_tree(mujoco_elem)
        return tree, tree.getroot()

    def _merge_tags(self, tag_name: str, root_1: Element, root_2: Element) -> None:
        """Merge a specific tag (e.g., worldbody, asset) from two models.

        Args:
            tag_name: Tag name to merge.
            root_1: First model root element.
            root_2: Second model root element.

        """
        section_1 = root_1.find(tag_name)
        section_2 = root_2.find(tag_name)

        # If target section doesn't exist in model 1 but exists in model 2, create it
        if section_1 is None and section_2 is not None:
            section_1 = StdET.SubElement(root_1, tag_name)

        # Only proceed if both sections exist now
        if section_1 is not None and section_2 is not None:
            # Merge the contents by appending all elements from model 2 to model 1
            for element in list(section_2):  # Use list() to avoid modification
                section_1.append(element)

    def __add__(self, other: Union["Builder", str]) -> "Builder":
        """Implement the + operator for merging two models.

        Args:
            other: Another Builder object or XML string.

        Returns:
            Builder: Self with merged content.

        Raises:
            TypeError: If other is not a Builder or string.

        """
        model_root: Element | None = None

        if isinstance(other, Builder):
            # Merge the current model with another Builder instance
            model_root = other.root
        elif isinstance(other, str):
            # If the input is a string, load as XML
            _, model_root = self._load_model(other)
        else:
            msg = "Addition only supported between Builder objects or with XML strings/files"
            raise TypeError(msg)

        # List of sections to merge
        sections_to_merge = [
            "asset", "worldbody", "camera", "light", "contact", "equality",
            "sensor", "actuator", "default", "tendon", "include",
        ]

        # Merge all relevant sections
        for section in sections_to_merge:
            self._merge_tags(section, self.root, model_root)

        # Return self to allow for method chaining
        return self

    def __radd__(self, other: Union["Builder", int]) -> "Builder":
        """Implement the reverse + operator for merging two models.

        Args:
            other: Another Builder object or 0 (for sum()).

        Returns:
            Builder: Self with merged content.

        """
        if other == 0:  # Handle sum() starting value
            return self
        return self.__add__(other)

    def save(self, file_path: str) -> str:
        """Save the merged model to a file.

        Args:
            file_path: Path to save the model.

        Returns:
            str: Absolute path to the saved file.

        Raises:
            ValueError: If no model is loaded.

        """
        if self.tree is not None:
            # Format the XML with proper indentation before saving
            self._indent_xml(self.root)
            self.tree.write(file_path, encoding="utf-8", xml_declaration=True)
        else:
            msg = "No model loaded. Cannot save."
            raise ValueError(msg)
        return os.path.abspath(file_path)

    def _indent_xml(self, elem: Element, level: int = 0) -> None:
        """Add proper indentation to make the XML file more readable.

        Args:
            elem: XML element to indent.
            level: Indentation level.

        """
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
        """Return the XML string of the model.

        Returns:
            str: XML string representation.

        Raises:
            ValueError: If no model is loaded.

        """
        if self.tree is not None:
            # Format the XML with proper indentation
            self._indent_xml(self.root)
            # Use standard ElementTree's tostring with proper encoding
            return StdET.tostring(self.root, encoding="unicode", method="xml")
        msg = "No model loaded. Cannot generate string."
        raise ValueError(msg)

    def __repr__(self) -> str:
        """Return the XML string of the model.

        Returns:
            str: XML string representation.

        """
        return self.__str__()

    def __len__(self) -> int:
        """Return the number of elements in the model.

        Returns:
            int: Number of elements in the model.

        """
        return len(self.root) if self.root is not None else 0
