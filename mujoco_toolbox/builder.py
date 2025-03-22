import os
import xml.etree.ElementTree as ET


class Builder:
    def __init__(self, xml_input: str):
        """ Initialize with an XML string or a file path. Input is required."""
        self.tree, self.root = self.load_model(xml_input)

    def load_model(self, xml_input):
        """ Load a MuJoCo model from a file or XML string."""
        if not isinstance(xml_input, str):
            raise ValueError("Input must be either an XML string or a file path")

        # Check if input is an XML string
        if xml_input.strip().startswith("<"):
            # Try parsing as XML string
            try:
                root = ET.fromstring(xml_input)
                tree = ET.ElementTree(root)

                # If it's a URDF (robot tag), wrap it in mujoco
                if root.tag == "robot":
                    return self.wrap_urdf_in_mujoco(tree)

                # If root is not mujoco, wrap it
                if root.tag != "mujoco":
                    mujoco_elem = ET.Element("mujoco")
                    mujoco_elem.append(root)
                    tree = ET.ElementTree(mujoco_elem)

                return tree, tree.getroot()
            except ET.ParseError:
                raise ValueError("Invalid XML string provided")
        else:
            # Treat input as file path
            if not os.path.exists(xml_input):
                raise FileNotFoundError(f"File not found: {xml_input}")

            # Handle URDF files
            if xml_input.lower().endswith(".urdf"):
                return self.wrap_urdf_in_mujoco(ET.parse(xml_input))

            # Parse regular XML file
            try:
                tree = ET.parse(xml_input)
                root = tree.getroot()

                # If root is not mujoco, wrap it
                if root.tag != "mujoco":
                    mujoco_elem = ET.Element("mujoco")
                    mujoco_elem.append(root)
                    tree = ET.ElementTree(mujoco_elem)

                return tree, tree.getroot()
            except ET.ParseError:
                raise ValueError(f"Invalid XML file: {xml_input}")

    def wrap_urdf_in_mujoco(self, urdf_input):
        """
        Wrap a URDF inside <mujoco> tags for MuJoCo compatibility.
        Input can be a file path, ElementTree, or XML string.
        """
        # Handle string input (could be a file path or XML string)
        if isinstance(urdf_input, str):
            # Check if it's an XML string
            if urdf_input.strip().startswith("<"):
                try:
                    root = ET.fromstring(urdf_input)
                    tree = ET.ElementTree(root)
                except ET.ParseError:
                    raise ValueError("Invalid URDF XML string provided")
            else:
                # It's a file path
                try:
                    tree = ET.parse(urdf_input)
                except:
                    raise ValueError(f"Could not parse URDF file: {urdf_input}")
        # Handle ElementTree input
        elif isinstance(urdf_input, ET.ElementTree):
            tree = urdf_input
        else:
            raise TypeError("URDF input must be a string or ElementTree")

        root = tree.getroot()

        # Handle the case where the root might already be a mujoco element
        if root.tag == "mujoco":
            return tree, root

        # Create mujoco element and append the URDF root
        mujoco_elem = ET.Element("mujoco")
        mujoco_elem.append(root)

        # Create a new tree with the <mujoco> root
        return ET.ElementTree(mujoco_elem), mujoco_elem

    def merge_tags(self, tag_name, root_1, root_2):
        """ Merge a specific tag (e.g., worldbody, asset) from two models."""
        section_1 = root_1.find(tag_name)
        section_2 = root_2.find(tag_name)

        # If target section doesn't exist in model 1 but exists in model 2, create it
        if section_1 is None and section_2 is not None:
            section_1 = ET.SubElement(root_1, tag_name)

        # Only proceed if both sections exist now
        if section_1 is not None and section_2 is not None:
            # Merge the contents by appending all elements from model 2 to model 1
            for element in list(section_2):  # Use list() to avoid modification during iteration
                section_1.append(element)

    def __add__(self, other):
        """ Implement the + operator for merging two models."""
        model_root = None

        if isinstance(other, Builder):
            # Merge the current model with another Builder instance
            model_root = other.root
        elif isinstance(other, str):
            # If the input is a string, load as XML
            _, model_root = self.load_model(other)
        else:
            raise TypeError("Addition only supported between Builder objects or with XML strings/files")

        # List of sections to merge
        sections_to_merge = ["asset", "worldbody", "camera", "light", "contact", "equality",
                            "sensor", "actuator", "default", "tendon", "include"]

        # Merge all relevant sections
        for section in sections_to_merge:
            self.merge_tags(section, self.root, model_root)

        # Return self to allow for method chaining
        return self

    def __radd__(self, other):
        """ Implement the reverse + operator for merging two models."""
        if other == 0:  # Handle sum() starting value
            return self
        return self.__add__(other)

    def save(self, file_path):
        """ Save the merged model to a file."""
        if self.tree is not None:
            # Format the XML with proper indentation before saving
            self._indent_xml(self.root)
            self.tree.write(file_path, encoding="utf-8", xml_declaration=True)
        else:
            raise ValueError("No model loaded. Cannot save.")

    def _indent_xml(self, elem, level=0):
        """ Add proper indentation to make the XML file more readable."""
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
        else:
            if level and (not elem.tail or not elem.tail.strip()):
                elem.tail = i

    def __str__(self):
        """ Return the XML string of the model."""
        if self.tree is not None:
            # Format the XML with proper indentation
            self._indent_xml(self.root)
            return ET.tostring(self.root, encoding="unicode", method="xml")
        else:
            raise ValueError("No model loaded. Cannot generate string.")

    def __repr__(self):
        """ Return the XML string of the model."""
        return self.__str__()

    @property
    def xml(self):
        """ Property to quickly grab the XML string of the model."""
        return str(self)
