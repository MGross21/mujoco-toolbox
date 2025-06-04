import pytest
from mujoco_toolbox.builder import Builder

# Test data
string1 = """
<mujoco>
  <asset>
    <texture name="skybox" builtin="gradient" rgb1=".3 .5 .7" rgb2="0 0 0" width="32" height="512"/>
    <material name="skybox_material" texture="skybox" texuniform="true" rgba="1 1 1 1"/>
  </asset>
  <worldbody>
    <body name="floor" pos="0 0 0">
      <geom type="plane" size="5 5 0.1" material="skybox_material"/>
    </body>
  </worldbody>
</mujoco>
"""

string2 = """
<mujoco>
  <asset>
    <texture name="body_texture" type="cube" builtin="flat" width="128" height="128" rgb1="0.8 0.6 0.4" rgb2="0.8 0.6 0.4"/>
    <material name="body_material" texture="body_texture" texuniform="true" rgba="0.8 0.6 .4 1"/>
  </asset>
  <worldbody>
    <body name="box" pos="0 0.5 0">
      <geom type="box" size="0.5 0.5 0.5" material="body_material"/>
    </body>
    <body name="sphere" pos="2 2 0">
      <geom type="sphere" size="0.5" material="body_material"/>
    </body>
  </worldbody>
</mujoco>
"""

string3 = """
<mujoco>
  <worldbody>
    <light name="light1" pos="0 0 10" dir="0 0 -1" diffuse="1 1 1" specular="0.5 0.5 0.5"/>
    <camera name="main_camera" pos="5 -5 5" euler="0 0.7 0"/>
  </worldbody>
</mujoco>
"""

string4 = """
<mujoco>
  <option timestep="0.002" gravity="0 0 -9.81"/>
</mujoco>
"""

@pytest.fixture
def builder():
    return Builder(string1) + Builder(string2) + Builder(string3) + Builder(string4)

@pytest.fixture
def builder_with_args():
    return Builder(string1, string2, string3, string4)

def test_builder_creation(builder) -> None:
    assert builder is not None, "Builder object is empty"
    assert len(builder) > 0, "Builder object has no elements"

def test_builder_with_args_creation(builder_with_args) -> None:
    assert builder_with_args is not None, "Builder2 object is empty"
    assert len(builder_with_args) > 0, "Builder2 object has no elements"

def test_builder_merge() -> None:
    builder1 = Builder(string1)
    builder2 = Builder(string2)
    merged_builder = builder1 + builder2
    assert merged_builder is not None, "Merged builder is empty"
    assert len(merged_builder) >= len(builder1), "Merged builder did not increase in size"

def test_builder_save(tmp_path) -> None:
    builder = Builder(string1)
    save_path = tmp_path / "test_model.xml"
    builder.save(str(save_path))
    assert save_path.exists(), "Saved file does not exist"

def test_builder_str_representation(builder) -> None:
    xml_str = str(builder)
    assert xml_str.startswith("<mujoco>"), "String representation does not start with <mujoco>"
    assert "</mujoco>" in xml_str, "String representation does not end with </mujoco>"

def test_builder_invalid_input() -> None:
    with pytest.raises(ValueError, match="Input is required to initialize the Builder"):
        Builder()

    with pytest.raises(TypeError, match="Input must be an XML string or a file path"):
        Builder(123)

def test_builder_merge_sections() -> None:
    builder1 = Builder(string1)
    builder2 = Builder(string2)
    merged_builder = builder1 + builder2
    assert merged_builder.root.find("asset") is not None, "Merged builder is missing 'asset' section"
    assert merged_builder.root.find("worldbody") is not None, "Merged builder is missing 'worldbody' section"

# Additional meshdir/compiler tests

def test_compiler_meshdir_not_set_when_not_provided():
    xml = """
    <mujoco>
      <compiler angle="radian"/>
      <worldbody/>
    </mujoco>
    """
    builder = Builder(xml)
    compiler = builder.root.find("compiler")
    assert compiler is not None
    assert "meshdir" not in compiler.attrib

def test_compiler_meshdir_set_when_provided():
    xml = """
    <mujoco>
      <worldbody/>
    </mujoco>
    """
    builder = Builder(xml, meshdir="my_meshes/")
    compiler = builder.root.find("compiler")
    assert compiler is not None
    assert compiler.attrib["meshdir"] == "my_meshes/"

def test_compiler_meshdir_not_overwritten():
    xml = """
    <mujoco>
      <compiler meshdir="existing_meshes/" angle="radian"/>
      <worldbody/>
    </mujoco>
    """
    builder = Builder(xml, meshdir="should_not_override/")
    compiler = builder.root.find("compiler")
    assert compiler is not None
    assert compiler.attrib["meshdir"] == "existing_meshes/"

def test_compiler_meshdir_merge_behavior():
    xml1 = """
    <mujoco>
      <compiler meshdir="foo/"/>
      <worldbody/>
    </mujoco>
    """
    xml2 = """
    <mujoco>
      <compiler meshdir="bar/"/>
      <worldbody/>
    </mujoco>
    """
    builder1 = Builder(xml1)
    builder2 = Builder(xml2)
    merged = builder1 + builder2
    compiler = merged.root.find("compiler")
    assert compiler is not None
    # The first builder's meshdir should be preserved
    assert compiler.attrib["meshdir"] == "foo/"

def test_compiler_existing_tag_not_overridden():
    xml = """
    <mujoco>
      <compiler meshdir="keep_this/" angle="degree" custom="yes"/>
      <worldbody/>
    </mujoco>
    """
    builder = Builder(xml, meshdir="should_not_override/")
    compiler = builder.root.find("compiler")
    assert compiler is not None
    assert compiler.attrib["meshdir"] == "keep_this/"
    assert compiler.attrib["angle"] == "degree"
    assert compiler.attrib["custom"] == "yes"


def test_compiler_existing_tag_with_partial_attrs():
    xml = """
    <mujoco>
      <compiler angle="degree"/>
      <worldbody/>
    </mujoco>
    """
    builder = Builder(xml, meshdir="should_not_override/")
    compiler = builder.root.find("compiler")
    assert compiler is not None
    assert "meshdir" not in compiler.attrib  # meshdir should not be injected
    assert compiler.attrib["angle"] == "degree"

# Comprehensive tests

def test_builder_file_path(tmp_path):
    xml_content = """
    <mujoco><worldbody><body name='test'/></worldbody></mujoco>
    """
    xml_file = tmp_path / "test.xml"
    xml_file.write_text(xml_content)
    builder = Builder(str(xml_file))
    assert builder.root.find("worldbody") is not None
    assert builder.root.find("worldbody").find("body").attrib["name"] == "test"

def test_builder_invalid_file_path():
    import tempfile
    with pytest.raises(FileNotFoundError):
        Builder("nonexistent_file.xml")

def test_builder_invalid_xml():
    bad_xml = "<mujoco><worldbody><body></mujoco>"  # malformed
    with pytest.raises(Exception):
        Builder(bad_xml)

def test_compiler_default_attributes():
    xml = "<mujoco><worldbody/></mujoco>"
    builder = Builder(xml)
    compiler = builder.root.find("compiler")
    assert compiler is not None
    assert compiler.attrib["angle"] == "radian"
    assert compiler.attrib["balanceinertia"] == "true"
    assert compiler.attrib["discardvisual"] == "true"

def test_compiler_preserves_existing_attributes():
    xml = "<mujoco><compiler angle='degree' custom='yes'/><worldbody/></mujoco>"
    builder = Builder(xml)
    compiler = builder.root.find("compiler")
    assert compiler is not None
    assert compiler.attrib["angle"] == "degree"
    assert compiler.attrib["custom"] == "yes"

def test_merge_non_overlapping_tags():
    xml1 = "<mujoco><asset><texture name='t1'/></asset></mujoco>"
    xml2 = "<mujoco><worldbody><body name='b1'/></worldbody></mujoco>"
    builder1 = Builder(xml1)
    builder2 = Builder(xml2)
    merged = builder1 + builder2
    assert merged.root.find("asset") is not None
    assert merged.root.find("worldbody") is not None

def test_merge_nested_tags():
    xml1 = "<mujoco><asset><texture name='t1'/></asset></mujoco>"
    xml2 = "<mujoco><asset><material name='m1'/></asset></mujoco>"
    builder1 = Builder(xml1)
    builder2 = Builder(xml2)
    merged = builder1 + builder2
    asset = merged.root.find("asset")
    assert asset is not None
    names = {el.attrib["name"] for el in asset}
    assert "t1" in names and "m1" in names

def test_add_and_radd():
    xml1 = "<mujoco><worldbody><body name='a'/></worldbody></mujoco>"
    xml2 = "<mujoco><worldbody><body name='b'/></worldbody></mujoco>"
    builder1 = Builder(xml1)
    builder2 = Builder(xml2)
    merged1 = builder1 + builder2
    merged2 = builder2.__radd__(builder1)
    assert any(body.attrib["name"] == "a" for body in merged1.root.find("worldbody"))
    assert any(body.attrib["name"] == "b" for body in merged1.root.find("worldbody"))
    assert any(body.attrib["name"] == "a" for body in merged2.root.find("worldbody"))
    assert any(body.attrib["name"] == "b" for body in merged2.root.find("worldbody"))

def test_str_and_repr():
    xml = "<mujoco><worldbody><body name='test'/></worldbody></mujoco>"
    builder = Builder(xml)
    s = str(builder)
    r = repr(builder)
    assert s.startswith("<mujoco>") and "body" in s
    assert r == s

def test_len():
    xml = "<mujoco><worldbody/><asset/></mujoco>"
    builder = Builder(xml)
    assert len(builder) == 2

def test_merge_with_empty_builder():
    xml = "<mujoco><worldbody/></mujoco>"
    builder1 = Builder(xml)
    builder2 = Builder("<mujoco/>")
    merged = builder1 + builder2
    assert merged.root.find("worldbody") is not None

def test_large_complex_xml():
    xml = "<mujoco>" + "".join(f"<body name='b{i}'/>" for i in range(100)) + "</mujoco>"
    builder = Builder(xml)
    assert all(f"b{i}" in str(builder) for i in range(100))
