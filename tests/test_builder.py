import pytest

import mujoco_toolbox as mjtb

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
    return mjtb.Builder(string1) + mjtb.Builder(string2) + mjtb.Builder(string3) + mjtb.Builder(string4)

@pytest.fixture
def builder_with_args():
    return mjtb.Builder(string1, string2, string3, string4)

def test_builder_creation(builder) -> None:
    assert builder is not None, "Builder object is empty"
    assert len(builder) > 0, "Builder object has no elements"

def test_builder_with_args_creation(builder_with_args) -> None:
    assert builder_with_args is not None, "Builder2 object is empty"
    assert len(builder_with_args) > 0, "Builder2 object has no elements"

def test_builder_merge() -> None:
    builder1 = mjtb.Builder(string1)
    builder2 = mjtb.Builder(string2)
    merged_builder = builder1 + builder2
    assert merged_builder is not None, "Merged builder is empty"
    assert len(merged_builder) >= len(builder1), "Merged builder did not increase in size"

def test_builder_save(tmp_path) -> None:
    builder = mjtb.Builder(string1)
    save_path = tmp_path / "test_model.xml"
    builder.save(str(save_path))
    assert save_path.exists(), "Saved file does not exist"

def test_builder_str_representation(builder) -> None:
    xml_str = str(builder)
    assert xml_str.startswith("<mujoco>"), "String representation does not start with <mujoco>"
    assert "</mujoco>" in xml_str, "String representation does not end with </mujoco>"

def test_builder_property_xml(builder) -> None:
    xml_str = builder.xml
    assert xml_str.startswith("<mujoco>"), "XML property does not start with <mujoco>"
    assert "</mujoco>" in xml_str, "XML property does not end with </mujoco>"

def test_builder_invalid_input() -> None:
    with pytest.raises(ValueError, match="Input is required to initialize the Builder"):
        mjtb.Builder()

    with pytest.raises(TypeError, match="Input must be an XML string or a file path"):
        mjtb.Builder(123)

def test_builder_merge_sections() -> None:
    builder1 = mjtb.Builder(string1)
    builder2 = mjtb.Builder(string2)
    merged_builder = builder1 + builder2
    assert merged_builder.root.find("asset") is not None, "Merged builder is missing 'asset' section"
    assert merged_builder.root.find("worldbody") is not None, "Merged builder is missing 'worldbody' section"
