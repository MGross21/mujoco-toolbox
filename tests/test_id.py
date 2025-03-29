import os

import mujoco_toolbox as mjtb

# Initialize the model
model = mjtb.Wrapper("tests/models/box_and_leg.xml")

# Test name2id for joints
def test_name2id_joints() -> None:
    assert model.name2id("prismatic_1") == 0
    assert model.name2id("joint_1") == 1
    assert model.name2id("joint_2") == 2
    assert model.name2id("nonexistent_joint") is None

# Test name2id for bodies
def test_name2id_bodies() -> None:
    assert model.name2id("body_1") == 1
    assert model.name2id("leg_1") == 2
    assert model.name2id("leg_2") == 3
    assert model.name2id("floor") == 4
    assert model.name2id("nonexistent_body") is None

# Test name2id for geoms
def test_name2id_geoms() -> None:
    assert model.name2id("geom_1") is None  # Assuming no explicit geom names
    assert model.name2id("nonexistent_geom") is None

# Test name2id for actuators
def test_name2id_actuators() -> None:
    assert model.name2id("motor_1") == 0
    assert model.name2id("nonexistent_motor") is None

# Test name2id for sensors
def test_name2id_sensors() -> None:
    assert model.name2id("body_pos") == 0
    assert model.name2id("nonexistent_sensor") is None

def test_name2id_lights() -> None:
    assert model.name2id("top") == 0
    assert model.name2id("nonexistent_light") is None

# Run all tests
if __name__ == "__main__":
    test_name2id_joints()
    test_name2id_bodies()
    test_name2id_geoms()
    test_name2id_actuators()
    test_name2id_sensors()
    test_name2id_lights()
    mjtb.utils._print_success(f"{os.path.basename(__file__)} Tests passed!\n")
