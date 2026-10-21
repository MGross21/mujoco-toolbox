import pytest

import mujoco_toolbox as mjtb

# Initialize the model
test = mjtb.Simulation("tests/models/box_and_leg.xml")


# Test name2id for joints
def test_name2id_joints() -> None:
    assert test.name2id("prismatic_1") == 0
    assert test.name2id("joint_1") == 1
    assert test.name2id("joint_2") == 2
    with pytest.raises(ValueError, match="not found"):
        test.name2id("nonexistent_joint")


# Test name2id for bodies
def test_name2id_bodies() -> None:
    assert test.name2id("body_1") == 1
    assert test.name2id("leg_1") == 2
    assert test.name2id("leg_2") == 3
    assert test.name2id("floor") == 4
    with pytest.raises(ValueError, match="not found"):
        test.name2id("nonexistent_body")


# Test name2id for geoms
def test_name2id_geoms() -> None:
    with pytest.raises(ValueError, match="not found"):
        test.name2id("geom_1")  # Assuming no explicit geom names
    with pytest.raises(ValueError, match="not found"):
        test.name2id("nonexistent_geom")


# Test name2id for actuators
def test_name2id_actuators() -> None:
    assert test.name2id("motor_1") == 0
    with pytest.raises(ValueError, match="not found"):
        test.name2id("nonexistent_motor")


# Test name2id for sensors
def test_name2id_sensors() -> None:
    assert test.name2id("body_pos") == 0
    with pytest.raises(ValueError, match="not found"):
        test.name2id("nonexistent_sensor")


def test_name2id_lights() -> None:
    assert test.name2id("top") == 0
    with pytest.raises(ValueError, match="not found"):
        test.name2id("nonexistent_light")


def test_name2id_typed() -> None:
    assert test.name2id("leg_1", "body") == 2
    with pytest.raises(ValueError, match="not found"):
        test.name2id("leg_1", "joint")
    with pytest.raises(ValueError, match="Unknown MuJoCo object type"):
        test.name2id("leg_1", "not_a_type")


def test_id2name() -> None:
    assert test.id2name(1, "joint") == "joint_1"
    assert test.id2name(2, "body") == "leg_1"
    assert test.id2name(0, "actuator") == "motor_1"
    with pytest.raises(ValueError, match="No named body with id 99"):
        test.id2name(99, "body")
