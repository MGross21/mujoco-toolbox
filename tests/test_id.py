import os
import pytest

import mujoco_toolbox as mjtb

# Initialize the model
test = mjtb.Wrapper("tests/models/box_and_leg.xml")

# Test name2id for joints
def test_name2id_joints() -> None:
    assert test.name2id("prismatic_1") == 0
    assert test.name2id("joint_1") == 1
    assert test.name2id("joint_2") == 2
    with pytest.raises(ValueError):
        test.name2id("nonexistent_joint")

# Test name2id for bodies
def test_name2id_bodies() -> None:
    assert test.name2id("body_1") == 1
    assert test.name2id("leg_1") == 2
    assert test.name2id("leg_2") == 3
    assert test.name2id("floor") == 4
    with pytest.raises(ValueError):
        test.name2id("nonexistent_body")

# Test name2id for geoms
def test_name2id_geoms() -> None:
    with pytest.raises(ValueError):
        test.name2id("geom_1")  # Assuming no explicit geom names
    with pytest.raises(ValueError):
        test.name2id("nonexistent_geom")

# Test name2id for actuators
def test_name2id_actuators() -> None:
    assert test.name2id("motor_1") == 0
    with pytest.raises(ValueError):
        test.name2id("nonexistent_motor")

# Test name2id for sensors
def test_name2id_sensors() -> None:
    assert test.name2id("body_pos") == 0
    with pytest.raises(ValueError):
        test.name2id("nonexistent_sensor")

def test_name2id_lights() -> None:
    assert test.name2id("top") == 0
    with pytest.raises(ValueError):
        test.name2id("nonexistent_light")

# def test_id2name() -> None:
#     for i in range(test.model.njnt):
#         assert test.id2name(i) == test.model.joint(i).name
#     for i in range(test.model.nbody):
#         assert test.id2name(i) == test.model.body(i).name
#     for i in range(test.model.ngeom):
#         assert test.id2name(i) == test.model.geom(i).name
#     for i in range(test.model.nactuator):
#         assert test.id2name(i) == test.model.actuator(i).name
#     for i in range(test.model.nsensor):
#         assert test.id2name(i) == test.model.sensor(i).name
#     for i in range(test.model.nlights):
#         assert test.id2name(i) == test.model.light(i).name
#     # Test for nonexistent IDs
#     assert test.id2name(test.model.njnt) is None
#     assert test.id2name(test.model.nbody) is None
#     assert test.id2name(test.model.ngeom) is None
#     assert test.id2name(test.model.nactuator) is None
#     assert test.id2name(test.model.nsensor) is None
#     assert test.id2name(test.model.nlights) is None


# Run all tests
if __name__ == "__main__":
    test_name2id_joints()
    test_name2id_bodies()
    test_name2id_geoms()
    test_name2id_actuators()
    test_name2id_sensors()
    test_name2id_lights()
    # test_id2name()
    mjtb.utils._print_success(f"{os.path.basename(__file__)} Tests passed!\n")
