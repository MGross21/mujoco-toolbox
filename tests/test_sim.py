import math

import mujoco
import numpy as np
import pytest
from helpers import BOX_AND_LEG, DROPPED_BOX, MODELS

import mujoco_toolbox as mjtb


@pytest.mark.parametrize("model", sorted(MODELS.glob("*.xml")), ids=lambda p: p.stem)
def test_local_models_run(model: object) -> None:
    rec = mjtb.Simulation(model).run(1)
    assert set(rec.data) == mjtb.DEFAULT_CAPTURE
    assert len(rec.time) == 100


def test_urdf_with_meshes() -> None:
    ur5 = MODELS / "ur5"
    sim = mjtb.Simulation(ur5 / "ur5.urdf", meshdir=str(ur5 / "meshes" / "collision"))
    rec = sim.run(0.5, capture={"qpos"})
    assert rec.data["qpos"].shape == (50, sim.model.nq)


def test_settings_applied() -> None:
    sim = mjtb.Simulation(DROPPED_BOX, timestep=1e-3, gravity=[0, 0, -1])
    assert sim.timestep == 1e-3
    np.testing.assert_array_equal(sim.gravity, [0, 0, -1])
    assert "timestep=0.001" in repr(sim)


@pytest.mark.parametrize(
    ("kwargs", "error", "match"),
    [
        ({"timestep": 0}, ValueError, "timestep"),
        ({"gravity": [0, 0]}, ValueError, "3D vector"),
        ({"controller": "nope"}, TypeError, "callable"),
        ({"initial_conditions": {"nope": 1}}, ValueError, "nope"),
        ({"keyframe": "nope"}, KeyError, "nope"),
        ({"gain": 3}, TypeError, "gain"),
    ],
)
def test_invalid_settings(kwargs: dict, error: type[Exception], match: str) -> None:
    with pytest.raises(error, match=match):
        mjtb.Simulation(DROPPED_BOX, **kwargs)


@pytest.mark.parametrize("path", [MODELS / "missing.xml", MODELS / "missing.urdf"])
def test_missing_file(path: object) -> None:
    with pytest.raises(FileNotFoundError):
        mjtb.Simulation(path)


@pytest.mark.parametrize(
    ("kwargs", "match"),
    [
        ({"duration": -1}, "duration"),
        ({"capture_rate": 0}, "capture_rate"),
        ({"capture_rate": 501}, "one capture per timestep"),
        ({"capture": {"qpos", "nope"}}, "nope"),
        ({"fps": 0}, "fps"),
    ],
)
def test_invalid_run(kwargs: dict, match: str) -> None:
    with pytest.raises(ValueError, match=match):
        mjtb.Simulation(DROPPED_BOX).run(**kwargs)


def test_short_run_keeps_default_rate() -> None:
    assert len(mjtb.Simulation(BOX_AND_LEG).run(0.1).time) == 10


def test_capture_all() -> None:
    rec = mjtb.Simulation(BOX_AND_LEG).run(0.1, capture="all")
    assert {"qpos", "xpos", "cvel", "efc_force"} <= set(rec.data)


def test_runs_are_independent() -> None:
    sim = mjtb.Simulation(BOX_AND_LEG, controller=mjtb.sin(amplitude=1))
    first, second = sim.run(0.5), sim.run(0.5)
    np.testing.assert_array_equal(first.data["qpos"], second.data["qpos"])
    assert first is not second


def test_controller_called_every_step() -> None:
    times: list[float] = []
    sim = mjtb.Simulation(BOX_AND_LEG, controller=lambda _m, d: times.append(d.time))
    sim.run(0.1)
    assert len(times) == round(0.1 / sim.timestep)


def test_initial_conditions_and_keyframe() -> None:
    keyed = DROPPED_BOX.replace("</mujoco>", '<keyframe><key name="high" qpos="0 0 2 1 0 0 0"/></keyframe></mujoco>')
    assert mjtb.Simulation(keyed, keyframe="high").run(0.01).data["qpos"][0, 2] == pytest.approx(2)
    sim = mjtb.Simulation(keyed, keyframe="high", initial_conditions={"qpos": [0, 0, 3, 1, 0, 0, 0]})
    assert sim.run(0.01).data["qpos"][0, 2] == pytest.approx(3)


def test_record_when_gates_capture() -> None:
    rec = mjtb.Simulation(BOX_AND_LEG).run(1, capture_rate=10, record_when=lambda d: d.time > 0.45)
    assert len(rec.time) == 5
    assert rec.time.min() > 0.45


def test_settle_stores_resting_pose() -> None:
    sim = mjtb.Simulation(DROPPED_BOX).settle()
    assert sim.initial_conditions["qpos"][2] == pytest.approx(0.1, abs=1e-3)
    assert np.abs(sim.run(0.5).data["qvel"]).max() < 1e-3


def test_settle_warns_when_never_at_rest() -> None:
    with pytest.warns(mjtb.SimulationWarning, match="did not settle"):
        mjtb.Simulation(DROPPED_BOX).settle(max_time=0.01)


def test_sweep_matches_run() -> None:
    sim = mjtb.Simulation(BOX_AND_LEG)
    earth, moon = sim.sweep({"opt.gravity": [[0, 0, -9.81], [0, 0, -1.62]]}, 0.5, capture={"qpos"}, capture_rate=20)
    expected = sim.run(0.5, capture={"qpos"}, capture_rate=20)
    np.testing.assert_allclose(earth.time, expected.time)
    np.testing.assert_allclose(earth.data["qpos"], expected.data["qpos"], atol=1e-9)
    assert not np.allclose(moon.data["qpos"], expected.data["qpos"])
    assert set(moon.data) == {"qpos"}


def test_sweep_rejects_controller_and_mismatched_sizes() -> None:
    with pytest.raises(mjtb.SimulationError, match="controller"):
        mjtb.Simulation(BOX_AND_LEG, controller=mjtb.sin()).sweep({"opt.gravity": [[0, 0, -9.81]]})
    with pytest.raises(ValueError, match="same number"):
        mjtb.Simulation(BOX_AND_LEG).sweep({"opt.gravity": [[0, 0, -9.81]], "qvel": np.zeros((2, 3))})


def test_studio_hot_reload_needs_model_file() -> None:
    with pytest.raises(ValueError, match="single model file"):
        mjtb.Simulation(DROPPED_BOX).studio(hot_reload=True)


def test_controllers() -> None:
    m = mujoco.MjModel.from_xml_path(str(BOX_AND_LEG))
    d = mujoco.MjData(m)
    for make in (mjtb.sin, mjtb.cos, mjtb.step, mjtb.random):
        make()(m, d)
    mjtb.step(amplitude=2, axis=0)(m, d)
    assert d.qpos[0] == 2
    assert mjtb.sin().__name__ == "sin"
    with pytest.raises(TypeError, match="amplitud"):
        mjtb.sin(amplitud=2)
    with pytest.raises(ValueError, match="both"):
        mjtb.step(joint=[0], axis=0)


GRID_FLOOR = '<mujoco><worldbody><geom type="plane" size="1 1 .1" material="grid"/></worldbody></mujoco>'


def test_merged_sources_share_assets() -> None:
    sim = mjtb.Simulation(GRID_FLOOR, mjtb.WORLD_ASSETS, mjtb.glovebox())
    assert sim.model.material("grid").id >= 0
    assert sim.model.body("glovebox").id > 0
    assert "<mujoco" in sim.xml


def test_urdf_merged_into_scene() -> None:
    ur5 = MODELS / "ur5"
    sim = mjtb.Simulation(DROPPED_BOX, ur5 / "ur5.urdf", meshdir=str(ur5 / "meshes" / "collision"))
    assert sim.model.body("shoulder_link").id > sim.model.body("world").id
    assert sim.model.nbody > 2


def test_spec_sources_are_copied() -> None:
    spec = mujoco.MjSpec.from_string(DROPPED_BOX)
    sim = mjtb.Simulation(spec, mjtb.glovebox())
    assert spec.compile().nbody == 2
    assert mjtb.Simulation(sim.spec).model.nbody == sim.model.nbody


def test_duplicate_names_and_bad_xml_rejected() -> None:
    with pytest.raises(ValueError, match="repeated name"):
        mjtb.Simulation(mjtb.glovebox(), mjtb.glovebox())
    with pytest.raises(ValueError, match="unrecognized element"):
        mjtb.Simulation("<mujoco><worldbody><bodyy/></worldbody></mujoco>")


def test_mjcf_keeps_mujoco_angle_default() -> None:
    sim = mjtb.Simulation(MODELS / "humanoid.xml")
    assert np.abs(sim.model.jnt_range).max() < math.pi
