import mujoco_toolbox as mjtb
from mujoco_toolbox import Wrapper
import os

def test_wrapper():
    """Test 1: Create a simulation with a box and a leg, and run it with a sine controller."""

    model = os.path.join(os.getcwd(), "tests", "models", "box_and_leg.xml")

    mjtb.VERBOSITY = True
  
    test1 = Wrapper(xml=model, duration=10, fps=30, resolution=(800, 600), controller=mjtb.sineController, amplitude=1e2, frequency=1e3).runSim()

    assert test1.captured_data.__len__() == mjtb.CAPTURE_PARAMETERS.__len__(), "Captured data is not complete."
    assert test1._captured_data.__len__() == (test1.duration * test1.data_rate) + 1, "Captured data is not complete."
    # assert os.path.exists(test1.renderMedia(codec="gif", title="sine_wave", save=True)), "Media file not found."

if __name__ == "__main__":
    test_wrapper()