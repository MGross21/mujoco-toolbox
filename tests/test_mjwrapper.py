import os
import sys
from MjWrapper import MjWrapper

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))


def test_mjwrapper():
    """Test 1: Create a simulation with a box and a leg, and run it with a sine controller."""
    model = "./models/box_and_leg.xml"
    sim = MjWrapper(xml=model, duration=10, fps=30, resolution=(800, 600), controller=MjWrapper.sineController, amplitude=1, frequency=1)
    sim.runSim(render=True).renderMedia(codec="gif", title="sine_wave", save=True)


if __name__ == "__main__":
    test_mjwrapper()