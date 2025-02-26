import os
import sys

# sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import mujoco_toolbox as mjtb
from mujoco_toolbox import Wrapper

def test_wrapper():
    """Test 1: Create a simulation with a box and a leg, and run it with a sine controller."""
    model = "/tests/models/box_leg.xml"
    
    sim = Wrapper(xml=model, duration=10, fps=30, resolution=(800, 600), controller=mjtb.sineController, amplitude=0.5, frequency=0.25)
    sim.runSim(render=True).renderMedia(codec="gif", title="sine_wave", save=True)

if __name__ == "__main__":
    test_wrapper()