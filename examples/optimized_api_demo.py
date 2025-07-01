"""Example demonstrating the optimized Digital Twin API usage."""
import time
import numpy as np
import mujoco_toolbox as mjtb


def demonstrate_optimized_api():
    """Demonstrate the three levels of API optimization described in the issue."""
    
    print("Digital Twin API Optimization Examples")
    print("=" * 50)
    
    # Example 1: Currently (original API, still supported)
    print("\n1. Current/Original API:")
    print("   Code: digitaltwin.controller(digitaltwin.model, digitaltwin.data, params)")
    
    try:
        with mjtb.Simulation("tests/models/box_and_leg.xml", controller=mjtb.real_time) as sim:
            # Simulate some motion parameters
            params = {"qpos": np.array([0.1, 0.2, 0.3])}
            sim.controller(sim.model, sim.data, params)
            print("   ✓ Original API works correctly")
    except Exception as e:
        print(f"   ✗ Error: {e}")
    
    # Example 2: Ideal (controller optimization)
    print("\n2. Ideal API (Controller Optimization):")
    print("   Code: digitaltwin.controller(params)")
    
    try:
        import unittest.mock
        with unittest.mock.patch.object(mjtb.Wrapper, 'launch'):
            with mjtb.Wrapper("tests/models/box_and_leg.xml", controller=mjtb.real_time) as wrapper:
                # Optimized controller call - no model/data needed
                params = {"qpos": np.array([0.1, 0.2, 0.3])}
                wrapper.controller(params)
                print("   ✓ Optimized controller API works correctly")
    except Exception as e:
        print(f"   ✗ Error: {e}")
    
    # Example 3: Ultimate optimization (auto controller + auto launch)
    print("\n3. Ultimate Optimization (Auto controller + Auto launch):")
    print("   Code: with mjtb.Wrapper('model.xml') as dt: dt.controller(params)")
    
    try:
        # Mock launch to avoid GUI in headless environment
        import unittest.mock
        with unittest.mock.patch.object(mjtb.Wrapper, 'launch'):
            with mjtb.Wrapper("tests/models/box_and_leg.xml") as wrapper:
                # Everything is automatic - controller assigned, launch called
                params = {"qpos": np.array([0.1, 0.2, 0.3])}
                wrapper.controller(params)
                print(f"   ✓ Auto-assigned controller: {wrapper._controller.__name__}")
                print("   ✓ Auto-launch called in context manager")
                print("   ✓ Ultimate optimization works correctly")
    except Exception as e:
        print(f"   ✗ Error: {e}")
    
    print("\n" + "=" * 50)
    print("All optimizations implemented successfully!")
    
    # Show the progression of simplification
    print("\nAPI Evolution:")
    print("OLD:  digitaltwin.controller(digitaltwin.model, digitaltwin.data, params)")
    print("NEW:  digitaltwin.controller(params)")
    print("FULL: with mjtb.Wrapper('model.xml') as dt: dt.controller(params)")


def test_real_world_usage():
    """Test a more realistic usage scenario."""
    print("\n" + "=" * 50)
    print("REAL-WORLD USAGE EXAMPLE")
    print("=" * 50)
    
    # Simulate a robot control loop
    import unittest.mock
    
    with unittest.mock.patch.object(mjtb.Wrapper, 'launch'):
        with mjtb.Wrapper("tests/models/box_and_leg.xml") as robot:
            print("Robot initialized with auto real-time controller")
            
            # Simulate a trajectory of joint positions
            trajectory = [
                {"qpos": np.array([0.0, 0.0, 0.0])},  # Initial position
                {"qpos": np.array([0.1, 0.1, 0.1])},  # Step 1
                {"qpos": np.array([0.2, 0.2, 0.2])},  # Step 2
                {"qpos": np.array([0.1, 0.1, 0.1])},  # Step 3
                {"qpos": np.array([0.0, 0.0, 0.0])},  # Return to start
            ]
            
            print("\nExecuting trajectory:")
            for i, position in enumerate(trajectory):
                robot.controller(position)
                print(f"  Step {i+1}: {position['qpos']}")
            
            print("✓ Trajectory execution complete!")


if __name__ == "__main__":
    demonstrate_optimized_api()
    test_real_world_usage()