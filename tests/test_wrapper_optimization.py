"""Tests for the optimized Digital Twin API (Wrapper class)."""

import numpy as np
import pytest
import mujoco_toolbox as mjtb


def test_wrapper_auto_controller_assignment():
    """Test that Wrapper automatically assigns real_time controller when none provided."""
    # Create wrapper without specifying controller
    wrapper = mjtb.Wrapper("tests/models/box_and_leg.xml")
    
    # Check that real_time controller was automatically assigned
    assert wrapper.controller is not None
    assert wrapper.controller.__name__ == "real_time"


def test_wrapper_controller_optimization():
    """Test that the optimized controller method works without model/data parameters."""
    # Mock test with dummy data since we can't run GUI in headless mode
    class MockController:
        def __init__(self):
            self.called_with = None
        
        def __call__(self, model, data, *args, **kwargs):
            self.called_with = (model, data, args, kwargs)
    
    wrapper = mjtb.Wrapper("tests/models/box_and_leg.xml")
    
    # Replace with mock controller for testing
    mock_controller = MockController()
    wrapper._controller = mock_controller
    
    # Test the optimized controller call
    test_params = {"qpos": [1.0, 2.0, 3.0]}
    wrapper.controller(test_params)
    
    # Verify that model and data were automatically passed
    model, data, args, kwargs = mock_controller.called_with
    assert model is wrapper._model
    assert data is wrapper._data
    assert len(args) == 1
    assert args[0] == test_params


def test_wrapper_context_manager_basic():
    """Test that Wrapper can be used as a context manager."""
    # Test context manager without launching GUI in headless mode
    import unittest.mock
    
    with unittest.mock.patch.object(mjtb.Wrapper, 'launch') as mock_launch:
        with mjtb.Wrapper("tests/models/box_and_leg.xml") as wrapper:
            assert wrapper is not None
            assert wrapper.controller is not None
            assert wrapper._launched is True  # Check that launch was called
            # Verify that launch was called with default parameters
            mock_launch.assert_called_once_with(show_menu=False)


def test_wrapper_explicit_controller():
    """Test that explicitly provided controller is used instead of auto-assigned."""
    from mujoco_toolbox.controllers import sin
    
    wrapper = mjtb.Wrapper("tests/models/box_and_leg.xml", controller=sin)
    assert wrapper.controller.__name__ == "sin"


def test_wrapper_no_controller_warning():
    """Test that calling controller when None returns None."""
    wrapper = mjtb.Wrapper("tests/models/box_and_leg.xml")
    wrapper._controller = None
    
    # Controller should return None when no controller is set
    assert wrapper.controller is None


def test_wrapper_backwards_compatibility():
    """Test that Wrapper still works with existing API patterns."""
    wrapper = mjtb.Wrapper("tests/models/box_and_leg.xml")
    
    # Should still work with old style calling (though not recommended)
    test_params = {"qpos": [1.0]}
    
    # The new controller method should work  
    wrapper.controller(test_params)
    
    # Verify the controller is callable
    assert callable(wrapper.controller)