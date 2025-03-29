from mujoco_toolbox import WORLD_ASSETS, glovebox


def test_world_assets() -> None:
    """Test if WORLD_ASSETS is correctly defined."""
    assert "<asset>" in WORLD_ASSETS
    assert "<texture" in WORLD_ASSETS
    assert "<material" in WORLD_ASSETS

def test_glovebox_generation() -> None:
    """Test glovebox XML generation with default parameters."""
    xml = glovebox()
    assert "<mujoco>" in xml
    assert "<asset>" in xml
    assert "<worldbody>" in xml
    assert 'material="glass"' in xml
    assert 'size="0.625 0.025 0.5"' in xml  # Default width/2, glass_thickness/2, height/2

def test_glovebox_custom_dimensions() -> None:
    """Test glovebox XML generation with custom dimensions."""
    xml = glovebox(width=2.0, depth=1.5, height=2.0, glass_thickness=0.1)
    assert 'size="1.0 0.05 1.0"' in xml  # Custom width/2, glass_thickness/2, height/2
    assert 'pos="0 0 2.05"' in xml  # Custom height + glass_thickness/2

def test_world_xml() -> None:
    """Test if a simple world XML is correctly constructed."""
    world = f"""
    <mujoco>
        {WORLD_ASSETS}
        <worldbody>
            <body name="floor" pos="0 0 0">
                <geom type="plane" size="5 5 0.1" material="grid"/>
            </body>
        </worldbody>
    </mujoco>
    """
    assert "<mujoco>" in world
    assert "<worldbody>" in world
    assert '<geom type="plane"' in world

if __name__ == "__main__":
    test_world_assets()
    test_glovebox_generation()
    test_glovebox_custom_dimensions()
    test_world_xml()
    print("All tests passed!")
