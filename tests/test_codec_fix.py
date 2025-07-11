"""Tests for codec mapping functionality."""
import os
import tempfile

import numpy as np
import pytest

from mujoco_toolbox import Simulation


class TestCodecMapping:
    """Test cases for codec mapping functionality."""

    @pytest.fixture
    def simple_model(self) -> str:
        """Simple MuJoCo model for testing."""
        return """
        <mujoco>
            <visual>
                <global offwidth="100" offheight="100" />
            </visual>
            <worldbody>
                <body name="floor" pos="0 0 0">
                    <geom name="floor" type="plane" size="1 1 .1" />
                </body>
            </worldbody>
        </mujoco>
        """

    @pytest.fixture
    def sim_with_frames(self, simple_model):
        """Simulation with injected frames for testing save functionality."""
        sim = Simulation(simple_model, fps=10, duration=1.0)
        # Inject fake frames to bypass rendering in test environment
        fake_frames = np.random.randint(0, 255, (5, 100, 100, 3), dtype=np.uint8)
        sim._frames = fake_frames
        sim._fps = 10
        return sim

    def test_mp4_codec_mapping(self, sim_with_frames) -> None:
        """Test that mp4 codec is properly mapped to libx264."""
        with tempfile.NamedTemporaryFile(delete=False) as tmp:
            temp_path = tmp.name

        try:
            result_path = sim_with_frames.save(title=temp_path, codec="mp4")
            assert os.path.exists(result_path), "MP4 file should be created"
            assert result_path.endswith(".mp4"), "File should have .mp4 extension"
            assert os.path.getsize(result_path) > 0, "File should not be empty"
        finally:
            if os.path.exists(result_path):
                os.remove(result_path)

    def test_container_format_mapping(self, sim_with_frames) -> None:
        """Test various container format mappings."""
        test_cases = [
            ("mp4", ".mp4"),
            ("avi", ".avi"),
            ("mov", ".mov"),
            ("mkv", ".mkv"),
            ("webm", ".webm"),
        ]

        for codec, expected_ext in test_cases:
            with tempfile.NamedTemporaryFile(delete=False) as tmp:
                temp_path = tmp.name

            try:
                result_path = sim_with_frames.save(title=temp_path, codec=codec)
                assert os.path.exists(result_path), f"{codec} file should be created"
                assert result_path.endswith(expected_ext), f"File should have {expected_ext} extension"
                assert os.path.getsize(result_path) > 0, f"{codec} file should not be empty"
            finally:
                if os.path.exists(result_path):
                    os.remove(result_path)

    def test_codec_to_container_mapping(self, sim_with_frames) -> None:
        """Test that video codecs are mapped to appropriate containers."""
        test_cases = [
            ("libx264", ".mp4"),
            ("libx265", ".mp4"),
            ("h264", ".mp4"),
            ("libvpx", ".webm"),
        ]

        for codec, expected_ext in test_cases:
            with tempfile.NamedTemporaryFile(delete=False) as tmp:
                temp_path = tmp.name

            try:
                result_path = sim_with_frames.save(title=temp_path, codec=codec)
                assert os.path.exists(result_path), f"{codec} file should be created"
                assert result_path.endswith(expected_ext), f"File should have {expected_ext} extension for {codec}"
                assert os.path.getsize(result_path) > 0, f"{codec} file should not be empty"
            finally:
                if os.path.exists(result_path):
                    os.remove(result_path)

    def test_backward_compatibility(self, sim_with_frames) -> None:
        """Test that existing codecs like GIF still work."""
        with tempfile.NamedTemporaryFile(delete=False) as tmp:
            temp_path = tmp.name

        try:
            result_path = sim_with_frames.save(title=temp_path, codec="gif")
            assert os.path.exists(result_path), "GIF file should be created"
            assert result_path.endswith(".gif"), "File should have .gif extension"
            assert os.path.getsize(result_path) > 0, "GIF file should not be empty"
        finally:
            if os.path.exists(result_path):
                os.remove(result_path)

    def test_case_insensitive_mapping(self, sim_with_frames) -> None:
        """Test that codec mapping is case-insensitive."""
        test_cases = ["MP4", "Mp4", "mP4", "mp4"]

        for codec in test_cases:
            with tempfile.NamedTemporaryFile(delete=False) as tmp:
                temp_path = tmp.name

            try:
                result_path = sim_with_frames.save(title=temp_path, codec=codec)
                assert os.path.exists(result_path), f"{codec} file should be created"
                assert result_path.endswith(".mp4"), f"File should have .mp4 extension for {codec}"
                assert os.path.getsize(result_path) > 0, f"{codec} file should not be empty"
            finally:
                if os.path.exists(result_path):
                    os.remove(result_path)
