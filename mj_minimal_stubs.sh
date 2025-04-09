# Install directly from GitHub
pip install "git+https://github.com/deepmind/mujoco.git#egg=mujoco&subdirectory=python"

# Then optionally generate stubs
pip install mypy
stubgen -m mujoco -o stubs