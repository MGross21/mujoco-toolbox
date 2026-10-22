"""Sphinx configuration for the mujoco-toolbox docs."""

import sys
from pathlib import Path

# Add the parent directory (repo root) to sys.path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import mujoco_toolbox as mjtb

# Project information
project = mjtb.__name__
author = mjtb.__author__
release = mjtb.__version__

# General configuration
extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.intersphinx",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
    "myst_parser",
    "sphinx_copybutton",
]

autodoc_default_options = {
    "members": True,
    "member-order": "bysource",
    "show-inheritance": True,
}
autodoc_typehints = "description"
autodoc_typehints_description_target = "documented_params"
autodoc_type_aliases = {"CaptureSpec": "mujoco_toolbox.sim.CaptureSpec"}
napoleon_google_docstring = True
napoleon_numpy_docstring = False
napoleon_use_rtype = False

intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
    "numpy": ("https://numpy.org/doc/stable", None),
}

myst_heading_anchors = 3
suppress_warnings = ["myst.header"]  # README sections skip heading levels
copybutton_prompt_text = r">>> |\.\.\. |\$ "
copybutton_prompt_is_regexp = True

html_theme = "furo"
html_title = "MuJoCo Toolbox"
html_logo = "../assets/images/mjtb_logo_transparent.png"
