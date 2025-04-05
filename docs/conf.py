import sys
from pathlib import Path
from typing import TYPE_CHECKING

import mujoco_toolbox as mjtb

if TYPE_CHECKING:
    from sphinx.application import Sphinx

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.absolute()))

# Project information
project = mjtb.__name__
author = mjtb.__author__
release = mjtb.__version__

# General configuration
extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
]

templates_path = ["_templates"]
exclude_patterns = []

# Options for HTML output
html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]

def setup(app: "Sphinx") -> None:
    """Custom Sphinx setup function."""
    app.add_css_file("custom.css")
