import sys
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from sphinx.application import Sphinx

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
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
]

templates_path = ["_templates"]
exclude_patterns = []

# Disable built-in Pygments styles to fully override via CSS
pygments_style = "none"

# Options for HTML output
html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]

def skip_properties(app, what, name, obj, skip, options):
    """Skip all @property decorated functions."""
    if isinstance(obj, property):
        return True  # skip this property
    return skip

def setup(app: "Sphinx") -> None:
    """Custom Sphinx setup function."""
    app.add_css_file("custom.css")
    app.connect("autodoc-skip-member", skip_properties)
