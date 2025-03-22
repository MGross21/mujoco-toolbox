import os
import sys

import mujoco_toolbox as mjtb

sys.path.insert(0, os.path.abspath(".."))

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

def setup(app):
    app.add_css_file("custom.css")
