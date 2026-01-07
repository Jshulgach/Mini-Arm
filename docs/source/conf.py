# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import sys
sys.path.insert(0, os.path.abspath('../../ros2/miniarm_core'))  # Ensure miniarm_core is discoverable

from miniarm_core.version import __version__

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Mini-Arm'
copyright = '2025, Jonathan Shulgach'
author = 'Jonathan Shulgach'
release = __version__

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
    "sphinx_autodoc_typehints",
    "sphinx_copybutton",
    "sphinx_design",
    "m2r2",
]

templates_path = ['_templates']
exclude_patterns = []
autodoc_typehints = 'description'
autodoc_member_order = 'bysource'
add_module_names = False


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'furo'  # Modern, clean theme (alternative: 'sphinx_rtd_theme')
html_static_path = ['_static']
html_logo = "_static/mini-arm-logo.png"
html_favicon = "_static/favicon.ico"

# Theme options
html_theme_options = {
    "light_css_variables": {
        "color-brand-primary": "#1abc9c",
        "color-brand-content": "#1abc9c",
    },
    "dark_css_variables": {
        "color-brand-primary": "#1abc9c",
        "color-brand-content": "#1abc9c",
    },
}

# For Markdown support:
source_suffix = ['.rst', '.md']

# Set your master doc:
master_doc = 'index'
