# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information


import sys
import toml
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pyproject = toml.load("../../pyproject.toml")
version = pyproject["tool"]["poetry"]["version"]


project = pyproject["tool"]["poetry"]["name"]
copyright = "2025, Markus Grotz"
author = "Markus Grotz"
release = version

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.doctest",
    "sphinx.ext.autosummary",
    # "sphinx.ext.viewcode",
    "sphinx.ext.autosectionlabel",
    "sphinx.ext.todo",
    "sphinxcontrib.video",
    "myst_parser",
]

templates_path = ["_templates"]
exclude_patterns = []


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]
html_logo = "./_static/logo.png"

source_suffix = {
    ".rst": "restructuredtext",
    ".txt": "markdown",
    ".md": "markdown",
}

highlight_options = {
    "default": {"stripall": True},
    "bash": {"startinline": True},
}


autodoc_mock_imports = [
    "torch",
    "openai",
    "xarm",
    "pyrealsense2",
    "urx",
    "franky",
    "sam2act",
]
