# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import sys
from datetime import datetime
from pathlib import Path

import catkin_pkg.package

# -- Retrieve package version ------------------------------------------------
catkin_dir = Path(__file__).joinpath("../../..").resolve()
catkin_package = catkin_pkg.package.parse_package(
    catkin_dir.joinpath(catkin_pkg.package.PACKAGE_MANIFEST_FILENAME)
)

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information
project = "Panda Gazebo"
copyright = f"{datetime.now().year}, Rick Staa"
author = "Rick Staa"
git_user_name = "rickstaa"
release = catkin_package.version
version = ".".join(release.split(".")[:3])
print("Doc release: ", release)
print("Doc version: ", version)

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

# If your documentation needs a minimal Sphinx version, state it here.
needs_sphinx = "3.0"

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    "sphinx.ext.napoleon",  # Add google docstring support.
    "sphinx.ext.extlinks",  # Simplify linking to external documentation.
    "sphinx.ext.githubpages",  # Allow GitHub Pages rendering.
    "sphinx.ext.intersphinx",  # Link to other Sphinx documentation.
    "sphinx.ext.viewcode",  # Add a link to the Python source code for python objects.
    "myst_parser",  # Support for MyST Markdown syntax.
    "autoapi.extension",  # Generate API documentation from code.
    "sphinx.ext.autodoc",  # Include documentation from docstrings.
]
autoapi_dirs = [
    "../../src/panda_gazebo",
    "../../nodes",
    "../../scripts",
    str(
        catkin_dir.joinpath(
            "../../../devel/lib/python3/dist-packages/panda_gazebo"
        ).resolve()
    ),
]

# Extensions settings
autoapi_member_order = "bysource"
autoapi_options = [
    "members",
    "undoc-members",
    "show-inheritance",
    "show-module-summary",
    "imported-members",
]
autoapi_template_dir = "_templates/autoapi"

# Add mappings
intersphinx_mapping = {
    "python3": ("https://docs.python.org/3", None),
    "numpy": ("https://numpy.org/doc/stable", None),
}

# Add any paths that contain templates here, relative to this directory.
templates_path = ["_templates"]

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This patterns also effect to html_static_path and html_extra_path
exclude_patterns = []

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
html_theme = "sphinx_rtd_theme"

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ["_static"]
html_logo = "images/logo.svg"
html_favicon = "_static/favicon.ico"

# Theme options are theme-specific and customize the look and feel of a theme
# further.  For a list of options available for each theme, see the
# documentation.
html_theme_options = {"logo_only": True}
html_context = {
    "display_github": True,  # Add 'Edit on Github' link instead of 'View page source'
    "github_user": "rickstaa",
    "github_repo": "panda-gazebo",
    "github_version": "noetic",
    "conf_py_path": "/panda_gazebo/docs/source/",  # needs leading and trailing slashes!
}

# -- External links dictionary -----------------------------------------------
# Here you will find some often used global url definitions.
extlinks = {
    "ros_gazebo_gym": (
        "https://github.com/rickstaa/ros-gazebo-gym/tree/noetic/%s",
        None,
    ),
    "panda_gazebo": ("https://github.com/rickstaa/panda-gazebo/%s", None),
    "control_msgs": (
        "https://docs.ros.org/api/control_msgs/%s",
        None,
    ),
    "controller_manager_msgs": (
        "https://docs.ros.org/api/controller_manager_msgs/%s",
        None,
    ),
}


# -- Add extra style sheets --------------------------------------------------
def setup(app):
    app.add_css_file("css/modify.css")
