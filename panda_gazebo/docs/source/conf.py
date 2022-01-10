# Configuration file for the Sphinx documentation builder.

# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html


# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.

# -- Make sure panda-gazebo is accessible without going through setup.py -----------
import os.path as osp
import sys
from datetime import datetime

sys.path.insert(0, osp.abspath("../../src/panda_gazebo"))
sys.path.insert(1, osp.abspath("../../nodes"))

# -- General configuration ------------------------------------------------

# If your documentation needs a minimal Sphinx version, state it here.
needs_sphinx = "3.0"

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    "sphinx.ext.napoleon",
    "sphinx.ext.extlinks",
    "sphinx.ext.githubpages",
    "sphinx.ext.intersphinx",
    "sphinx.ext.viewcode",
    "myst_parser",
]

# Extension settings
autosummary_generate = True
autosummary_generate_overwrite = True
autodoc_member_order = "bysource"
# autosummary_imported_members = True

# Add mappings
intersphinx_mapping = {
    "python3": ("https://docs.python.org/3", None),
    "numpy": ("http://docs.scipy.org/doc/numpy", None),
}

# Add any paths that contain templates here, relative to this directory.
templates_path = ["_templates"]

# Map suffix(es) to parsers
source_suffix = {".rst": "restructuredtext", ".txt": "markdown", ".md": "markdown"}

# The master toctree document.
master_doc = "index"

# General information about the project.
project = "panda-gazebo"
copyright = f"{datetime.now().year}, Rick Staa"
author = "Rick Staa"
git_user_name = "rickstaa"

# The version info for the project you're documenting, acts as replacement for
# |version| and |release|, also used in various other places throughout the
# built documents.
#
# The short X.Y version.
version = "2.7.7"

# The full version, including alpha/beta/rc tags.
release = "2.7.7"

# The language for content autogenerated by Sphinx. Refer to documentation
# for a list of supported languages.
#
# This is also used if you do content translation via gettext catalogues.
# Usually you set "language" from the command line for these cases.
language = None

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This patterns also effect to html_static_path and html_extra_path
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store", "TODO.*", "README.*"]

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = "default"  # 'sphinx'

# -- Options for HTML output ----------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
html_theme = "sphinx_rtd_theme"

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ["_static"]

html_context = {
    "display_github": True,  # Add 'Edit on Github' link instead of 'View page source'
    "github_user": git_user_name,
    "github_repo": project,
    "github_version": "noetic-devel",
    "conf_py_path": "/docs/source/",  # needs leading and trailing slashes!
}

# -- External links dictionary -----------------------------------------------
# Here you will find some often used global url definitions.
extlinks = {
    "panda-gazebo": ("https://github.com/rickstaa/panda-gazebo/%s", None),
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
