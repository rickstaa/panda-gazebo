# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
import os
import sys
import sphinx_rtd_theme

# -- Add module, and scripts paths to the system path --
# sys.path.insert(0, os.path.join(os.path.abspath(os.path.dirname(__name__)), "../../"))
# sys.path.insert(0, os.path.join(os.path.abspath(os.path.dirname(__name__)), ".."))
sys.path.insert(
    0,
    os.path.join(os.path.abspath(os.path.dirname(__name__)), "../panda_openai_sim/src"),
)
sys.path.insert(
    0,
    os.path.abspath(
        os.path.join(os.path.dirname(__name__), "../../panda_openai_sim/nodes")
    ),
)
sys.path.insert(
    0,
    os.path.abspath(
        os.path.join(os.path.dirname(__name__), "../../panda_training/scripts")
    ),
)
sys.path.insert(
    0,
    os.path.join(os.path.abspath(os.path.dirname(__name__)), "../panda_training"),
)

# -- Project information -----------------------------------------------------
project = "gazebo-panda-gym"
copyright = "2020, Rick Staa"
author = "Rick Staa"

# The full version, including alpha/beta/rc tags
release = "0.0.0"


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.

# Extensions
extensions = [
    "sphinx.ext.napoleon",
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    "sphinx.ext.intersphinx",
    "sphinx.ext.viewcode",
    "sphinx.ext.githubpages",
    "sphinx.ext.extlinks",
]

# Extension settings
autoclass_content = "class"
autodoc_member_order = "bysource"
autodoc_default_options = {
    "show-inheritance": True,
    "private-members": True,
}
napoleon_include_special_with_doc = True
napoleon_include_init_with_doc = True
autosummary_generate = True

# Add mappings
intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
    "numpy": ("https://numpy.org/doc/stable/", None),
}

# Add any paths that contain templates here, relative to this directory.
templates_path = ["_templates"]

# The suffix(es) of source filenames.
# You can specify multiple suffix as a list of string:
# source_suffix = [".rst", ".md"]
source_suffix = ".rst"

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []

# The master toctree document.
master_doc = "index"

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = "sphinx"

# If true, `todo` and `todoList` produce output, else they produce nothing.
todo_include_todos = False

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
html_theme = "sphinx_rtd_theme"
html_theme_path = [sphinx_rtd_theme.get_html_theme_path()]


# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ["_static"]
html_context = {
    "css_files": [
        "_static/theme_overrides.css"  # overrides for wide tables in RTD theme
    ]
}

# -- External links dictionary -----------------------------------------------
extlinks = {
    "gazebo-panda-gym": ("https://github.com/rickstaa/panda_openai_sim/%s", None),
    "panda_openai_sim": (
        "https://github.com/rickstaa/panda_openai_sim/tree/melodic-devel/panda_openai_sim/%s",
        None,
    ),
    "panda_training": (
        "https://github.com/rickstaa/panda_openai_sim/tree/melodic-devel/panda_training/%s",
        None,
    ),
    "env_config": (
        (
            "https://github.com/rickstaa/panda_openai_sim/blob/melodic-devel/"
            "panda_openai_sim/cfg/env_config.yaml/%s"
        ),
        None,
    ),
    "stable-baselines": ("https://stable-baselines.readthedocs.io/en/master/%s", None),
    "sphinx": ("http://www.sphinx-doc.org/en/master/%s", None),
    "ros": ("https://wiki.ros.org/%s", None),
    "gym": ("https://gym.openai.com/%s", None),
    "openai_ros": ("https://wiki.ros.org/openai_ros/%s", None),
    "franka": ("https://www.franka.de/%s", None),
    "geometry_msgs": ("https://docs.ros.org/api/geometry_msgs/%s", None),
    "visualization_msgs": ("https://docs.ros.org/api/visualization_msgs/%s", None),
    "gazebo_msgs": ("https://docs.ros.org/api/gazebo_msgs/%s", None),
    "control_msgs": (
        "https://docs.ros.org/api/control_msgs/%s",
        None,
    ),
    "controller_manager_msgs": (
        "https://docs.ros.org/api/controller_manager_msgs/%s",
        None,
    ),
}
