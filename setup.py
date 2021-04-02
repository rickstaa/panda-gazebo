"""A setup.py file for the 'panda_openai_sim' ros package. This setupfile is used by
catkin to add all of the python scripts and modules to the devel and install spaces.
Additionally it can be used to add extra python dependencies that can not be added
to the package.xml. For example when we need a specific version of a python module.
"""

# Standard library imports
import logging
import os
from setuptools import setup, find_packages
import sys
import re
from distutils.sysconfig import get_python_lib

# Get the relative path for including (data) files with the package
relative_site_packages = get_python_lib().split(sys.prefix + os.sep)[1]
date_files_relative_path = os.path.join(relative_site_packages, "panda_openai_sim")

# Additional python requirements that could not be specified in the package.xml
requirements = ["rospkg", "netifaces"]

# Set up logger
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

#################################################
# Setup script ##################################
#################################################

# Get current package version
__version__ = re.sub(
    r"[^\d.]",
    "",
    open(
        os.path.join(os.path.dirname(os.path.realpath(__file__)), "version.py")
    ).read(),
)

# Parse readme.md
with open("README.md") as f:
    readme = f.read()

# Run python setup
setup(
    name="panda_openai_sim",
    version=__version__,
    description=(
        "A ROS package that creates an Openai gym environment for the Panda Emika "
        "Franka robot."
    ),
    long_description=readme,
    long_description_content_type="text/markdown",
    author="Rick Staa",
    author_email="rick.staa@outlook.com",
    license="Rick Staa copyright",
    url="https://github.com/rickstaa/panda_openai_sim",
    keywords="ros, rl, panda, openai gym",
    classifiers=[
        "Programming Language :: Python :: 3.5",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Natural Language :: English",
        "Topic :: Scientific/Engineering",
    ],
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=requirements,
    extras_require={
        "docs": [
            "sphinx",
            "sphinxcontrib-napoleon",
            "sphinx_rtd_theme",
            "sphinx-navtree",
        ],
        "dev": ["bumpversion", "flake8"],  # IMPROVE: Add black if switched to Noetic
    },
    include_package_data=True,
    data_files=[(date_files_relative_path, ["README.md"])],
)
