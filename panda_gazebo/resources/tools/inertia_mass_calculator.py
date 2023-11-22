"""Small script that calculates the inertia matrix for common shapes. See
`this wiki <https://en.wikipedia.org/wiki/List_of_moments_of_inertia>`_ for more
information.
"""
import math

import numpy as np

# Script settings.
CUBE_WIDTH = 0.02
CUBE_DEPTH = 0.032
CUBE_HEIGHT = 0.064

CYLINDER_RADIUS = 0.025
CYLINDER_HEIGHT = 0.02

MATERIAL_DENSITY = 2000


def get_cylinder_mass(radius, height, rho):
    """Calculate the mass of a cylinder.

    Args:
        radius (float): The radius of the cylinder [m].
        height (float): The height of the cylinder [m].
        rho (float): The material density [kg/m^3].

    Returns:
        float: The mass [kg].
    """
    return rho * math.pi * radius**2 * height


def get_cube_inertia(height, width, depth, mass):
    """Calculates the moment of inertia matrix for a cube.

    Args:
        height (float): The height of the cube [m].
        width (float): The width of the cube [m].
        depth (float): The length of the cube [m].
        mass (float): The mass of the cube [kg].

    Returns:
        numpy.ndarray: The inertia matrix [kg*m^2].
    """
    return np.array(
        [
            [(1 / 12) * mass * (width**2 + depth**2), 0, 0],
            [0, (1 / 12) * mass * (depth**2 + height**2), 0],
            [0, 0, (1 / 12) * mass * (width**2 + height**2)],
        ]
    )


def get_cube_mass(height, width, depth, rho):
    """Calculate the mass of a cube.

    Args:
        radius (float): The radius of the cylinder [m].
        height (float): The height of the cylinder [m].
        depth (float): The depth of the cylinder [m].
        rho (float): The material density [kg/m^3].

    Returns:
        float: The mass [kg].
    """
    return rho * height * width * depth


def get_cylinder_inertia(radius, height, mass):
    """Calculates the moment of inertia matrix for a cube.

    Args:
        radius (float): The radius of the cylinder [m].
        height (float): The height of the cylinder [m].
        mass (float): The mass of the cylinder [kg].

    Returns:
        numpy.ndarray: The inertia matrix [kg*m^2].
    """
    return np.array(
        [
            [(1 / 12) * mass * (3 * radius**2 + height**2), 0, 0],
            [0, (1 / 12) * mass * (3 * radius**2 + height**2), 0],
            [0, 0, (1 / 2) * mass * radius**2],
        ]
    )


if __name__ == "__main__":
    # Cue calculations.
    print("==Cube==")
    cube_mass = get_cube_mass(
        height=CUBE_HEIGHT, width=CUBE_WIDTH, depth=CUBE_DEPTH, rho=MATERIAL_DENSITY
    )
    I_cube = get_cube_inertia(
        height=CUBE_HEIGHT, width=CUBE_WIDTH, depth=CUBE_DEPTH, mass=cube_mass
    )
    print(f"mass = {cube_mass}")
    print("inertia =")
    print(I_cube)
    print("\n")

    # Cylinder calculations.
    print("==Cylinder==")
    cylinder_mass = get_cylinder_mass(
        radius=CYLINDER_RADIUS, height=CYLINDER_HEIGHT, rho=MATERIAL_DENSITY
    )
    I_cylinder = get_cylinder_inertia(
        radius=CYLINDER_RADIUS, height=CYLINDER_HEIGHT, mass=cylinder_mass
    )
    print(f"mass = {cylinder_mass}")
    print("inertia =")
    print(I_cylinder)
