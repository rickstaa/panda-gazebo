"""Small script that calculates the inertia matrix for common shapes. See
`this wiki <https://en.wikipedia.org/wiki/List_of_moments_of_inertia>`_ for more
information.
"""
import numpy as np


def cube_inertia(height, width, depth, mass):
    """Calculates the moment of inertia matrix for a cube.

    Args:
        height (float): The height of the cube [m].
        width (float): The width of the cube [m].
        depth (float): The length of the cube [m].
        mass (float): The mass of the cube [kg].

    Returns:
        numpy.ndarray: The inertia matrix.
    """

    return np.array(
        [
            [(1 / 12) * mass * (width ** 2 + depth ** 2), 0, 0],
            [0, (1 / 12) * mass * (depth ** 2 + height ** 2), 0],
            [0, 0, (1 / 12) * mass * (width ** 2 + height ** 2)],
        ]
    )


def cylinder_inertia(radius, height, mass):
    """Calculates the moment of inertia matrix for a cube.

    Args:
        radius (float): The radius of the cylinder [m].
        height (float): The height of the cylinder [m].
        mass (float): The mass of the cylinder [kg].

    Returns:
        numpy.ndarray: The inertia matrix.
    """

    return np.array(
        [
            [(1 / 12) * mass * (3 * radius ** 2 + height ** 2), 0, 0],
            [0, (1 / 12) * mass * (3 * radius ** 2 + height ** 2), 0],
            [0, 0, (1 / 2) * mass * radius ** 2],
        ]
    )


if __name__ == "__main__":

    # Inertia matrixes
    I_cube = cube_inertia(height=0.06, width=0.06, depth=0.06, mass=0.25)
    I_cylinder = cylinder_inertia(radius=0.025, height=0.02, mass=0.10)
    print("Cube:")
    print(I_cube)
    print("Cylinder")
    print(I_cylinder)
