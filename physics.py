"""
    Created by Suhruth Vuppala on 07-13-23
"""

density_water = 1000  # density of water in kg / m^3
g = 9.81  # acceleration of gravity in m / s^2


def calculate_buoyancy(V: float or int, density_fluid: float or int):
    """
    This function calculates the buoyancy force exerted on an object submerged in water.

    Parameters:
        V (float or int) : the volume of the object in cubic meters
        density_fluid (float or int) : the density of the fluid in kg / m^3

    Returns:
        The buoyancy force in Newtons (float)

    """

    if V < 0 or density_fluid < 0:
        raise ValueError("The volume and the density cannot be negative")

    return V * density_fluid * g


def will_it_float(V: float or int, mass: float or int):
    """
    This function determines whether an object will float or sink in water.

    Parameters:
        V (float or int) : the volume of the object in cubic meters
        mass (float or int) : the mass of the object in kg

    Returns:
        True, if the object will float
        False, if the object will sink

    """

    if V < 0 or mass < 0:
        raise ValueError("The volume and the mass cannot be negative")

    return mass / V <= density_water


def calculate_pressure(depth: float or int):
    """
    This function calculates the pressure at a given depth in water.

    Parameters:
        depth : the depth in meters.

    Returns:
        The pressure in Pascals (float)

    """

    surface_pressure = 101325

    if depth < 0:
        raise ValueError("Please refer to depth using a positive number")

    return depth * density_water * g + surface_pressure
