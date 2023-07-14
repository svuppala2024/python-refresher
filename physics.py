"""
    Created by Suhruth Vuppala on 07-13-23
"""

import numpy as np

density_water = 1000  # density of water in kg / m^3
g = 9.81  # acceleration of gravity in m / s^2


def calculate_buoyancy(V: float, density_fluid: float):
    """
    This function calculates the buoyancy force exerted on an object submerged in water.

    Parameters:
        V : the volume of the object in cubic meters
        density_fluid : the density of the fluid in kg / m^3

    Returns:
        The buoyancy force in Newtons (float)

    """

    if V <= 0 or density_fluid <= 0:
        raise ValueError("The volume and the density cannot be negative")

    return V * density_fluid * g


def will_it_float(V: float, mass: float):
    """
    This function determines whether an object will float or sink in water.

    Parameters:
        V : the volume of the object in cubic meters
        mass : the mass of the object in kg

    Returns:
        True, if the object will float
        False, if the object will sink

    """

    if V <= 0 or mass <= 0:
        raise ValueError("The volume and the mass cannot be negative or zero")

    return mass / V <= density_water


def calculate_pressure(depth: float):
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


def calculate_acceleration(F: float, m: float):
    """
    This function calcuulates the acceleration of an object given the force applied to it and its mass

    Parameters:
        F : the force applied to the object in Newtons
        m : the mass of the object in kilograms

    Returns:
        The acceleration in m / s^2 (float)
    """

    if m <= 0:
        raise ValueError("Please use a positive mass")

    return F / m


def calculate_angular_acceleration(tau: float, I: float):
    """
    This function calculates the angular acceleration of an object given the torque applied to it and its moment of inertia

    Parameters:
        tau : the torque applied to the object in Newton-meters
        I: the moment of inertia of the object in kg * m^2

    Returns:
        The angular acceleration in m / s^2 (float)
    """

    if I <= 0:
        raise ValueError("Please use a positive moment of inertia")

    return tau / I


def calculate_torque(F_magnitude: float, F_direction: float, r: float):
    """
    This function calculates the torque applied to an object given the force applied to it and
    the distance from the axis of rotation to the point where the force is applied

    Parameters:
        F_magnitude : the magnitude of force applied to the object in Newtons
        F_direction : the direction of the force applied to the object in degrees (only positive)
        r : the distance from the axis of rotation to the point where the force is applied in meters

    Returns:
        The torque in Newton-meters (float)
    """

    if r <= 0 or F_magnitude <= 0:
        raise ValueError(
            "Please use positive values for radius and the magnitude of the force."
        )

    return r * F_magnitude * np.cos(np.deg2rad(F_direction))


def calculate_moment_of_inertia(m: float, r: float):
    """
    This function calculates the moment of inertia of an object given its mass and the distance
    from the axis of rotation to the center of mass of the object

    Parameters:
        m : the mass of the object in kilograms,
        r : the distance from the axis of rotation to the center of mass of the object in meters

    Returns:
        The moment of inertia in kg * m^2 (float)
    """

    if m <= 0 or r <= 0:
        raise ValueError("Please use positive values for mass and radius")

    return m * r**2


def calculate_auv_acceleration(
    F_magnitude: float, F_angle: float, mass=100.0, volume=0.1, thruster_distance=0.5
):
    """
    This function calculates the acceleration of the AUV in the 2D plane.

    Parameters:
        F_magnitude: the magnitude of force applied by the thruster in Newtons.
        F_angle : the angle of the force applied by the thruster in radians. The angle is measured from the x-axis. Positive angles are measured in the counter-clockwise direction.
        mass (optional) : the mass of the AUV in kilograms. The default value is 100kg.
        volume (optional) : the volume of the AUV in cubic meters. The default value is 0.1 m^3
        thruster_distance (optional) : the distance from the center of mass of the AUV to the thruster in meters. The default value is 0.5m

    Returns:
        The acceleration of the AUV in m / s^2 (Float)
    """

    if F_magnitude > 100:
        raise ValueError("The AUV can only apply a positive force up to 100 N")
    if F_angle > 30 or F_angle < -30:
        raise ValueError(
            "The thuster can only angle up to 30 degrees in either direction."
        )
    if F_magnitude < 0 or mass <= 0 or volume <= 0 or thruster_distance < 0:
        raise ValueError(
            "The force, mass, volume, and thruster distance should be positive."
        )

    return F_magnitude * np.cos(np.deg2rad(F_angle)) / mass


def calculate_auv_angular_acceleration(
    F_magnitude: float, F_angle: float, inertia=1.0, thruster_distance=0.5
):
    """
    This function calculates the angular acceleration of the AUV

    Parameters:
        F_magnitude: the magnitude of force applied by the thruster in Newtons.
        F_angle: the angle of the force applied by the thruster in radians.
        inertia (optional) : the moment of inertia of the AUV in kg * m^2. The default value is 1 kg * m^2.
        thruster_distance (optional) : the distance from the center of mass of the AUV to the thruster in meters. The default value is 0.5m.

    Returns:
        the angular acceleration of the AUV in radians per second squared (float)

    """
    if F_magnitude > 100:
        raise ValueError("The AUV can only apply a positive force up to 100 N")
    if F_angle > 30 or F_angle < -30:
        raise ValueError(
            "The thuster can only angle up to 30 degrees in either direction."
        )
    if F_magnitude < 0 or inertia <= 0 or thruster_distance < 0:
        raise ValueError(
            "The force, moment of inertia, and thruster distance should be positive."
        )

    return thruster_distance * F_magnitude * np.cos(np.deg2rad(F_angle)) / inertia
