"""
    Created by Suhruth Vuppala on 07-13-23
"""

import numpy as np
import matplotlib.pyplot as plt

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
        F_angle : the angle of the force applied by the thruster in radians.
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


def calculate_auv2_acceleration(T: np.ndarray, alpha: float, theta: float, mass=100):
    """
    This function calculates the acceleration of the AUV in the 2D plane.

    Parameters:
        T : an np.ndarray of the magnitudes of the forces applied by the thrusters in Newtons.
        alpha : the angle of the thrusters in radians.
        theta : the angle of the auv in radians
        mass (optional) : the mass of the AUV in kilograms. The default value is 100 kg.

    Returns:
        The acceleration of the AUV in m / s^2 (np.ndarray)
    """

    if np.shape(T) != (4,):
        raise ValueError("Please provide forces for exactly 4 thrusters")

    X = np.array([np.cos(alpha), np.cos(alpha), -1 * np.cos(alpha), -1 * np.cos(alpha)])
    Y = np.array([np.sin(alpha), -1 * np.sin(alpha), -1 * np.sin(alpha), np.sin(alpha)])
    components = np.matrix([X, Y])
    rotation_matrix = np.matrix(
        [[np.cos(theta), -1 * np.sin(theta)], [np.sin(theta), np.cos(theta)]]
    )
    forces = np.matmul(rotation_matrix, components)
    forces = np.matmul(forces, T)
    acc = np.divide(forces, mass)
    return np.round(acc, 3)


def calculate_auv2_angular_acceleration(
    T: np.ndarray, alpha: float, L: float, l: float, inertia=100
):
    """
    This function calculates the angular acceleration of the AUV

    Parameters:
        T : an np.ndarray of the magnitudes of the forces applied by the thrusters in Newtons.
        alpha: the angle of the thrusters in radians.
        L : the distance from the center of mass of the AUV to the thrusters in meters.
        l : the distance from the center of mass of the AUV to the thrusters in meters.
        inertia (optional) : the moment of inertia of the AUV in kg * m^2. The default value is 100 kg * m^2.

    Returns:
        The angular acceleration of the AUV in rad / s^2
    """

    if np.shape(T) != (4,):
        raise ValueError("Please provide forces for exactly 4 thrusters")
    if inertia <= 0:
        raise ValueError("The moment of inertia of the AUV has to be greater than 0")
    if L <= 0 or l <= 0:
        raise ValueError(
            "The distance from the center of mass of the AUV to the thrusters has to be positive."
        )
    force = (T[0] - T[1] + T[2] - T[3]) * (L * np.cos(alpha) + l * np.sin(alpha))
    return force / inertia


def simulate_auv2_motion(
    T: np.ndarray,
    alpha: float,
    L: float,
    l: float,
    inertia=100.0,
    dt=0.1,
    t_final=10.0,
    x0=0.0,
    y0=0.0,
    theta0=0.0,
):
    """
    This function simulates the motion of the AUV in 2D plane

    Parameters:
        T: an np.ndarray of the magnitudes of the forces applied by the thrusters in Newtons.
        alpha: the angle of the thrusters in radians.
        L: the distance from the center of mass of the AUV to the thrusters in meters.
        l: the distance from the center of mass of the AUV to the thrusters in meters.
        inertia (optional): the moment of inertia of the AUV in kg * m^2. THe default value is 100 kg * m^2
        dt (optional): the time step of the simulation in seconds. The default value is 0.1s
        t_final (optional): the final time of the simulation in seconds. The default value is 10s
        x0 (optional): the initial x-position of the AUV in meters. The default value is 0m.
        y0 (optional): the initial y-position of the AUV in meters. The default value is 0m.
        theta0 (optional): the initial angle of the AUV in radians. The default value is 0rad

    Returns:
        t: an np.ndarray of the time steps of the simulation in seconds.
        x: an np.ndarray of the x-positions of the AUV in meters.
        y: an np.ndarray of the y-positions of the AUV in meters.
        theta: an np.ndarray of the angles of the AUV in radians.
        v: an np.ndarray of the velocities of the AUV in meters per second.
        omega: an np.ndarray of the angular velocities of the AUV in radians per second.
        a: an np.ndarray of the accelerations of the AUV in meters per second squared.
    """

    if T.shape != (4,):
        raise ValueError("Please provide forces for exactly 4 thrusters.")
    if L <= 0 or l <= 0 or inertia <= 0 or dt <= 0 or t_final <= 0:
        raise ValueError(
            "Please use positive values for lengths, inertia, time steps, and final time."
        )

    t = np.arange(0, t_final, dt)
    angular_acceleration = calculate_auv2_angular_acceleration(T, alpha, L, l, inertia)
    omega = np.zeros_like(t)
    theta = np.zeros_like(t)
    theta[0] = theta0
    a = np.ndarray((theta.size, 2))
    a[0] = calculate_auv2_acceleration(T, alpha, theta0)[0]
    v = np.ndarray((theta.size, 2))
    v[0] = np.array([0, 0])
    x = np.zeros_like(theta)
    y = np.zeros_like(theta)
    x[0] = x0
    y[0] = y0
    for i in range(1, theta.size):
        omega[i] = omega[i - 1] + angular_acceleration * dt
        theta[i] = theta[i - 1] + omega[i] * dt
        a[i] = calculate_auv2_acceleration(T, alpha, theta[i - 1])
        v[i] = v[i - 1] + a[i] * dt
        x[i] = x[i - 1] + v[i][0] * dt
        y[i] = y[i - 1] + v[i][1] * dt

    return (t, x, y, theta, v, omega, a)


def plot_auv2_motion(
    t: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    theta: np.ndarray,
    v: np.ndarray,
    omega: np.ndarray,
    a: np.ndarray,
):
    """
    This function plots the motion of the AUV in the 2D plane.

    Parameters:
        t: an np.ndarray of the time steps of the simulation in seconds.
        x: an np.ndarray of the x-positions of the AUV in meters.
        y: an np.ndarray of the y-positions of the AUV in meters.
        theta: an np.ndarray of the angles of the AUV in radians.
        v: an np.ndarray of the velocities of the AUV in meters per second.
        omega: an np.ndarray of the angular velocities of the AUV in radians per second.
        a: an np.ndarray of the accelerations of the AUV in meters per second squared.
    """

    plt.plot(t, x)
    plt.plot(t, y)
    pass


print(simulate_auv2_motion(np.array([40, 60, 80, 100]), np.pi / 3, 3, 2))
