# Copyright (c) 2026 Justin Kleiber

import numpy as np

INCHES_TO_METERS = 0.0254
METERS_TO_FEET = 3.28084 
FEET_TO_METERS = 1.0 / METERS_TO_FEET

def linear_velocity_to_angular_velocity(linear_velocity: float, wheel_diameter: float):
    """
    Inputs:
    - linear velocity in meters/second
    - wheel diameter in inches

    Output:
    - Angular velocity in rotations per minute.
    """
    wheel_diameter_meters = wheel_diameter * INCHES_TO_METERS
    wheel_circumference = np.pi * wheel_diameter_meters

    angular_velocity_rpm = (linear_velocity * 60.0) / wheel_circumference
    return angular_velocity_rpm

def angular_velocity_to_linear_velocity(angular_velocity: float, wheel_diameter: float):
    """
    Inputs:
    - angular velocity in rotations per minute
    - wheel diameter in inches

    Output:
    - Linear velocity in meters per second.
    """
    wheel_diameter_meters = wheel_diameter * INCHES_TO_METERS
    wheel_circumference = np.pi * wheel_diameter_meters

    linear_velocity = (angular_velocity / 60.0) * wheel_circumference
    return linear_velocity
