# Copyright (c) 2026 Justin Kleiber

import numpy as np
from units import INCHES_TO_METERS, angular_velocity_to_linear_velocity

# Gravity in m/s^2
GRAVITY = -9.8 

# Height of target in meters
TARGET_HEIGHT = 1.83 # 72 inches

# Height of the launch point in meters
LAUNCH_HEIGHT = 0.6096 # 24 inches

# Launch angle in degrees.
MIN_LAUNCH_ANGLE = 1.0
MAX_LAUNCH_ANGLE = 89.0

# Diameter of flywheel in inches
FLYWHEEL_DIAMETER = 4.0

# Flywheel RPM constraints and related launch velocity constraint defaults
FLYWHEEL_MIN_RPM = 100.0
FLYWHEEL_MAX_RPM = 5000.0
MIN_LAUNCH_VELOCITY = angular_velocity_to_linear_velocity(FLYWHEEL_MIN_RPM, FLYWHEEL_DIAMETER)
MAX_LAUNCH_VELOCITY = angular_velocity_to_linear_velocity(FLYWHEEL_MAX_RPM, FLYWHEEL_DIAMETER)

# Minimum launch distance in meters.
MIN_LAUNCH_DISTANCE = (18.0 + 23.5) * INCHES_TO_METERS
MAX_LAUNCH_DISTANCE = 8.0 

# Shallowest allowed arrival angle
SHALLOWEST_ARRIVAL_ANGLE = np.radians(-30.0)
TARGET_ARRIVAL_ANGLE = np.radians(-60.0)