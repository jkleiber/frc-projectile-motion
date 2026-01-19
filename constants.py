
import numpy as np

INCHES_TO_METERS = 0.0254

# Gravity in m/s^2
GRAVITY = -9.8 

# Height of target in meters
TARGET_HEIGHT = 1.83 # 72 inches

# Height of the launch point in meters
LAUNCH_HEIGHT = 0.6096 # 24 inches

MIN_LAUNCH_ANGLE = np.radians(1.0)
MAX_LAUNCH_ANGLE = np.radians(89.0)

# Diameter of flywheel in meters
FLYWHEEL_DIAMETER = 4.0 * INCHES_TO_METERS
FLYWHEEL_MIN_RPM = 2000.0
FLYWHEEL_MAX_RPM = 5000.0
MIN_LAUNCH_VELOCITY = (FLYWHEEL_MIN_RPM / 60.0) * FLYWHEEL_DIAMETER * np.pi
MAX_LAUNCH_VELOCITY = (FLYWHEEL_MAX_RPM / 60.0) * FLYWHEEL_DIAMETER * np.pi

# Minimum launch distance in meters.
MIN_LAUNCH_DISTANCE = (18.0 + 23.5) * INCHES_TO_METERS
MAX_LAUNCH_DISTANCE = 8.0 

# Shallowest allowed arrival angle
SHALLOWEST_ARRIVAL_ANGLE = np.radians(-35.0)
TARGET_ARRIVAL_ANGLE = np.radians(-60.0)