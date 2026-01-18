
import numpy as np
from scipy.optimize import NonlinearConstraint, minimize, Bounds
from projectile import compute_projectile_motion
from optimizer_utils import compute_error
from constants import (
    LAUNCH_HEIGHT,
    TARGET_HEIGHT,
    MIN_LAUNCH_DISTANCE,
    IDEAL_ARRIVAL_ANGLE,
    MIN_LAUNCH_VELOCITY,
    MAX_LAUNCH_VELOCITY, 
    MIN_LAUNCH_ANGLE, 
    MAX_LAUNCH_ANGLE
)

def scipy_objective_fn(opt_params, *args):
    """
    Args:
    - opt_params: Optimization parameters
    - args:
        - 0: delta_y
        - 1: target_distance
        - 2: desired arrival angle
    """
    # Unpack params
    v0 = opt_params[0]
    launch_angle = opt_params[1]

    # Unpack *args
    delta_y = args[0]
    target_distance = args[1]
    target_arrival_angle = args[2]

    distance, tof, arrival_angle = compute_projectile_motion([v0, launch_angle], delta_y)
    error = compute_error([distance, arrival_angle], [target_distance, target_arrival_angle])

    dist_error = error[0]
    angle_error = error[1]

    return 1000.0*dist_error**2 + 500.0*angle_error**2

def scipy_optimize_main():
    print("SCIPY")

    v0 = (MAX_LAUNCH_VELOCITY + MIN_LAUNCH_VELOCITY) / 2.0
    launch_angle = (MAX_LAUNCH_ANGLE + MIN_LAUNCH_ANGLE) / 2.0

    launch_height = LAUNCH_HEIGHT
    target_height = TARGET_HEIGHT
    min_distance = MIN_LAUNCH_DISTANCE
    max_distance = 8.0  # meters

    delta_y = target_height - launch_height
    target_arrival_angle = IDEAL_ARRIVAL_ANGLE

    bounds = Bounds([MIN_LAUNCH_VELOCITY, MIN_LAUNCH_ANGLE], [MAX_LAUNCH_VELOCITY, MAX_LAUNCH_ANGLE])

    distances = np.linspace(min_distance, max_distance, 50)
    for target_distance in distances:
        x0 = [v0, launch_angle]
        result = minimize(scipy_objective_fn, x0, args=(delta_y, target_distance, target_arrival_angle), method='nelder-mead', bounds=bounds)

        opt_v0 = result.x[0]
        opt_angle = result.x[1]
        distance, tof, arrival_angle = compute_projectile_motion([opt_v0, opt_angle], delta_y)
        error = compute_error([distance, arrival_angle], [target_distance, target_arrival_angle])

        print(f"Distance {target_distance:.3f} m optimal shot: v0={opt_v0:.3f} m/s, theta={
              np.degrees(opt_angle):.3f} deg --> Error: {error[0]:.4f} m, {np.degrees(error[1]):.3f} deg")