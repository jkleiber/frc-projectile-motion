

import lmfit
import numpy as np


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

def lmfit_objective_fn(opt_params, *args):
    """
    Args:
    - opt_params: Optimization parameters
    - args:
        - 0: delta_y
        - 1: target_distance
        - 2: desired arrival angle
    """
    # Unpack params
    parvals = opt_params.valuesdict()
    v0 = parvals['v0']
    launch_angle = parvals['launch_angle']

    # Unpack *args
    delta_y = args[0]
    target_distance = args[1]
    target_arrival_angle = args[2]

    distance, tof, arrival_angle = compute_projectile_motion([v0, launch_angle], delta_y)
    error = compute_error([distance, arrival_angle], [target_distance, target_arrival_angle])

    dist_error = error[0]
    angle_error = error[1]

    return 1000.0*dist_error**2 + 500.0*angle_error**2


def lmfit_main():
    print("LMFIT")
    
    launch_height = LAUNCH_HEIGHT
    target_height = TARGET_HEIGHT
    min_distance = MIN_LAUNCH_DISTANCE
    max_distance = 8.0  # meters

    delta_y = target_height - launch_height
    target_arrival_angle = IDEAL_ARRIVAL_ANGLE

    distances = np.linspace(min_distance, max_distance, 50)
    for target_distance in distances:
        params = lmfit.Parameters()
        params.add('v0', value=(MAX_LAUNCH_VELOCITY + MIN_LAUNCH_VELOCITY) /
                   2.0, min=MIN_LAUNCH_VELOCITY, max=MAX_LAUNCH_VELOCITY)
        params.add('launch_angle', value=(MAX_LAUNCH_ANGLE + MIN_LAUNCH_ANGLE) /
                   2.0, min=MIN_LAUNCH_ANGLE, max=MAX_LAUNCH_ANGLE)
        result = lmfit.minimize(lmfit_objective_fn, params, method='nelder',
                                args=(delta_y, target_distance, target_arrival_angle))

        opt_v0 = result.params['v0'].value
        opt_angle = result.params['launch_angle'].value
        distance, tof, arrival_angle = compute_projectile_motion([opt_v0, opt_angle], delta_y)
        error = compute_error([distance, arrival_angle], [target_distance, target_arrival_angle])

        print(f"Distance {target_distance:.3f} m optimal shot: v0={opt_v0:.3f} m/s, theta={
              np.degrees(opt_angle):.3f} deg --> Error: {error[0]:.4f} m, {np.degrees(error[1]):.3f} deg")
