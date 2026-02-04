# Copyright (c) 2026 Justin Kleiber

from projectile_kinematics import compute_projectile_motion
from performance import compute_error

def objective_fn(opt_params, *args):
    """
    Args:
    - opt_params: Optimization parameters
    - args:
        - 0: delta_y
        - 1: target_distance
        - 2: desired arrival angle
    """
    # Unpack params
    flywheel_rps = opt_params[0]
    launch_angle = opt_params[1]

    # Unpack *args
    delta_y = args[0]
    target_distance = args[1]
    target_arrival_angle = args[2]
    projectile = args[3]
    flywheel_diameter = args[4]

    # The projectile has linear speed as a factor of the ratio between the flywheel diameter and the projectile diameter. 
    v0 = flywheel_rps * (flywheel_diameter / projectile.diameter)

    distance, tof, arrival_angle = compute_projectile_motion([v0, launch_angle], delta_y)
    error = compute_error([distance, arrival_angle], [target_distance, target_arrival_angle])

    dist_error = error[0]
    angle_error = error[1]

    return 1000.0*dist_error**2 + 1500.0*angle_error**2
