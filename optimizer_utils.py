
import numpy as np

from dataclasses import dataclass

from projectile import compute_projectile_motion

def compute_error(actual, target):
    actual_vec = np.array(actual)
    target_vec = np.array(target)

    return target_vec - actual_vec

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

@dataclass 
class Constraint:
    min: float
    max: float


@dataclass
class ProjectileConstraints:
    distance: Constraint
    launch_velocity: Constraint
    launch_angle: Constraint
    
@dataclass
class TargetInfo:
    delta_height: float
    arrival_angle: float