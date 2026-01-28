# Copyright (c) 2026 Justin Kleiber

from scipy.optimize import minimize, Bounds
from optimizer_utils import ProjectileMotionConstraints, TargetInfo
from projectile import Projectile
from collections.abc import Callable

def scipy_optimize_main(constraints: ProjectileMotionConstraints, target_info: TargetInfo, projectile: Projectile, flywheel_diameter: float, objective_fn: Callable):
    v0 = (constraints.launch_velocity.max + constraints.launch_velocity.min) / 2.0
    launch_angle = (constraints.launch_angle.max + constraints.launch_angle.min) / 2.0

    bounds = Bounds([constraints.launch_velocity.min, constraints.launch_angle.min], [
                    constraints.launch_velocity.max, constraints.launch_angle.max])

    x0 = [v0, launch_angle]
    result = minimize(objective_fn, x0, args=(target_info.delta_height, target_info.distance,
                      target_info.arrival_angle, projectile, flywheel_diameter), method='nelder-mead', bounds=bounds)

    opt_v0 = result.x[0]
    opt_angle = result.x[1]

    return opt_v0, opt_angle
