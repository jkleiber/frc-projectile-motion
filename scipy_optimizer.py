
import numpy as np
from scipy.optimize import minimize, Bounds
from projectile import compute_projectile_motion
from optimizer_utils import objective_fn, compute_error, ProjectileConstraints, TargetInfo


def scipy_optimize_main(constraints: ProjectileConstraints, target_info: TargetInfo):
    print("SCIPY")

    v0 = (constraints.launch_velocity.max + constraints.launch_velocity.min) / 2.0
    launch_angle = (constraints.launch_angle.max + constraints.launch_angle.min) / 2.0

    min_distance = constraints.distance.min
    max_distance = constraints.distance.max

    delta_y = target_info.delta_height
    target_arrival_angle = target_info.arrival_angle

    bounds = Bounds([constraints.launch_velocity.min, constraints.launch_angle.min], [constraints.launch_velocity.max, constraints.launch_angle.max])

    distances = np.linspace(min_distance, max_distance, 50)
    for target_distance in distances:
        x0 = [v0, launch_angle]
        result = minimize(objective_fn, x0, args=(delta_y, target_distance, target_arrival_angle), method='nelder-mead', bounds=bounds)

        opt_v0 = result.x[0]
        opt_angle = result.x[1]
        distance, tof, arrival_angle = compute_projectile_motion([opt_v0, opt_angle], delta_y)
        error = compute_error([distance, arrival_angle], [target_distance, target_arrival_angle])

        print(f"Distance {target_distance:.3f} m optimal shot: v0={opt_v0:.3f} m/s, theta={
              np.degrees(opt_angle):.3f} deg --> Error: {error[0]:.4f} m, {np.degrees(error[1]):.3f} deg")