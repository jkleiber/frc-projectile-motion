

import lmfit
import numpy as np


from projectile import compute_projectile_motion
from optimizer_utils import objective_fn, compute_error, ProjectileConstraints, TargetInfo


def lmfit_objective_fn(opt_params, *args):
    # Unpack params
    parvals = opt_params.valuesdict()
    v0 = parvals['v0']
    launch_angle = parvals['launch_angle']
    
    return objective_fn([v0, launch_angle], *args)


def lmfit_main(constraints: ProjectileConstraints, target_info: TargetInfo):
    print("LMFIT")
    
    v0 = (constraints.launch_velocity.max + constraints.launch_velocity.min) / 2.0
    launch_angle = (constraints.launch_angle.max + constraints.launch_angle.min) / 2.0

    min_distance = constraints.distance.min
    max_distance = constraints.distance.max

    delta_y = target_info.delta_height
    target_arrival_angle = target_info.arrival_angle

    distances = np.linspace(min_distance, max_distance, 50)
    for target_distance in distances:
        params = lmfit.Parameters()
        params.add('v0', value=v0, min=constraints.launch_velocity.min, max=constraints.launch_velocity.max)
        params.add('launch_angle', value=launch_angle, min=constraints.launch_angle.min, max=constraints.launch_angle.max)
        result = lmfit.minimize(lmfit_objective_fn, params, method='nelder',
                                args=(delta_y, target_distance, target_arrival_angle))

        opt_v0 = result.params['v0'].value
        opt_angle = result.params['launch_angle'].value
        distance, tof, arrival_angle = compute_projectile_motion([opt_v0, opt_angle], delta_y)
        error = compute_error([distance, arrival_angle], [target_distance, target_arrival_angle])

        print(f"Distance {target_distance:.3f} m optimal shot: v0={opt_v0:.3f} m/s, theta={
              np.degrees(opt_angle):.3f} deg --> Error: {error[0]:.4f} m, {np.degrees(error[1]):.3f} deg")
