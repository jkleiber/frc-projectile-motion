# Copyright (c) 2026 Justin Kleiber

import lmfit
from optimizer_utils import objective_fn, ProjectileConstraints, TargetInfo


def lmfit_objective_fn(opt_params, *args):
    # Unpack params
    parvals = opt_params.valuesdict()
    v0 = parvals['v0']
    launch_angle = parvals['launch_angle']
    
    return objective_fn([v0, launch_angle], *args)


def lmfit_main(constraints: ProjectileConstraints, target_info: TargetInfo):
    v0 = (constraints.launch_velocity.max + constraints.launch_velocity.min) / 2.0
    launch_angle = (constraints.launch_angle.max + constraints.launch_angle.min) / 2.0

    params = lmfit.Parameters()
    params.add('v0', value=v0, min=constraints.launch_velocity.min, max=constraints.launch_velocity.max)
    params.add('launch_angle', value=launch_angle, min=constraints.launch_angle.min, max=constraints.launch_angle.max)
    result = lmfit.minimize(lmfit_objective_fn, params, method='nelder',
                            args=(target_info.delta_height, target_info.distance, target_info.arrival_angle))

    opt_v0 = result.params['v0'].value
    opt_angle = result.params['launch_angle'].value

    return opt_v0, opt_angle