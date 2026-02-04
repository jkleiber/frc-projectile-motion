# Copyright (c) 2026 Justin Kleiber

import lmfit
from optimizer_utils import objective_fn
from optimizer_types import ProjectileMotionConstraints, TargetInfo
from projectile import Projectile

def lmfit_objective_fn(opt_params, *args):
    # Unpack params
    parvals = opt_params.valuesdict()
    flywheel_rps = parvals['flywheel_rps']
    launch_angle = parvals['launch_angle']
    
    return objective_fn([flywheel_rps, launch_angle], *args)


def lmfit_main(constraints: ProjectileMotionConstraints, target_info: TargetInfo, projectile: Projectile, flywheel_diameter: float):
    flywheel_rps = (constraints.flywheel_rps.max + constraints.flywheel_rps.min) / 2.0
    launch_angle = (constraints.launch_angle.max + constraints.launch_angle.min) / 2.0

    params = lmfit.Parameters()
    params.add('flywheel_rps', value=flywheel_rps, min=constraints.flywheel_rps.min, max=constraints.flywheel_rps.max)
    params.add('launch_angle', value=launch_angle, min=constraints.launch_angle.min, max=constraints.launch_angle.max)
    result = lmfit.minimize(lmfit_objective_fn, params, method='nelder',
                            args=(target_info.delta_height, target_info.distance, target_info.arrival_angle, projectile, flywheel_diameter))

    opt_rps = result.params['flywheel_rps'].value
    opt_angle = result.params['launch_angle'].value

    return opt_rps, opt_angle