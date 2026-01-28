# Copyright (c) 2026 Justin Kleiber

import argparse
import numpy as np
from lmfit_optimizer import lmfit_main
from scipy_optimizer import scipy_optimize_main
from optimizer_utils import objective_fn

from constants import (
    LAUNCH_HEIGHT,
    TARGET_HEIGHT,
    TARGET_ARRIVAL_ANGLE,
    MIN_LAUNCH_DISTANCE,
    MAX_LAUNCH_DISTANCE,
    MIN_LAUNCH_VELOCITY,
    MAX_LAUNCH_VELOCITY,
    MIN_LAUNCH_ANGLE,
    MAX_LAUNCH_ANGLE,
    FLYWHEEL_DIAMETER,
    PROJECTILE_DIAMETER,
    PROJECTILE_MASS
)
from optimizer_utils import Constraint, ProjectileMotionConstraints, TargetInfo
from projectile import Projectile
import projectile_dynamics
from performance import evaluate_kinematics_performance, evaluate_dynamics_performance, plot_projectile_trajectory
from units import METERS_TO_FEET

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--optimizer", default='scipy')
    parser.add_argument("--sim", default='dynamics')
    parser.add_argument("--launch_height", type=float, default=LAUNCH_HEIGHT)
    parser.add_argument("--target_height", type=float, default=TARGET_HEIGHT)
    parser.add_argument("--target_arrival_angle", type=float, default=TARGET_ARRIVAL_ANGLE)
    parser.add_argument("--min_distance", type=float, default=MIN_LAUNCH_DISTANCE)
    parser.add_argument("--max_distance", type=float, default=MAX_LAUNCH_DISTANCE)
    parser.add_argument("--min_launch_velocity", type=float, default=MIN_LAUNCH_VELOCITY)
    parser.add_argument("--max_launch_velocity", type=float, default=MAX_LAUNCH_VELOCITY)
    parser.add_argument("--min_launch_angle", type=float, default=MIN_LAUNCH_ANGLE)
    parser.add_argument("--max_launch_angle", type=float, default=MAX_LAUNCH_ANGLE)
    parser.add_argument("--flywheel_diameter", type=float, default=FLYWHEEL_DIAMETER)
    parser.add_argument("--projectile_diameter", type=float, default=PROJECTILE_DIAMETER)
    parser.add_argument("--projectile_mass", type=float, default=PROJECTILE_MASS)
    args = parser.parse_args()

    # Build constraints for optimization
    opt_constraints = ProjectileMotionConstraints(distance=Constraint(min=args.min_distance, max=args.max_distance),
                                            launch_velocity=Constraint(
                                                min=args.min_launch_velocity, max=args.max_launch_velocity),
                                            launch_angle=Constraint(min=np.radians(args.min_launch_angle), max=np.radians(args.max_launch_angle)))

    # Get target information.
    delta_height = args.target_height - args.launch_height
    target_info = TargetInfo(delta_height=delta_height, arrival_angle=args.target_arrival_angle, distance=0.0, height=args.target_height)

    projectile = Projectile(mass=args.projectile_mass, diameter=args.projectile_diameter)

    optimizer_loop(opt_constraints, target_info, sim_type=args.sim, optimizer=args.optimizer, projectile=projectile, flywheel_diameter=args.flywheel_diameter)


def optimizer_loop(constraints: ProjectileMotionConstraints, target_info: TargetInfo, projectile: Projectile, sim_type='dynamics', optimizer='scipy', flywheel_diameter=FLYWHEEL_DIAMETER):
    min_distance = constraints.distance.min
    max_distance = constraints.distance.max

    distances = np.linspace(min_distance, max_distance, 50)
    for target_distance in distances:
        target_info.distance = target_distance
        
        if sim_type == 'kinematics':
            if optimizer == 'lmfit':
                result = lmfit_main(constraints, target_info, projectile, flywheel_diameter)
            else:
                result = scipy_optimize_main(constraints, target_info, projectile, flywheel_diameter, objective_fn)
        
            evaluate_kinematics_performance(result, target_info, projectile, flywheel_diameter)
        else:
            result = scipy_optimize_main(constraints, target_info, projectile, flywheel_diameter, projectile_dynamics.dynamics_objective_fn)
            trajectory = projectile_dynamics.compute_projectile_motion(result, projectile, flywheel_diameter)
            perf = evaluate_dynamics_performance(trajectory, target_info.height, target_info.distance)
            
            print(f"rps: {result[0]} rps, {np.degrees(result[1])} deg  -->  {perf[0] * METERS_TO_FEET} ft, {np.degrees(perf[1])} deg")
            # plot_projectile_trajectory(trajectory, target_info.distance)
        

if __name__ == "__main__":
    main()
