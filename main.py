# Copyright (c) 2026 Justin Kleiber

import argparse
import numpy as np
from lmfit_optimizer import lmfit_main
from scipy_optimizer import scipy_optimize_main

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
    PROJECTILE_DIAMETER
)
from optimizer_utils import Constraint, ProjectileMotionConstraints, TargetInfo, compute_error
from projectile import Projectile
from projectile_kinematics import compute_projectile_motion
from units import linear_velocity_to_angular_velocity, INCHES_TO_METERS, METERS_TO_FEET


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--optimizer", default='scipy')
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
    args = parser.parse_args()

    # Build constraints for optimization
    opt_constraints = ProjectileMotionConstraints(distance=Constraint(min=args.min_distance, max=args.max_distance),
                                            launch_velocity=Constraint(
                                                min=args.min_launch_velocity, max=args.max_launch_velocity),
                                            launch_angle=Constraint(min=np.radians(args.min_launch_angle), max=np.radians(args.max_launch_angle)))

    # Get target information.
    delta_height = args.target_height - args.launch_height
    target_info = TargetInfo(delta_height=delta_height, arrival_angle=args.target_arrival_angle, distance=0.0)

    projectile = Projectile(mass=0.0, diameter=args.projectile_diameter)

    optimizer_loop(opt_constraints, target_info, optimizer=args.optimizer, projectile=projectile, flywheel_diameter=args.flywheel_diameter)


def optimizer_loop(constraints: ProjectileMotionConstraints, target_info: TargetInfo, projectile: Projectile, optimizer='scipy', flywheel_diameter=FLYWHEEL_DIAMETER):
    min_distance = constraints.distance.min
    max_distance = constraints.distance.max

    delta_y = target_info.delta_height
    target_arrival_angle = target_info.arrival_angle

    distances = np.linspace(min_distance, max_distance, 50)
    for target_distance in distances:
        target_info.distance = target_distance
        
        if optimizer == 'lmfit':
            result = lmfit_main(constraints, target_info, projectile, flywheel_diameter)
        else:
            result = scipy_optimize_main(constraints, target_info, projectile, flywheel_diameter)


        flywheel_v0 = result[0]
        opt_launch_angle = result[1]
        projectile_v0 = flywheel_v0 * (flywheel_diameter / projectile.diameter)
        distance, tof, arrival_angle = compute_projectile_motion([projectile_v0, opt_launch_angle], delta_y)
        error = compute_error([distance, arrival_angle], [target_distance, target_arrival_angle])

        # Use the flywheel v0 for converting to RPM because the projectile is at a different speed.
        rpm = linear_velocity_to_angular_velocity(flywheel_v0, flywheel_diameter)

        print(f"Distance {target_distance*METERS_TO_FEET:.3f} ft - optimal shot: rps={rpm / 60.0:.3f} RPS, theta={
              np.degrees(opt_launch_angle):.3f} deg --> Error: {error[0]/INCHES_TO_METERS:.4f} in, Arrival: {np.degrees(arrival_angle):.3f} deg")

if __name__ == "__main__":
    main()
