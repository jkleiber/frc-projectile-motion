
import argparse
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
    MAX_LAUNCH_ANGLE
)
from optimizer_utils import Constraint, ProjectileConstraints, TargetInfo


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--optimizer", default='scipy')
    parser.add_argument("--launch_height", default=LAUNCH_HEIGHT)
    parser.add_argument("--target_height", default=TARGET_HEIGHT)
    parser.add_argument("--target_arrival_angle", default=TARGET_ARRIVAL_ANGLE)
    parser.add_argument("--min_distance", default=MIN_LAUNCH_DISTANCE)
    parser.add_argument("--max_distance", default=MAX_LAUNCH_DISTANCE)
    parser.add_argument("--min_launch_velocity", default=MIN_LAUNCH_VELOCITY)
    parser.add_argument("--max_launch_velocity", default=MAX_LAUNCH_VELOCITY)
    parser.add_argument("--min_launch_angle", default=MIN_LAUNCH_ANGLE)
    parser.add_argument("--max_launch_angle", default=MAX_LAUNCH_ANGLE)
    args = parser.parse_args()

    # Build constraints for optimization
    opt_constraints = ProjectileConstraints(distance=Constraint(min=args.min_distance, max=args.max_distance),
                                            launch_velocity=Constraint(
                                                min=args.min_launch_velocity, max=args.max_launch_velocity),
                                            launch_angle=Constraint(min=args.min_launch_angle, max=args.max_launch_angle))

    # Get target information.
    delta_height = args.target_height - args.launch_height
    target_info = TargetInfo(delta_height=delta_height, arrival_angle=args.target_arrival_angle)

    if args.optimizer == 'lmfit':
        lmfit_main(opt_constraints, target_info)
    else:
        scipy_optimize_main(opt_constraints, target_info)


if __name__ == "__main__":
    main()
