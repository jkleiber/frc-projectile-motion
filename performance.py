# Copyright (c) 2026 Justin Kleiber

import numpy as np

from matplotlib import pyplot as plt
from matplotlib import patches

from units import METERS_TO_FEET, INCHES_TO_METERS, linear_velocity_to_angular_velocity
from constants import MAX_DISTANCE_ERROR, TARGET_HEIGHT
from projectile_kinematics import compute_projectile_motion


def compute_error(actual, target):
    actual_vec = np.array(actual)
    target_vec = np.array(target)

    return target_vec - actual_vec


def evaluate_kinematics_performance(result, target_info, projectile, flywheel_diameter):
    flywheel_v0 = result[0]
    opt_launch_angle = result[1]

    delta_y = target_info.delta_height
    target_arrival_angle = target_info.arrival_angle
    target_distance = target_info.distance

    projectile_v0 = flywheel_v0 * (flywheel_diameter / projectile.diameter)
    distance, tof, arrival_angle = compute_projectile_motion([projectile_v0, opt_launch_angle], delta_y)
    error = compute_error([distance, arrival_angle], [target_distance, target_arrival_angle])

    # Use the flywheel v0 for converting to RPM because the projectile is at a different speed.
    rpm = linear_velocity_to_angular_velocity(flywheel_v0, flywheel_diameter)

    print(f"Distance {target_distance*METERS_TO_FEET:.3f} ft - optimal shot: rps={rpm / 60.0:.3f} RPS, theta={
            np.degrees(opt_launch_angle):.3f} deg --> Error: {error[0]/INCHES_TO_METERS:.4f} in, Arrival: {np.degrees(arrival_angle):.3f} deg")
    

def evaluate_dynamics_performance(trajectory, target_height, target_distance):
    has_intercept_1 = False

    # Evaluation criteria
    shot_distance = 0.0
    arrival_angle = 0.0

    prev_height = trajectory[0,2]
    for point in trajectory:
        height = point[2]

        if height >= target_height and prev_height <= target_height:
            has_intercept_1 = True
        if has_intercept_1 and (height <= target_height and prev_height >= target_height):
            shot_distance = point[1]
            arrival_angle = np.arctan2(point[4], point[3])
            break

        prev_height = height

    return shot_distance, arrival_angle



def plot_projectile_trajectory(trajectory, target_distance):
    base_path = "plots"

    fig, ax = plt.subplots()
    ax.plot(trajectory[:,1] * METERS_TO_FEET, trajectory[:,2] * METERS_TO_FEET)
    target_center = target_distance * METERS_TO_FEET
    target_nearest_x = target_center - (MAX_DISTANCE_ERROR * METERS_TO_FEET)
    rect = patches.Rectangle((target_nearest_x, 0), 2.0*MAX_DISTANCE_ERROR * METERS_TO_FEET, TARGET_HEIGHT * METERS_TO_FEET, linewidth=2, edgecolor='red', facecolor='white')
    ax.add_patch(rect)
    plt.savefig(f'{base_path}/trajectory.png', bbox_inches='tight')