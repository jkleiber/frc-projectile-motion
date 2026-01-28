# Copyright (c) 2026 Justin Kleiber

import numpy as np


from projectile import Projectile
from units import angular_velocity_to_linear_velocity
from constants import GRAVITY, LAUNCH_HEIGHT, FLYWHEEL_DIAMETER, TARGET_HEIGHT, PROJECTILE_DIAMETER
from units import INCHES_TO_METERS, METERS_TO_FEET, FEET_TO_METERS
from performance import compute_error, evaluate_dynamics_performance, plot_projectile_trajectory

from optimizer_utils import Constraint, ProjectileMotionConstraints, TargetInfo
from scipy_optimizer import scipy_optimize_main

def projectile_eom(t, state, projectile: Projectile):
    """
    Compute equations of motion for projectile.
    """
    x = state[0]
    z = state[1]
    vx = state[2]
    vz = state[3]
    omega_x = state[4]

    
    drag_coefficient = 0.47 # sphere
    lift_coefficient = 0.1 # guessed from NASA's baseball Cl estimate
    air_density = 1.225 # kg/m^3
    projectile_diameter_meters = projectile.diameter * INCHES_TO_METERS
    cross_sectional_area = (np.pi * projectile_diameter_meters**2) / 4.0
    projectile_radius_meters = projectile_diameter_meters / 2.0
    projectile_inertia = (2.0 / 5.0)*projectile.mass*projectile_radius_meters**2

    total_velocity = np.sqrt(vx**2 + vz**2)

    magnus_force = 0.5 * lift_coefficient * cross_sectional_area * air_density * (omega_x * total_velocity) * (projectile_diameter_meters * 0.5)
    drag_force = -0.5 * drag_coefficient * air_density * cross_sectional_area * total_velocity**2
    gravity_force = projectile.mass * GRAVITY

    angle_of_travel = np.arctan2(vz, vx)

    Fx = (magnus_force + drag_force) * np.cos(angle_of_travel)
    Fz = (magnus_force + drag_force) * np.sin(angle_of_travel) + gravity_force 
    torque = drag_force * projectile_radius_meters

    ax_dot = Fx / projectile.mass
    az_dot = Fz / projectile.mass
    
    # this is a rough approximation for reducing spin rate
    alpha_x = torque / projectile_inertia

    return np.array([
        vx, vz, ax_dot, az_dot, alpha_x
    ])



def compute_projectile_motion(opt_params, projectile: Projectile, flywheel_diameter: float):
    flywheel_rps = opt_params[0]
    launch_angle = opt_params[1]


    # Velocity initial conditions
    flywheel_linear_v0 = angular_velocity_to_linear_velocity(flywheel_rps * 60.0, flywheel_diameter)
    projectile_linear_v0 = flywheel_linear_v0 * (flywheel_diameter / projectile.diameter)
    vx_0 = projectile_linear_v0 * np.cos(launch_angle)
    vz_0 = projectile_linear_v0 * np.sin(launch_angle)

    # Convert RPS to rad/s.
    omega_0 = flywheel_rps * np.pi * 2.0 

    # Initialize state space
    x = np.array([
        0, LAUNCH_HEIGHT, vx_0, vz_0, omega_0
    ])

    # State space
    # x, z
    # v_x, v_z
    # w_x
    
    # x_dot = vx
    # vx_dot = [Fm_x + (Fd_x)] / m
    # z_dot = vz
    # vz_dot = [Fm_z + (g)] / m

    # Forces:
    # - Drag: Fd = -0.5 * cd * rho * A * v
    # - Gravity: g
    # - Magnus: Fm = S (w x v) = 0.5 * cl * A * rho * v^2 ( w x v )
    #
    # Variables:
    # cd: drag coefficient
    # rho: air density
    # A: cross sectional area
    # v: velocity

    # Forward euler
    dt = 0.001
    t = 0
    trajectory = []
    while x[1] > 0.0:
        xdot = projectile_eom(t, x, projectile)

        x = x + xdot * dt

        traj_entry = np.insert(x, 0, t)
        trajectory.append(traj_entry)

        t += dt

    return np.array(trajectory)



def dynamics_objective_fn(opt_params, *args):
    """
    Args:
    - opt_params: Optimization parameters
    - args:
        - 0: target_height
        - 1: target_distance
        - 2: desired arrival angle
    """
    # Unpack params
    flywheel_rps = opt_params[0]
    launch_angle = opt_params[1]

    # Unpack *args
    target_height = args[0]
    target_distance = args[1]
    target_arrival_angle = args[2]
    projectile = args[3]
    flywheel_diameter = args[4]

    trajectory = compute_projectile_motion([flywheel_rps, launch_angle], projectile=projectile, flywheel_diameter=flywheel_diameter)
    perf = evaluate_dynamics_performance(trajectory, target_height, target_distance)
    error = compute_error([perf[0], perf[1]], [target_distance, target_arrival_angle])

    dist_error = error[0]
    angle_error = error[1]

    print(f"x error: {dist_error}, angle error: {angle_error}")

    return 1000.0*dist_error**2 + 1500.0*angle_error**2




def projectile_opt_main(projectile: Projectile, target_info:TargetInfo, flywheel_diameter):
    constraints = ProjectileMotionConstraints(
        distance=Constraint(min=0.0, max=100.0),
        launch_velocity=Constraint(min=10.0, max=60.0),
        launch_angle=Constraint(min=np.radians(0.0), max=np.radians(89.9)))

    result = scipy_optimize_main(constraints, target_info, projectile, flywheel_diameter, dynamics_objective_fn)
    
    return result

def projectile_run_main(params, projectile: Projectile, target_info:TargetInfo, flywheel_diameter):
    trajectory = compute_projectile_motion(params, projectile, flywheel_diameter)
    perf = evaluate_dynamics_performance(trajectory, target_info.height, target_info.distance)
    
    print(f"rps: {params[0]} rps, {np.degrees(params[1])} deg  -->  {perf[0] * METERS_TO_FEET} ft, {np.degrees(perf[1])} deg")
    
    plot_projectile_trajectory(trajectory, target_info.distance)

def projectile_main():
    target_distance = 15.0 * FEET_TO_METERS
    params = [39.0, np.radians(66.0)]
    flywheel_diameter = FLYWHEEL_DIAMETER

    projectile = Projectile(mass=0.227, diameter=PROJECTILE_DIAMETER)
    
    target_info = TargetInfo(delta_height=0.0, arrival_angle=np.radians(-60), distance=target_distance, height=TARGET_HEIGHT)
    
    is_optimizing = False
    if is_optimizing:
        opt_params = projectile_opt_main(projectile, target_info, flywheel_diameter)
        projectile_run_main(opt_params, projectile, target_info, flywheel_diameter)
    else:
        projectile_run_main(params, projectile, target_info, flywheel_diameter)


if __name__ == "__main__":
    projectile_main()