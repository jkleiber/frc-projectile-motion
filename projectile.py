import numpy as np

from constants import (
    GRAVITY
)

def compute_time_of_flight(opt_params, delta_y, is_intercept=False):
    v0 = opt_params[0]
    launch_angle = opt_params[1]

    # Decompose the initial velocity
    vy_0 = v0 * np.sin(launch_angle)

    # Get time of flight to target height.
    # y = y0 + vy_0*t + 0.5*a*t^2
    # 0.5*a*t^2 + vy_0*t + (y0 - y) = 0
    # Solve for t:
    #
    # t = (-vy_0 +/- sqrt(vy_0^2 - 2*a*(y0-y))) / a
    quad_a = 0.5*GRAVITY
    quad_b = vy_0
    quad_c = -1.0 * delta_y
    determinant = quad_b*quad_b - 4*quad_a*quad_c

    if determinant < 0.0:
        return 0.0
    
    t1 = (-1.0 * quad_b - np.sqrt(determinant)) / (2.0 * quad_a)
    t2 = (-1.0 * quad_b + np.sqrt(determinant)) / (2.0 * quad_a)

    time_of_flight = max(t1, t2)
    if is_intercept:
        intercept_time = min(t1, t2)
        time_of_flight = intercept_time if intercept_time > 0.0 else time_of_flight 

    return time_of_flight

def compute_arrival_distance(opt_params, delta_y, time_of_flight=None):
    if time_of_flight is None:
        time_of_flight = compute_time_of_flight(opt_params, delta_y)

    v0 = opt_params[0]
    launch_angle = opt_params[1]
    
    vx_0 = v0 * np.cos(launch_angle)

    # Determine flight distance, assuming starting from x=0.
    x = vx_0 * time_of_flight

    return x

def compute_arrival_angle(opt_params, delta_y, time_of_flight=None):
    if time_of_flight is None:
        time_of_flight = compute_time_of_flight(opt_params, delta_y)
    
    v0 = opt_params[0]
    launch_angle = opt_params[1]

    # Decompose the initial velocity
    vx_0 = v0 * np.cos(launch_angle)
    vy_0 = v0 * np.sin(launch_angle)

    # Compute final velocities
    vx_f = vx_0
    vy_f = vy_0 + time_of_flight * GRAVITY
    arrival_angle = np.arctan2(vy_f, vx_f)

    return arrival_angle

def compute_projectile_motion(opt_params, delta_y):
    v0 = opt_params[0]
    launch_angle = opt_params[1]

    tof = compute_time_of_flight([v0, launch_angle], delta_y)
    distance = compute_arrival_distance([v0, launch_angle], delta_y, time_of_flight=tof)
    arrival_angle = compute_arrival_angle([v0, launch_angle], delta_y, time_of_flight=tof)

    return distance, tof, arrival_angle
