
import numpy as np
from matplotlib import pyplot as plt


from projectile import Projectile
from units import angular_velocity_to_linear_velocity
from constants import GRAVITY, LAUNCH_HEIGHT, FLYWHEEL_DIAMETER, INCHES_TO_METERS


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
    lift_coefficient = 0.15 # guessed from NASA's baseball Cl estimate
    air_density = 1.225 # kg/m^3
    projectile_diameter_meters = projectile.diameter * INCHES_TO_METERS
    cross_sectional_area = (np.pi * projectile_diameter_meters**2) / 4.0


    total_velocity = np.sqrt(vx**2 + vz**2)

    magnus_force = 0.5 * lift_coefficient * cross_sectional_area * air_density * (omega_x * total_velocity) * (projectile_diameter_meters * 0.5)
    drag_force = -0.5 * drag_coefficient * air_density * cross_sectional_area * total_velocity**2
    gravity_force = projectile.mass * GRAVITY

    angle_of_travel = np.arctan2(vz, vx)

    Fx = (magnus_force + drag_force) * np.cos(angle_of_travel)
    Fz = (magnus_force + drag_force) * np.sin(angle_of_travel) + gravity_force 

    ax_dot = Fx / projectile.mass
    az_dot = Fz / projectile.mass
    alpha_x = 0.0

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




def projectile_main():
    flywheel_rps = 50
    launch_angle = np.radians(55.0)

    projectile = Projectile(mass=0.227, diameter=6.0)

    trajectory = compute_projectile_motion([flywheel_rps, launch_angle], projectile=projectile, flywheel_diameter=FLYWHEEL_DIAMETER)

    plt.figure()
    plt.plot(trajectory[:,1], trajectory[:,2])
    # plt.show(block=False)

    # fig, axs = plt.subplots(2, sharex=True)
    # axs[0].scatter(trajectory[:,0], trajectory[:,3])

    # axs[1].scatter(trajectory[:,0], trajectory[:,3])
    plt.show()
    


if __name__ == "__main__":
    projectile_main()