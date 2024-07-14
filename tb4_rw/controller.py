import numpy as np

def controller(diff_x, diff_y, diff_theta, iteration, initial_velocity, initial_omega, dodge):
    # global robot_velocity, robot_omega, current_robot_position, robot_x_velo, robot_y_velo, dodge

    # print('now come in the controller function...')

    speed_limit = 0.13

    if dodge:
        k1 = 500e-4
        k2 = (20*k1)
    else:
        k1 = (1100e-4)      # speed
        k2 = (5*k1)      # trajectory

    z1 = diff_theta
    z2 = diff_x*np.cos(diff_theta) + diff_y*np.sin(diff_theta)
    z3 = diff_x*np.sin(diff_theta) - diff_y*np.cos(diff_theta)

    # print('z1:', z1)

    x1 = z1
    x2 = z2
    x3 = -2*z3+z1*z2

    denominator = x1**2+x2**2
    u1 = -k1*x1 + ((k2*x3)/denominator)*x2
    u2 = -k1*x2 - ((k2*x3)/denominator)*x1

    # get start from zero velocity
    initial_region = 30
    if iteration <= initial_region:
        velocity = (iteration*((iteration+1)/(initial_region*1.1))*(u2 + z3*u1) + (initial_region-iteration)*initial_velocity)/initial_region
        omega = (iteration*((iteration+0.1*initial_region)/(initial_region*1.1))*u1 + (initial_region-iteration)*initial_omega)/initial_region
    else:
        velocity = 1*(u2 + z3*u1)   # the linear velocoty of the robot
        omega = 1*u1

    # limit the speed
    if velocity > speed_limit:
        velocity = speed_limit
    elif velocity < -speed_limit:
        velocity = -speed_limit

    # print('return velo:', velocity, 'return omega:', omega)

    robot_velocity = -velocity
    robot_omega = omega
    # robot_x_velo = -robot_velocity* np.cos(state_arr[0][2])      # the negative sign should be modified
    # robot_y_velo = -robot_velocity* np.sin(state_arr[0][2])      # the negative sign should be modified

    return robot_velocity, robot_omega

