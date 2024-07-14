import numpy as np
# def pid_controller(error_x, error_y, error_theta, iteration, initial_velocity, initial_omega, dodge, last_error, integral, dt):
# last_error = {'pos': 0, 'theta': 0}
# integral = {'pos': 0, 'theta': 0}

def pid_controller(error_x, error_y, error_theta, last_error, integral, dt):
    # combine x and y to one, remove unnecessary terms from argument
    # Constants
    speed_limit = 0.05
    # initial_region = 30
    Kp, Ki, Kd = [0.5,.1], [0.01, .001], [0.0001, .001]  

    # error_pos = np.sqrt(error_x**2 + error_y**2)

    # Integral and derivative calculations
    integral_x = integral['x'] + error_x * dt
    integral_y = integral['y'] + error_y * dt
    integral_theta = integral['theta'] + error_theta * dt

    derivative_x = (error_x - last_error['x']) / dt
    derivative_y = (error_y - last_error['y']) / dt
    derivative_theta = (error_theta - last_error['theta']) / dt

    # PID control
    ux = Kp[0] * error_x + Ki[0] * integral_x + Kd[0] * derivative_x
    uy = Kp[0] * error_y + Ki[0] * integral_y + Kd[0] * derivative_y
    omega = Kp[1] * error_theta + Ki[1] * integral_theta + Kd[1] * derivative_theta

    # Output - Need to correct
    # velocity = np.sqrt(u_x**2 + u_y**2)
    # omega = u_theta
    velocity = np.sqrt(ux**2 + uy**2)
    velocity = max(0, min(speed_limit, velocity))

    last_error['x'] = error_x
    last_error['y'] = error_y
    last_error['theta'] = error_theta
    integral['x'] = integral_x
    integral['y'] = integral_y
    integral['theta'] = integral_theta

    return velocity, omega, last_error, integral
