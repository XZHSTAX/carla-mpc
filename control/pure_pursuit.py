import numpy as np
from .get_target_point import get_target_point
# from solutions.control.get_target_point import get_target_point
import math

# TODO: Tune parameters of PID with these global variables
param_Kp = 3
param_Ki = 0.0057
param_Kd = 0
# TODO: Tune parameters of Pure Pursuit with these global variables
param_K_dd = 0.3
# The above parameters will be used in the Carla simulation
# The simple simulation in tests/control/control.ipynb does not use these parameters


class PurePursuit:
    def __init__(self, K_dd=param_K_dd, wheel_base=2.65, waypoint_shift=1.4):
        self.K_dd = K_dd
        self.wheel_base = wheel_base # called L in the book
        self.waypoint_shift = waypoint_shift
    
    def get_control(self, waypoints, speed):
        # transform x coordinates of waypoints such that coordinate origin is in rear wheel
        waypoints[:,0] += self.waypoint_shift

        # TODO: implement pure pursuit algorithm to get the steering angle
        # lookahead = self.K_dd * speed
        lookahead = np.clip(self.K_dd * speed, 3,20)
        coordinate = get_target_point(lookahead,waypoints)

        if (coordinate == None):
            return 0
        
        x,y = coordinate
        alpha = math.atan2(y, x)
        steer = math.atan(2 * self.wheel_base * math.sin(alpha)/lookahead) # called delta in the book

        # undo transform to waypoints 
        waypoints[:,0] -= self.waypoint_shift
        return steer


class PIDController:
    """ PID Controller copied from book """
    def __init__(self, Kp, Ki, Kd, set_point):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.set_point = set_point
        self.int_term = 0
        self.derivative_term = 0
        self.last_error = None
    
    def get_control(self, measurement, dt):
        error = self.set_point - measurement
        self.int_term += error*self.Ki*dt
        if self.last_error is not None:
            self.derivative_term = (error-self.last_error)/dt*self.Kd
        self.last_error = error
        return self.Kp * error + self.int_term + self.derivative_term


class PurePursuitPlusPID:
    def __init__(self, pure_pursuit=PurePursuit(), pid=PIDController(param_Kp, param_Ki, param_Kd, 0)):
        self.pure_pursuit = pure_pursuit
        self.pid = pid

    def get_control(self,waypoints, speed, desired_speed, dt):
        self.pid.set_point = desired_speed
        a = self.pid.get_control(speed,dt)
        steer = self.pure_pursuit.get_control(waypoints, speed)
        return a, steer

