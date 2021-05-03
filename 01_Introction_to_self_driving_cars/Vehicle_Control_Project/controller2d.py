#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        # RETRIEVE SIMULATOR FEEDBACK
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0


        self.vars.create_var('last_error', 0.0)
        self.vars.create_var('t_previous', 0.0)
        self.vars.create_var('a_previous', 0.0)
        #self.vars.create_var('v_previous', 0.0)

        # Skip the first frame to store previous values properly
        if self._start_control_loop:

            # define PIDs constants
            kp=4.5
            ki=1.4
            kd=0.37

            # define required errors
            errP = v_desired - v
            errI = errP + self.vars.last_error
            errD = errP - self.vars.last_error

            #update error
            self.vars.last_error = errP

            # define time step for decritization
            dt = t - self.vars.t_previous
            # update time
            self.vars.t_previous = t

            # PID controller

            a_desired = errP * kp  + errI * ki * dt + (errD * kd)/dt 
  
            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.

            # throttle_output = a_desired
            # brake_output = 0
            if  a_desired >= 0 :
                throttle_output = a_desired
                brake_output = 0
            else:
                brake_output    = a_desired
                throttle_output = 0 

            # # update accerelation
            # self.vars.a_previous =  a_desired 

			# Use stanley controller for lateral control
            # stanley parameter
            k = 0.5

            # calculate heading error
            yaw_path = np.arctan2(waypoints[-1][1]-waypoints[0][1], waypoints[-1][0]-waypoints[0][0])
            epsi = yaw_path - yaw 
            if epsi > np.pi:
                epsi -= 2 * np.pi
            if epsi < - np.pi:
                epsi += 2 * np.pi

            # calculate crosstrack error
            # Trajectory line    ax+by+c=0
            slope = (waypoints[-1][1]-waypoints[0][1])/ (waypoints[-1][0]-waypoints[0][0])
            a = -slope
            b = 1.0
            c = (slope*waypoints[0][0]) - waypoints[0][1]
            XTE = (a*x + b*y + c) / np.sqrt(a**2 + b**2)

            yaw_cross_track = np.arctan2(y-waypoints[0][1], x-waypoints[0][0])
            
            # Adjust the direction
            yaw_path2ct = yaw_path - yaw_cross_track
            if yaw_path2ct > np.pi:
                yaw_path2ct -= 2 * np.pi
            if yaw_path2ct < - np.pi:
                yaw_path2ct += 2 * np.pi
            if yaw_path2ct > 0:
                XTE = abs(XTE)
            else:
                XTE = - abs(XTE)


            # control low
            steer_expect = epsi + np.arctan(k * XTE / (v))

            # modify the direction
            if steer_expect > np.pi:
                steer_expect -= 2 * np.pi
            if steer_expect < - np.pi:
                steer_expect += 2 * np.pi

            # update
            steer_output = steer_expect

            # SET CONTROLS OUTPUT
            
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)


        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        #self.vars.v_previous = v  # Store forward speed to be used in next step
