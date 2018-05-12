from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, params):
	self.min_speed = 0.1
	self.yaw_controller = YawController(params['wheel_base'],
					  params['steer_ratio'],
					  self.min_speed,
					  params['max_lat_accel'],
					  params['max_steer_angle'])  

	self.vehicle_mass = params['vehicle_mass'] 
	self.decel_limit  = params['decel_limit']
	self.wheel_radius = params['wheel_radius']      

	kp = 0.3
	ki = 0.1
	kd = 0.0
	mn = 0.0
	mx = 0.2 

	self.throttle_controller = PID(kp, ki, kd, mn, mx)

	tau = 0.5
	ts  = 0.02
	self.velocity_filter = LowPassFilter(tau, ts)

	self.last_time = rospy.get_time()



    def control(self, tgt_lin_vel, tgt_ang_vel, cur_lin_vel, dbw_enabled):

	if not dbw_enabled:
        	return 0., 0., 0.

	current_velocity = self.velocity_filter.filt(cur_lin_vel)
	steering = self.yaw_controller.get_steering(tgt_lin_vel, tgt_ang_vel, cur_lin_vel)
	vel_error = tgt_lin_vel - current_velocity
	
	self.last_velocity = current_velocity
	cur_time = rospy.get_time()
	sample_time = cur_time - self.last_time
	self.last_time = cur_time

	throttle = self.throttle_controller.step(vel_error, sample_time)

	brake_torque = 0

	if tgt_lin_vel == 0 and current_velocity < self.min_speed:
		throttle = 0
		brake_torque = 400

	elif throttle < 0.1 and vel_error < 0:
		throttle = 0
		decel = max(vel_error, self.decel_limit)
		brake_torque = abs(decel) * self.vehicle_mass * self.wheel_radius
	
	return throttle, brake_torque, steering
