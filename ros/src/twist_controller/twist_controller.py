from yaw_controller import YawController

import rospy
import pid
from lowpass import LowPassFilter

import math


GAS_DENSITY = 2.858 # needed to calc the car's mass when fuel is used
ONE_MPH = 0.44704

# PID params for throttle
T_kp = 0.8
T_ki = 0.0
T_kd = 0.01

# PID params for steer
S_kp = 0.45
S_ki = 0.03
S_kd = 0.04

# PID params for steer - high speed
S_kp_high = 5.00
S_ki_high = 0.5
S_kd_high = 1.2

# Params for lowpass filter
tau = 0.1
ts = 1.0



class Controller(object):


## *kwargs definition (see dwb_node.py)
## 
## controller_args = {'wheel_base': wheel_base,
##                    'steer_ratio': steer_ratio,
##                    'min_speed': min_speed,
##                    'max_lat_accel' : max_lat_accel,
##                    'max_steer_angle' : max_steer_angle,
##                    'decel_limit': decel_limit,
##                    'accel_limit' : accel_limit.
##                    'vehicle_mass' : vehicle_mass,
##					  'wheel_radius' : wheel_radius,
##					  'brake_deadband' : brake_deadband
##                    }

	 def __init__(self, *args, **kwargs):

		 wheel_base = kwargs.get('wheel_base')
		 steer_ratio = kwargs.get('steer_ratio')
		 min_speed = kwargs.get('min_speed')
		 max_lat_accel = kwargs.get('max_lat_accel')
		 max_steer_angle = kwargs.get('max_steer_angle')
		 self.decel_limit = kwargs.get('decel_limit')
		 self.accel_limit = kwargs.get('accel_limit')
		 
		 self.vehicle_mass = kwargs.get('vehicle_mass')
		 self.wheel_radius = kwargs.get('wheel_radius')
		 self.brake_deadband = kwargs.get('brake_deadband')

	 	 self.yawcontroller = YawController(wheel_base, steer_ratio, min_speed,
											max_lat_accel, max_steer_angle)

		 self.throttle_pid = pid.PID(kp=T_kp, ki=T_ki, kd=T_kd, mn=self.decel_limit, mx=self.accel_limit)
		 self.steer_pid = pid.PID(kp=S_kp, ki=S_ki, kd=S_kd, mn=-max_steer_angle, mx=max_steer_angle)
		 self.steer_pid_high = pid.PID(kp=S_kp_high, ki=S_ki_high, kd=S_kd_high, mn=-max_steer_angle, mx=max_steer_angle)
		 self.lowpass_filter = LowPassFilter(tau, ts) # TODO find optimal params

		 self.last_time = rospy.rostime.get_time()


## *kwargs definition (see dwb_nobe.py)
## control_args = {'trgtv' : self.trgtv, # target linear velocity
##                 'currv' : self.currv, # current linear velocity
##                 'trgtav' : self.trgtav, # target angular velocity
##                 'currav' : self.currav, # current angular velocity
##                 'dbw_enabled' : self.dbw_enabled, # dbw status
##				   'current_angle' : self.current_angle,
##                 'elapsed' : elapsed
##                  }

	 def control(self, *args, **kwargs):

		 trgtv = kwargs.get('trgtv')
		 currv = kwargs.get('currv')
		 trgtav = kwargs.get('trgtav')
		 currav = kwargs.get('currav')
		 dbw_enabled = kwargs.get('dbw_enabled')
		 elapsed = kwargs.get('elapsed')
		 current_angle = kwargs.get('current_angle')

		 max_angle = 0.0

		 # used PID for throttle 
		 # used yawcontroller to get the steering angle
		 # used PID for steering(using target angle and current angle)
		 # used lowpass filter to smooth steering response
		 # brake value set to vehicle mass times throttle times wheel radius
		  
		 if dbw_enabled:

		 	throttle = min(self.accel_limit, self.throttle_pid.step(trgtv - currv, elapsed))
			brake = 0.0 

			target_angle = self.yawcontroller.get_steering(trgtv, trgtav, currv) 

			angle_low_speed = self.steer_pid.step(target_angle - current_angle, elapsed)
			angle_high_speed = self.steer_pid_high.step(target_angle - current_angle, elapsed)
			 
			if trgtv < 20:
				angle = angle_low_speed
				angle = self.lowpass_filter.filt(angle) 
			else:
				angle = angle_high_speed

			if throttle < self.brake_deadband: # desired speed is 0 or close to 0 brake deadband
			 	brake =  -(self.vehicle_mass * throttle * self.wheel_radius) # vehicle mass times deceleration
			 	brake = max(self.decel_limit, brake)
			 	throttle = 0 # do not activate the throttle while braking
			else:
			 	brake = 0 # no braking if the car is traveling
			 
			return throttle, brake, angle
		 else:
			self.steer_pid.reset()
			self.throttle_pid.reset()
			self.lowpass_filter.last_val = 0.
			return 0., 0., 0.
