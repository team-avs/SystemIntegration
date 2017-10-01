from yaw_controller import YawController

import rospy
import pid
from lowpass import LowPassFilter

import math


GAS_DENSITY = 2.858 # needed to calc the car's mass when fuel is used
ONE_MPH = 0.44704

# PID params for throttle
T_kp = 2.0
T_ki = 0.4
T_kd = 0.1

# PID params for steer - high speed
S_kp_high = 0.8
S_ki_high = 0.01
S_kd_high = 0.03


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
		 self.steer_ratio = kwargs.get('steer_ratio')
		 self.min_speed = kwargs.get('min_speed')
		 self.max_speed = rospy.get_param('/waypoint_loader/velocity') / 3.6
		 max_lat_accel = kwargs.get('max_lat_accel')
		 self.max_steer_angle = kwargs.get('max_steer_angle')
		 self.decel_limit = kwargs.get('decel_limit')
		 self.accel_limit = kwargs.get('accel_limit')
		 
		 self.vehicle_mass = kwargs.get('vehicle_mass')
		 self.wheel_radius = kwargs.get('wheel_radius')
		 self.brake_deadband = kwargs.get('brake_deadband')

		 self.yawcontroller = YawController(wheel_base, self.steer_ratio, self.min_speed,
											max_lat_accel, self.max_steer_angle)

		 self.throttle_pid = pid.PID(kp=T_kp, ki=T_ki, kd=T_kd, mn=self.decel_limit, mx=self.accel_limit)
		 self.steer_pid_high = pid.PID(kp=S_kp_high, ki=S_ki_high, kd=S_kd_high, mn=-self.max_steer_angle, mx=self.max_steer_angle)
		 self.steer_pid_high_cte = pid.PID(kp=S_kp_high, ki=S_ki_high, kd=S_kd_high, mn=-self.max_steer_angle, mx=self.max_steer_angle)
		 
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
		 cte = kwargs.get('cte')

		 # used PID for throttle 
		 # used yawcontroller to get the steering angle
		 # used two PIDs for steering (yaw and CTE error)
		 # brake value set to vehicle mass times throttle times wheel radius

		 if dbw_enabled:

			throttle = min(self.accel_limit, self.throttle_pid.step(trgtv - currv, elapsed))
			brake = 0.

			target_angle = self.yawcontroller.get_steering(trgtv, trgtav, currv) 
			angle_high_speed = self.steer_pid_high.step(target_angle - current_angle, elapsed)
			angle_high_speed_cte = self.steer_pid_high_cte.step(cte, elapsed)
			 
			angle = (target_angle + angle_high_speed + angle_high_speed_cte) / 3. # average
			
			if throttle < self.brake_deadband or trgtv < 0.1: #desired speed is close to 0 or we are in the brake deadband
				brake = -(self.vehicle_mass * throttle * self.wheel_radius) # vehicle mass times deceleration time wheel radius
				brake = max(self.decel_limit, brake)
				throttle = 0. # do not activate the throttle while braking
			else:
				brake = 0. # no braking if the car is traveling
			 
			return throttle, brake, angle
		 else:
			self.steer_pid_high.reset()
			self.steer_pid_high_cte.reset()
			self.throttle_pid.reset()
			return 0., 0., 0.