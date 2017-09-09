from yaw_controller import YawController

import rospy
import pid
from lowpass import LowPassFilter

import math


GAS_DENSITY = 2.858 # needed to calc the car's mass when fuel is used
ONE_MPH = 0.44704

# PID params for throttle
T_kp = 2.0
T_ki = 0.0
T_kd = 0.0

# PID params for steer
S_kp = 2.6
S_ki = 0.5
S_kd = 0.01

tau = 0.5
s = 0.02



class Controller(object):


## *kwargs definition (see dwb_node.py)
## 
## controller_args = {'wheel_base': wheel_base,
##                    'steer_ratio': steer_ratio,
##                    'min_speed': min_speed,
##                    'max_lat_accel' : max_lat_accel,
##                    'max_steer_angle' : max_steer_angle,
##                    'decel_limit': decel_limit,
##                    'accel_limt' : accel_limit
##                    }

	 def __init__(self, *args, **kwargs):

		 wheel_base = kwargs.get('wheel_base')
		 steer_ratio = kwargs.get('steer_ratio')
		 min_speed = kwargs.get('min_speed')
		 max_lat_accel = kwargs.get('max_lat_accel')
		 max_steer_angle = kwargs.get('max_steer_angle')
		 decel_limit = kwargs.get('decel_limit')
		 accel_limit = kwargs.get('accel_limit')
	 
		 self.yawcontroller = YawController(wheel_base, steer_ratio, min_speed,
											max_lat_accel, max_steer_angle)

		 self.throttle_pid = pid.PID(kp=T_kp, ki=T_ki, kd=T_kd, mn=decel_limit, mx=accel_limit)
		 self.steer_pid = pid.PID(kp=S_kd, ki=S_ki, kd=S_kd, mn=-max_steer_angle, mx=max_steer_angle)
		 self.lowpass_filter = LowPassFilter(tau, s) # TODO find params

		 self.start_time = rospy.get_time()


## *kwargs definition (see dwb_nobe.py)
## control_args = {'trgtv' : self.trgtv, # target linear velocity
##                 'currv' : self.currv, # current linear velocity
##                 'trgtav' : self.trgtav, # target angular velocity
##                 'currav' : self.currav, # current angular velocity
##                 'dbw_enabled' : self.dbw_enabled, # dbw status
##                 'current_pose' : self.current_pose, # needed for CTE calc
##                 'final_waypoints' : self.final_waypoint, # needed for CTE calc
##                 'elapsed' : elapsed
##                  }

	 def control(self, *args, **kwargs):

		 trgtv = kwargs.get('trgtv')
		 currv = kwargs.get('currv')
		 trgtav = kwargs.get('trgtav')
		 currav = kwargs.get('currav')
		 dbw_enabled = kwargs.get('dbw_enabled')
		 current_pose = kwargs.get('current_pose')
		 final_waypoints = kwargs.get('final_waypoints') 
		 elapsed = kwargs.get('elapsed')

		 # TODO: Maurizio to check with Mate
		 # use PID for throttle 
		 # use yawcontroller to get the steering angle
		 # use PID for steering
		 # Question: how to calculate the CTE for steering? 
		  
		 if dbw_enabled:
			 if (currv < 10): # TODO how should speed be limited?
				throttle = 0.5
				test = self.throttle_pid.step(trgtv - currv, elapsed)
				#rospy.loginfo(trgtv - currv)
			 else:
				throttle = 0.0
			 brake = 0.0 
			 target_angle = self.yawcontroller.get_steering(trgtv, trgtav, currv) 
			 current_angle = self.yawcontroller.get_steering(trgtv, currav, currv)
			 angle = self.steer_pid.step(target_angle - current_angle, elapsed)

			 angle = angle*180./math.pi/5.0
			 
			 return throttle, brake, angle
		 else:
			self.steer_pid.reset()
			self.throttle_pid.reset()
			return 0., 0., 0.
