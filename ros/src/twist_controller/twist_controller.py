from yaw_controller import YawController


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, 
                       max_lat_accel, max_steer_angle):
	self.yawcontroller = YawController(wheel_base, steer_ratio, min_speed, 
                                           max_lat_accel, max_steer_angle) 

    def control(self, linear_velocity, angular_velocity, current_velocity):
	throttle = 0.2
	brake    = 0.0
	angle    = self.yawcontroller.get_steering(linear_velocity, angular_velocity, current_velocity)
        return throttle, brake, angle
