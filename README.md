# Team AVS

Team Member | Udacity Account Eamil
----------- | ---------------------
Mate Bartalos | bartmate@gmail.com
Maurizio Pinto | maurizio.pinto@gmail.com
Sampath Vanimisetti | sampath.vanimisetti@gmail.com
Florian Wulff | florian.wulff@web.de
Victor Costa | vfcosta@gmail.com

# Short Description about our Solution

## Waypoint Updater (WPU)

### Topics WPU is subscribed to

Topic Name | Description
---------- | -----------
/vehicle/dbw_enabled | Getting the information if DBW is switched on or off
/base_waypoints | Getting the waypoint list
/current_pose | Getting the current position of the car
/current_velocity | Getting the current velocity of the car
/traffic_waypoints | Getting the waypoint of the next Stop Line if the corresponding Traffic Light is red or yellow
/obstacle_waypoints | Getting the waypoint of the next obstacle (Not used)

### Topics WPU publishes into

Topic Name | Description
---------- | -----------
/final_waypoints | Publishing the velocity values for each waypoints (in certain horizon)

### Summary on how WPU works

The WPU is basically a finitive state automaton with 4 states.

State | Description | Possible Next States
----- | ----------- | ------------------
Start of Acceleration | In this state it is planned how the car will accelerate until reaching the target speed. The extent of the acceleration is hard wired. This is the start state. | Acceleration, Start of Deceleration
Acceleration | In this state the acceleration plan is followed. If the target speed is reached, it is kept further. | Acceleration, Start of Deceleration
Start of Deceleration | In this state it is planned how the car will decelerate until reaching the Stop Line. The deceleration rate is between a hard wired minimum and maximum deceleration rate. | Deceleration, Start of Acceleration
Deceleration | In this state the deceleration plan is followed. If the zero speed is reached, it is kept further. | Deceleration, Start of Acceleration

In case of acceleration (i.e. states Start of Acceleration and Acceleration) it is checked if there is a red or yellow Traffic Light in a distance for which the car should decelerate between a hard wired minimum and maximum deceleration rate. If yes, the state goes to Start of Deceleration. This logic ensures two important things:
* When the Traffic Light changes from green to yellow/red, the car will not stop if it is not safe to do so (i.e. the deceleration rate would be too high).
* When a red Traffic Light is too far away, the car will not decelerate very slowly, instead it goes on (maybe the Traffic Light will change to green), and brakes only if the Traffic Light is close enough.

In case of deceleration (i.e. states Start of Deceleration and Deceleration) it is checked if the next Traffic Light changed to green. If yes, the state goes to Start of Acceleration.


### Other remarks

The jerk-minimizing formula was not used, because the controller in DBW smoothes the speed values, so the jerk does not approach the critical level.

## DBW

## Traffic Light Detection


