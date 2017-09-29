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

The WPU is basically a finitive state automaton with 4 states. See the details in the table below.

State Number | State | Description | Possible Next States
------------ | ----- | ----------- | ------------------
0 | Start of Acceleration | | Acceleration, Start of Deceleration
1 | Acceleration | | Acceleration, Start of Deceleration
2 | Start of Deceleration | | Deceleration, Start of Acceleration
3 | Deceleration | | Deceleration, Start of Acceleration


### Other remarks

The jerk-minimizing formula was not used, because the controller in DBW smooth the speed values, so the jerk does not approach the critical level.

## DBW

## Traffic Light Detection


