#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import copy
from enum import Enum

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number

ONE_MPH = 0.44704 #in m/s

SPEED = 10 * ONE_MPH

ACCEL = 1.0 #Acceleration in m/s2

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        # Uncomment the following two lines and the import when dealing with TL 
        rospy.Subscriber('/obstacle_waypoints', PoseStamped, self.obstacle_cb)
        rospy.Subscriber('/traffic_waypoints', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.position = None

        self.lane = None

        self.last_time = rospy.Time.now().to_sec()

        self.moving = True

        self.loop()

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            if ( self.position is not None and self.lane is not None):
                self.publish_final_wps()
                
            rate.sleep()

    def pose_cb(self, msg):
        self.position = msg.pose.position
        #self.publish_final_wps()

        #quaternion = ( 
        #    msg.pose.orientation.x, 
        #    msg.pose.orientation.y, 
        #    msg.pose.orientation.z, 
            #    msg.pose.orientation.w)
        #euler = tf.transformations.euler_from_quaternion(quaternion)
        #self.yaw = euler[2]

    def waypoints_cb(self, waypoints):
        self.lane = waypoints
        #self.publish_final_wps()

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        rospy.loginfo("TL received", msg)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        rospy.loginfo("Obstacle received", msg.pose.position)

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def wpbehind(self, wp):
        wpslen = len(self.lane.waypoints)
        dx = self.position.x - self.lane.waypoints[wp].pose.pose.position.x
        dy = self.position.y - self.lane.waypoints[wp].pose.pose.position.y
        nx = self.lane.waypoints[(wp+1)%wpslen].pose.pose.position.x - \
             self.lane.waypoints[wp].pose.pose.position.x
        ny = self.lane.waypoints[(wp+1)%wpslen].pose.pose.position.y - \
             self.lane.waypoints[wp].pose.pose.position.y
        dp = dx*nx + dy*ny
        return dp>0.0

    def dist(self,x1,y1,x2,y2):
        return math.sqrt( (x2-x1)**2 + (y2-y1)**2 )

    def closestwp(self):
        wpslen = len(self.lane.waypoints)
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        mindist = 1000000000.0
        wp = -1
        for i in range(wpslen):
            d = dl(self.position, self.lane.waypoints[i].pose.pose.position)
            if d < mindist:
              mindist = d
              wp = i
        if self.wpbehind(wp):
           wp = (wp+1)%wpslen
        return wp


    def publish_final_wps(self):

        # switch between moving and not moving every 10 seconds
        current_time = rospy.Time.now().to_sec()
        if  current_time - self.last_time > 20:
            self.moving = not self.moving
            self.last_time = current_time

        if self.lane == None or self.position==None:
           return
        wpslen = len(self.lane.waypoints)
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        mindist = 1000000000.0
        wp = -1
        for i in range(wpslen):
            d = dl(self.position, self.lane.waypoints[i].pose.pose.position)
            if d < mindist:
              mindist = d
              wp = i
        if self.wpbehind(wp):
           wp = (wp+1)%wpslen
        l = Lane()
        #l.header = std_msgs.msg.Header()
        l.header.stamp = rospy.Time.now()

        for i in range(wp, wp+LOOKAHEAD_WPS):
            currwp = copy.deepcopy(self.lane.waypoints[i%wpslen])
            currwp.twist.twist.linear.x = SPEED

            # TEST MAURIZIO
            if not self.moving:
                currwp.twist.twist.linear.x = 0.

            l.waypoints.append(currwp)
            #rospy.loginfo('Pub: - wp:%s, len:%s',wp, len(l.waypoints))
        self.final_waypoints_pub.publish(l)



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
