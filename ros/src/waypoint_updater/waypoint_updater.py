#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Bool,Int32, Float32


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

ONE_MPH = 0.44704 #m/s

SAFE = 0.5

class State(Enum):
    ACC = 1
    DEC = 2

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/obstacle_waypoints', PoseStamped, self.obstacle_cb)
        rospy.Subscriber('/traffic_waypoints', Int32, self.traffic_cb)
	rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
	rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb,queue_size=1)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.cte_pub = rospy.Publisher('/vehicle/cte', Float32, queue_size=1)

        self.lane = None
        self.wpslen = None

	self.dbw_enabled = False

        self.position     = None
	self.currv        = None 
	self.stop_line_wp = None
	self.state        = None
	self.statechanged = None 
	self.last_output  = None

	self.MAX_ACC_A = rospy.get_param('/dbw_node/accel_limit')
	self.MAX_DEC_A = -rospy.get_param('/dbw_node/decel_limit')
	self.MIN_DEC_A = min(1.0,-rospy.get_param('/dbw_node/decel_limit')/2.)
	self.SPEED     = rospy.get_param('/waypoint_loader/velocity')/3.6 #converting from km/h to m/s
	#print("{},{},{},{}".format(self.MAX_ACC_A,self.MAX_DEC_A,self.MIN_DEC_A,self.SPEED))

        self.loop()

    def reset(self):
        self.position = None
	self.currv = None #m/s
	self.stop_line_wp = -1 #Closest WP to the next TL-stop-line, -1 if it is too far
	self.state = State.ACC
	self.statechanged = True 
	self.last_output = None
	self.MAX_ACC_A = rospy.get_param('~accel_limit', 1.) #m/s
	self.MAX_DEC_A = -rospy.get_param('~decel_limit', -5.) #m/s
	self.MIN_DEC_A = min(1.0,-rospy.get_param('~decel_limit', -5.)/2.) #m/s


    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_final_wps()    
            rate.sleep()

    def pose_cb(self, msg):
        self.position = msg.pose.position

    def waypoints_cb(self, waypoints):
        self.lane = waypoints
	self.wpslen = len(self.lane.waypoints)

    def traffic_cb(self, msg):
        #rospy.loginfo("TL received: %s", msg)
	self.stop_line_wp = msg.data
	if self.stop_line_wp>0 and self.wpslen is not None:
	    self.stop_line_wp = ( self.stop_line_wp - 5 + self.wpslen ) % self.wpslen

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        #rospy.loginfo("Obstacle received: %s", msg.pose.position)
	pass

    def current_velocity_cb(self, msg):
	self.currv  = msg.twist.linear.x 

    def dbw_enabled_cb(self, msg):
	if msg.data != self.dbw_enabled and msg.data:
            self.reset()
	self.dbw_enabled = msg.data

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self,x1,y1,x2,y2):
        return math.sqrt( (x2-x1)**2 + (y2-y1)**2 )

    def distancepos(self,pos1,pos2):
        return math.sqrt( (pos2.x-pos1.x)**2 + (pos2.y-pos1.y)**2 )

    def distancewp(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[wp2].pose.pose.position)
            wp1 = i
        return dist

    def wpbehind(self, wp):
        dx = self.position.x - self.lane.waypoints[wp].pose.pose.position.x
        dy = self.position.y - self.lane.waypoints[wp].pose.pose.position.y
        nx = self.lane.waypoints[(wp+1)%self.wpslen].pose.pose.position.x - \
             self.lane.waypoints[wp].pose.pose.position.x
        ny = self.lane.waypoints[(wp+1)%self.wpslen].pose.pose.position.y - \
             self.lane.waypoints[wp].pose.pose.position.y
        dp = dx*nx + dy*ny
        return dp>0.0

    def closestwp(self):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        mindist = 1000000000.0
        wp = -1
        for i in range(self.wpslen):
            d = dl(self.position, self.lane.waypoints[i].pose.pose.position)
            if d < mindist:
              mindist = d
              wp = i
        if self.wpbehind(wp):
           wp = (wp+1)%self.wpslen
	return wp

    def startaccel(self,l,wp):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
	a = self.MAX_ACC_A
	v0 = self.currv 
	v = self.currv
	i = 0
	while v<self.SPEED or i<LOOKAHEAD_WPS:
	    d = dl(self.position,self.lane.waypoints[(wp+i)%self.wpslen].pose.pose.position)
	    v = math.sqrt(v0**2.0+2.0*a*d)
	    if v>self.SPEED:
		v = self.SPEED
            currwp = copy.deepcopy(self.lane.waypoints[(wp+i)%self.wpslen])
            currwp.twist.twist.linear.x = v
            l.waypoints.append(currwp)
            i += 1

    def startdecel(self,l,wp):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        v0 = self.currv 
	v = self.currv
        d = dl(self.position, self.lane.waypoints[self.stop_line_wp].pose.pose.position)-SAFE
        a = v0**2.0/(2.0*d)

	i = 0
	while v>0.0 or i<LOOKAHEAD_WPS:
	    d = dl(self.position,self.lane.waypoints[(wp+i)%self.wpslen].pose.pose.position)
	    if v0**2.0-2.0*a*d<=0:
                v = 0
            else:
	        v = math.sqrt(v0**2.0-2.0*a*d)
            currwp = copy.deepcopy(self.lane.waypoints[(wp+i)%self.wpslen])
            currwp.twist.twist.linear.x = v
            l.waypoints.append(currwp)
            i += 1

    def keep(self,l,wp,cv):
	j=0
	while j<len(self.last_output):
	    if self.last_output[j].pose.pose.position == self.lane.waypoints[wp].pose.pose.position:
		break
            j += 1
	for i in range(j,len(self.last_output)):
            #currwp = copy.deepcopy(self.last_output[j])
            currwp = copy.deepcopy(self.lane.waypoints[(wp+i-j)%self.wpslen])
            currwp.twist.twist.linear.x = self.last_output[i].twist.twist.linear.x
	    l.waypoints.append(currwp)
	for i in range(len(l.waypoints),LOOKAHEAD_WPS):
            currwp = copy.deepcopy(self.lane.waypoints[(wp+i)%self.wpslen])
            currwp.twist.twist.linear.x = cv
            l.waypoints.append(currwp)

    def publish_final_wps(self):
        
	dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

        if not self.dbw_enabled or self.lane == None or self.position==None or self.currv==None:
           return

	t0 = rospy.Time.now()

	wp = self.closestwp()
        l = Lane()	
        l.header.stamp = rospy.Time.now()

	#Handling state-changes
	if self.state == State.ACC:
	    if self.stop_line_wp != -1:
                d = dl(self.position, self.lane.waypoints[self.stop_line_wp].pose.pose.position)-SAFE
	        min_brake_d = 0.5*self.currv**2/self.MAX_DEC_A
		max_brake_d = 0.5*self.currv**2/self.MIN_DEC_A
		if d<=max_brake_d and d>=min_brake_d:
		    self.state = State.DEC
		    self.statechanged = True 
	elif self.state == State.DEC:
	    if self.stop_line_wp == -1:
	    	self.state = State.ACC
		self.statechanged = True 
	else:
            rospy.logerr("Waypoint Updater: Not existing state.")

	#print("State:{}".format(self.state))

	
	#Handling states
	if self.state == State.ACC and self.statechanged:
            self.startaccel(l,wp)
	elif self.state == State.ACC and not self.statechanged:
            self.keep(l,wp,self.SPEED)
	elif self.state == State.DEC and self.statechanged:
           self.startdecel(l,wp)
	elif self.state == State.DEC and not self.statechanged:
           self.keep(l,wp,0)
	else:
            rospy.logerr("Waypoint Updater: Not existing state.")
	self.statechanged = False 

	self.last_output = copy.deepcopy(l.waypoints)
        self.final_waypoints_pub.publish(l)

	wp0 = (wp-1+self.wpslen)%self.wpslen
	a = self.distancepos(self.lane.waypoints[wp0].pose.pose.position,self.lane.waypoints[wp].pose.pose.position)
	b = self.distancepos(self.position,self.lane.waypoints[wp].pose.pose.position)
	c = self.distancepos(self.position,self.lane.waypoints[wp0].pose.pose.position)
	s = (a+b+c)/2.
        tmp = s*(s-a)*(s-b)*(s-c)
	if tmp<0.0:
	    cte = 0.0
	else:
	    cte = 2. * math.sqrt(tmp) / a
	x1 = self.lane.waypoints[wp].pose.pose.position.x-self.lane.waypoints[wp0].pose.pose.position.x
	y1 = self.lane.waypoints[wp].pose.pose.position.y-self.lane.waypoints[wp0].pose.pose.position.y
	x2 = self.position.x-self.lane.waypoints[wp0].pose.pose.position.x
	y2 = self.position.y-self.lane.waypoints[wp0].pose.pose.position.y
	cross = x1*y2-x2*y1
	if cross>0:
	    cte *= -1
	dt = rospy.Time.now() - t0
	planv = l.waypoints[0].twist.twist.linear.x
	
        self.cte_pub.publish(cte)

	#FOR DEBUG PURPOSES
	#print("ST:{}; STC:{}; WP:{}; TLSLWP:{}; CURRV:{:.2f}; PLANV:{:.2f}; CTE:{:.3f}; TIME:{:.2f}".format(self.state,self.statechanged, wp, self.stop_line_wp, self.currv, planv,cte, dt.to_sec()))
	

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
