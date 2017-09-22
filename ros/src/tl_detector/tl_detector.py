#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import numpy as np

STATE_COUNT_THRESHOLD = 3
DEBUG_MODE = True

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=1)
        # https://answers.ros.org/question/220502/image-subscriber-lag-despite-queue-1/
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1, buff_size=2**16)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights
        # print("traffic light state: %d" % self.lights[0].state)

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        if not self.waypoints:
            return None
        # brute-force algorithm
        closest_index = None
        min_dist = float("inf")
        for i, waypoint in enumerate(self.waypoints.waypoints):
            curr_dist = self.distance(pose.position, waypoint.pose.pose.position)
            if curr_dist < min_dist:
                min_dist = curr_dist
                closest_index = i
        return closest_index

    def distance(self, a, b):
        """Calculates distance between positions a and b
        Args:
            a (position): first position
            b (position): second position

        Returns:
            value: distance value between a and b
        """
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)

    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # get transform between pose of camera and world frame
        trans = None
        rot = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #TODO Use tranform and rotation to calculate 2D position of light in image
        object_points = np.array([[point_in_world.y, point_in_world.z, point_in_world.x]])
        euler_rot = tf.transformations.euler_from_quaternion(rot)
        rospy.logwarn("trans: %s", trans)
        rospy.logwarn("rot: %s", euler_rot)
        rospy.logwarn("points: %s", object_points)
        tvec = np.array([trans[1], trans[2], trans[0]])
        camera_matrix = np.array([[fx, 0, image_width*0.5], [ 0, fy*300, -image_height*0.5], [ 0,  0,  1]])
        ret, _ = cv2.projectPoints(object_points, euler_rot, tvec, camera_matrix, None)
        x, y = ret[0][0]
        # x = (fx*point_in_world.y+trans[1])/point_in_world.x + image_width*0.5
        # y = image_height*0.5 - (fy*point_in_world.y+trans[2])/point_in_world.x

        rospy.logwarn("xy: %f %f", x, y)
        return (int(x), int(y))

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        # cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # x, y = self.project_to_image_plane(light.pose.pose.position)
        #TODO use light location to zoom in on traffic light in image

        #Get classification
        # predicted = self.light_classifier.get_classification(cv_image)
        # get light state from ground truth topic
        predicted = self.lights[0].state
        print("ground truth state: %d" % predicted)

        if DEBUG_MODE:
            # save image for debug purposes
            now = rospy.Time.now()
            # cv2.rectangle(cv_image, (x-70, y-100), (x+70, y+100), (255, 0, 0), 2)
            # cv2.imwrite('tmp/' + str(predicted) + "_" + str(now) +'.jpg', cv_image)

        return predicted


    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
            closest_index = None
            # find the closest visible traffic light (if one exists)
            if self.waypoints:
                pose = Pose()
                pose.position.z = 0
                min_dist = float("inf")
                for i, light_position in enumerate(stop_line_positions):
                    pos = self.waypoints.waypoints[car_position].pose.pose.position
                    curr_dist = math.sqrt((pos.x-light_position[0])**2 + (pos.y-light_position[1])**2)
                    pose.position.x = light_position[0]
                    pose.position.y = light_position[1]
                    light_wp = self.get_closest_waypoint(pose)
                    wp_dist_tolerance = 400
                    if curr_dist < min_dist and (light_wp >= car_position or (light_wp + len(self.waypoints.waypoints)) - car_position <= wp_dist_tolerance ):
                        min_dist = curr_dist
                        closest_index = i

                if closest_index != None:
                    # create object for closest light position
                    pose.position.x = stop_line_positions[closest_index][0]
                    pose.position.y = stop_line_positions[closest_index][1]
                    light_wp = self.get_closest_waypoint(pose)

                    rospy.logdebug("Traffic light waypoint: %d", light_wp)
                    rospy.logdebug("Car waypoint: %d", car_position)
                    # create light object
                    light = TrafficLight()
                    light.pose = self.waypoints.waypoints[light_wp].pose
                    light.pose.pose.position.z = 5.85 # FIXME detect light position based on image

        if light:
            state = self.lights[closest_index].state # self.get_light_state(light)
            return light_wp, state
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
