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
from timeit import default_timer as timer

STATE_COUNT_THRESHOLD = 3
DEBUG_MODE = True

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.current_light_index = None
        self.last_car_wp = None
        self.lights = []
        self.stop_line_waypoints = []
        self.detection_time = 0
        self.start_detection = None
        self.end_detection = None


        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoints', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

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
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        # precalculate stop line waypoints
        for light_position in self.config['stop_line_positions']:
            pose = Pose()
            pose.position.x = light_position[0]
            pose.position.y = light_position[1]
            self.stop_line_waypoints.append(self.get_closest_waypoint(pose))

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        # skip detection to avoid queue on the image publisher
        if self.detection_time > 0 and timer() - self.end_detection < self.detection_time:
            rospy.logdebug("skip tl detection")
            return

        self.has_image = True
        self.camera_image = msg
        self.start_detection = timer()
        light_wp, state = self.process_traffic_lights()
        self.end_detection = timer()
        self.detection_time = self.end_detection - self.start_detection
        rospy.logdebug("image_cb time: %f", self.detection_time)

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
            light_wp = light_wp if state == TrafficLight.RED or state == TrafficLight.YELLOW else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose, start_from=0):
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
        for i in range(0, len(self.waypoints.waypoints)):
            if start_from > 0:
                i = (i + start_from)%len(self.waypoints.waypoints)
            waypoint = self.waypoints.waypoints[i]
            curr_dist = self.distance(pose.position, waypoint.pose.pose.position)
            if curr_dist < min_dist:
                min_dist = curr_dist
                closest_index = i
            if start_from!=0 and curr_dist > min_dist:
                break
        return closest_index

    def distance(self, a, b):
        """Calculates distance between positions a and b
        Args:
            a (position): first position
            b (position): second position

        Returns:
            value: distance value between a and b
        """
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)


    def get_light_state(self):
        """Determines the current color of the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        predicted = self.light_classifier.get_classification(cv_image)

        #if DEBUG_MODE:
            # save image for debug purposes
            # now = rospy.Time.now()
            # cv2.rectangle(cv_image, (x-70, y-100), (x+70, y+100), (255, 0, 0), 2)
            # cv2.imwrite('tmp/' + str(predicted) + "_" + str(now) +'.jpg', cv_image)

        rospy.logdebug("traffic light state: %d", self.lights[0].state)
        return predicted


    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        wp_dist_tolerance = 300

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if self.pose and self.stop_line_waypoints:
            car_position = self.get_closest_waypoint(self.pose.pose, self.last_car_wp or 0)
            self.last_car_wp = car_position
            closest_light_wp = None
            # find the closest visible traffic light (if one exists)
            min_dist = float("inf")
            for i, light_position in enumerate(stop_line_positions):
                pos = self.waypoints.waypoints[car_position].pose.pose.position
                curr_dist = math.sqrt((pos.x-light_position[0])**2 + (pos.y-light_position[1])**2)
                light_wp = self.stop_line_waypoints[i]
                if curr_dist < min_dist and ((0 <= light_wp - car_position < wp_dist_tolerance) or (0 <= light_wp + len(self.waypoints.waypoints) - car_position < wp_dist_tolerance)):
                    min_dist = curr_dist
                    closest_index = i
                    closest_light_wp = light_wp
                elif curr_dist > min_dist:
                    break

            if closest_light_wp != None:
                # create object for closest light position
                rospy.logdebug("Traffic light waypoint: %d", closest_light_wp)
                rospy.logdebug("Car waypoint: %d", car_position)
                self.current_light_index = closest_index
                return closest_light_wp, self.get_light_state()

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
