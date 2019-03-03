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
from scipy.spatial import KDTree

STATE_COUNT_THRESHOLD = 3


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.waypoints_2d = None
        self.waypoint_tree = None
        
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.has_image = False
        self.img_count = 0

        #for data collect
        self.data_collect = False
        self.red_light_num = 0
        self.green_light_num = 0
        self.yellow_light_num = 0
        self.none_light_num = 0
        
        #for debug :image open
        self.camera_open = False   
   
        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)




        
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.data_collect == True:
            light_wp, state = self.process_traffic_lights()
            
        #rospy.logwarn("tl_detector pose_cb update {} -- {}".format(light_wp, state))
        if self.camera_open == False:
            light_wp, state = self.process_traffic_lights()
            #rospy.logwarn("tl_detector pose_cb 1 ")
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

    def waypoints_cb(self, waypoints):

        self.waypoints = waypoints
        #self.base_waypoints = waypoints
        #rospy.logwarn("tl_detector waypoints_cb 1 ")
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        #rospy.logwarn("tl_detector traffic_cb update ")
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint
        Args:
            msg (Image): image from car-mounted camera
        """
        self.img_count = self.img_count+1
        #predicted every 5 images
        if (self.img_count-1) %4 != 0 :
            return
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
        
        if self.data_collect == True:
            #save img
            self.img_count = self.img_count+1
            if self.img_count % 5 == 0:
                cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
                filename = "../../../detector/imgs/sim_imgs/"
                
                self.none_light_num = self.none_light_num+1
                cv2.imwrite(filename+"none/none_"+str(self.none_light_num)+".jpg", cv_image)
                return     
                if state == TrafficLight.RED :
                    self.red_light_num = self.red_light_num+1
                    cv2.imwrite(filename+"red/red_"+str(self.red_light_num)+".jpg", cv_image)
                elif state == TrafficLight.GREEN :
                    self.green_light_num = self.green_light_num+1
                    cv2.imwrite(filename+"green/green_"+str(self.green_light_num)+".jpg", cv_image)
                elif state == TrafficLight.YELLOW :
                    self.yellow_light_num = self.yellow_light_num+1
                    cv2.imwrite(filename+"yellow/yellow_"+str(self.yellow_light_num)+".jpg", cv_image)
                else:
                    self.none_light_num = self.none_light_num+1
                    cv2.imwrite(filename+"none/none_"+str(self.none_light_num)+".jpg", cv_image)
                

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        #TODO implement
        if self.waypoint_tree:
            closest_idx = self.waypoint_tree.query([x, y],1)[1]
            return closest_idx
        else:
            return -1

    def get_light_state(self, light):
        """Determines the current color of the traffic light
        Args:
            light (TrafficLight): light to classify
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        #for testing
        if self.camera_open == False:
            return light.state
            #return TrafficLight.GREEN
        
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        light_state = self.light_classifier.get_classification(cv_image)
        if light_state  != light.state:
            rospy.logwarn("get_light_state==> classfier state {} -- real state {}".format(light_state , light.state))
        
        return light_state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color
        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        #light = None
        closest_light = None
        line_wp_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            #car_position = self.get_closest_waypoint(self.pose.pose)
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)
            if car_wp_idx < 0:
                return -1, TrafficLight.UNKNOWN
            
            diff = len(self.waypoints.waypoints)
            for i,light in enumerate(self.lights):
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                if temp_wp_idx < 0:
                    continue
                #find closest stop line waypoint index
                d = temp_wp_idx - car_wp_idx
                if d >= 0 and d < diff:
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx
                

        #TODO find the closest visible traffic light (if one exists)

        if closest_light:
            state = self.get_light_state(closest_light)
            return line_wp_idx, state
        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
