#!/usr/bin/env python

import rospy
import rospkg
import actionlib
from utils import Utils
from POIs_generator import PoI
from map import Map
from cv_bridge import CvBridge, CvBridgeError
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import Image, CameraInfo
import message_filters
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
import sys
class storePlanner:

    def __init__(self, args):

        #we need: static map, PoIs file

        rospack = rospkg.RosPack()
        self.base_path = rospack.get_path('store-planner') + '/'
        self.foldername = str(args[1])

        self.map_ = Map(self.base_path + 'maps/' + self.foldername )
        #initial position is given in pixels (image coordinates)
        init_x, init_y, init_yaw = Utils.parse_param_file(self.base_path + 'config/params.json')[0:3]
        #we need to invert y axis and transform in meters
        init_x_meters, init_y_meters = self.map_.image2map(init_x,init_y)

        self.PoIs = self.parsePoIs(self.base_path + 'results/' + self.foldername + '/final_path.txt')

        self.PoIs.pop(0) #removing depot since the robot spawns already there

        self.bridge = CvBridge()

        self.goal_cnt = 0

        #ROS interface
        #subscribers
        #get cameras info
        #TODO parametrize them into a .yaml file
        # all is packed as right cameras if one wants to add also 3 cameras on the left side of robot
        self.tr_cam_rgb_info = rospy.wait_for_message("/tr_rgbd_camera/rgb/camera_info", CameraInfo) 
        self.mr_cam_rgb_info = rospy.wait_for_message("/mr_rgbd_camera/rgb/camera_info", CameraInfo)
        self.br_cam_rgb_info = rospy.wait_for_message("/br_rgbd_camera/rgb/camera_info", CameraInfo)

        #synchronize cameras image by message_filters 
        self.tr_rgb_img_sub = message_filters.Subscriber("/tr_rgbd_camera/rgb/image_raw",Image)
        self.mr_rgb_img_sub = message_filters.Subscriber("/mr_rgbd_camera/rgb/image_raw",Image)
        self.br_rgb_img_sub = message_filters.Subscriber("/br_rgbd_camera/rgb/image_raw",Image)

        self.ts_right_cameras = message_filters.TimeSynchronizer([self.tr_rgb_img_sub, self.mr_rgb_img_sub, self.br_rgb_img_sub], 10)
        self.ts_right_cameras.registerCallback(self.right_cameras_cb)

        self.tr_rgb_img_msg = Image()     
        self.mr_rgb_img_msg = Image()
        self.br_rgb_img_msg = Image()

        self.right_capture_time = 0

        self.pose_sub = rospy.Subscriber("/robot_pose",PoseWithCovarianceStamped, self.pose_cb) #try gazebo ground truth eventually
        self.pose = PoseWithCovarianceStamped()
        # self.laser_scan_sub = rospy.Subscriber("/scan",LaserScan, self.laser_cb)
        # self.laser_msg = LaserScan()

        self.cmd_vel_pub = rospy.Publisher("/mobile_base_controller/cmd_vel",Twist, queue_size=1)

        #This publisher is used to spawn the robot on the docking station
        self.init_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.initial_pose = PoseWithCovarianceStamped()

        self.initial_pose.header.frame_id = "map"

        self.initial_pose.pose.pose.position.x = init_x_meters
        self.initial_pose.pose.pose.position.y = init_y_meters
        self.initial_pose.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, init_yaw)
        self.initial_pose.pose.pose.orientation.x = q[0]
        self.initial_pose.pose.pose.orientation.y = q[1]
        self.initial_pose.pose.pose.orientation.z = q[2]
        self.initial_pose.pose.pose.orientation.w = q[3]

        #wait for gazebo and set initial pose of the robot
        self.set_init_pose(self.initial_pose)

        #action for move_base
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        wait = self.client.wait_for_server()
        if not wait:
            rospy.signal_shutdown("Could not find move_base Action Server")

        
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = 0.0
        self.goal.target_pose.pose.orientation.y = 0.0
        self.goal.target_pose.pose.orientation.z = 0.0
        self.goal.target_pose.pose.orientation.w = 1.0 #not sending orientation, we will rotate to align at the end

        #since when the robot initial pose is changed, local costmap gets erased
        self.clear_costmaps()

        


    def run(self):
        
        print('Running Planner!')

        #removing capturing folder if exists
        Utils.rm_dir_and_contents(self.base_path + 'acquisitions/' + str(self.foldername))
        Utils.create_dir(self.base_path + 'acquisitions/' + str(self.foldername))
        
        for poi in self.PoIs: 
            x, y = self.map_.image2map(poi.x, poi.y) #PoIs are are always in pixel originally
            #NOT IMPLEMENTED
            # x, y = self.utils.shift_goal((x,y),self.map_shift)
            self.send_goal(x, y)
            self.goal_cnt = self.goal_cnt + 1
            if poi.shelf_id != 0:
                self.align_robot(poi.theta)
                self.capture(poi.shelf_id) 

        rospy.signal_shutdown("Reached last goal, shutting down wpoint generator node")
        rospy.logdebug("Reached last goal, shutting down wpoint generator node")
        rospy.signal_shutdown('Killing store_planner_node. All clear!')
    
    def parsePoIs(self,filepath):
        PoIs = list()
        with open(filepath,'r') as f:
            for line in f:
                poi = line.split(",")
                PoIs.append(PoI(int(poi[0]),int(poi[1]),int(poi[3]),int(poi[2])))

        return PoIs

    def send_goal(self, x, y):
        print("Sending waypoint!")
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y

        #send the current goal
        self.client.send_goal(self.goal,done_cb=self.done_cb) #it is possible to add the others callbacks directly from here
        
        result = self.client.wait_for_result() #blocking execution until some result comes

        if not result:
            rospy.logerr("Something went wrong with the waypoint number: !", self.goal_cnt)
            rospy.signal_shutdown("Something went wrong with the waypoint number: !", self.goal_cnt)
        else:
            return self.client.get_result()

    def pose_cb(self,data):
        self.pose = data
        

    def done_cb(self, status, result):
        #status gives info on preemptions etc on the goal
        print("status: ", status)
        print("result:" ,result)

    def clear_costmaps(self):
        rospy.wait_for_service('/move_base/clear_costmaps')
    
        clear_ = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        clear_call = clear_()
    
    def set_init_pose(self, init_pose):

        #change it in gazebo
        state_msg = ModelState()
        state_msg.model_name = 'pmb2'
        state_msg.pose = init_pose.pose.pose

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        #change it also on amcl
        rospy.wait_for_service('/amcl/set_parameters') #this is needed just to be sure amcl is up
        rospy.sleep(1.0)
        self.init_pose_pub.publish(self.initial_pose)

    def align_robot(self,theta_deg):
        print("Aligning robot.")

        theta = Utils.deg2rad(theta_deg)
        
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        #get pose and turning direction sign
        curr_x = self.pose.pose.pose.position.x
        curr_y = self.pose.pose.pose.position.y
        # curr_yaw = self.pose.pose.pose.orientation.z

        qx = self.pose.pose.pose.orientation.x
        qy = self.pose.pose.pose.orientation.y
        qz = self.pose.pose.pose.orientation.z
        qw = self.pose.pose.pose.orientation.w

        curr_yaw = euler_from_quaternion([qx,qy,qz,qw])[2]

        epsilon = 0.02 #around 1 degree

        #TODO: calculate the "shorter" rotation direction (for now it rotates always on positive direction)

        # w_direction = 1 if abs(theta) >= Utils.pi()/2 else -1

        msg.angular.z = 0.15 #w_direction*0.2 #moving very slowly

        while not rospy.is_shutdown() and  abs(abs(theta) - abs(curr_yaw)) > epsilon:

            self.cmd_vel_pub.publish(msg)

            qx = self.pose.pose.pose.orientation.x
            qy = self.pose.pose.pose.orientation.y
            qz = self.pose.pose.pose.orientation.z
            qw = self.pose.pose.pose.orientation.w

            curr_yaw = euler_from_quaternion([qx,qy,qz,qw])[2]

        
            # curr_yaw = self.pose.pose.pose.orientation.z
            # theta = Utils.get_robot_obj_angle(curr_x,curr_y,curr_yaw,object_position)

        # capture_side = "left" if w_direction == -1 else "right"

        # return capture_side

    def capture(self, shelf_id):

        capture_path = self.base_path + 'acquisitions/' + str(self.foldername) + '/' + str(self.goal_cnt)
        
        Utils.create_dir(capture_path)
        
        capture_time = 0
        top_cam_info = 0
        middle_cam_info = 0
        bottom_cam_info = 0
        
        #save images
        capture_time = self.right_capture_time
        Utils.save_image( capture_path + '/top_rgb.png', self.tr_rgb_img)   
        Utils.save_image( capture_path + '/middle_rgb.png', self.mr_rgb_img)  
        Utils.save_image(capture_path + '/bottom_rgb.png', self.br_rgb_img) 
        top_cam_info = self.tr_cam_rgb_info
        middle_cam_info = self.mr_cam_rgb_info
        bottom_cam_info = self.br_cam_rgb_info


        #save pose and shelf data
        x = self.pose.pose.pose.position.x
        y = self.pose.pose.pose.position.y

        q_x = self.pose.pose.pose.orientation.x
        q_y = self.pose.pose.pose.orientation.y
        q_z = self.pose.pose.pose.orientation.z
        q_w = self.pose.pose.pose.orientation.w

        yaw = euler_from_quaternion([q_x,q_y,q_z,q_w])[2] #self.pose.pose.pose.orientation.z
        #NOT IMPLEMENTED
        # x,y = Utils.shift_goal((x,y),self.map_shift,-1) #restore goal position wrt map origin
        Utils.save_pose_metadata(capture_path + '/pose.yaml',capture_time,x,y,yaw , shelf_id)
        Utils.save_camera_metadata(capture_path + '/top_camera.yaml',capture_time,top_cam_info)
        Utils.save_camera_metadata(capture_path + '/middle_camera.yaml',capture_time,middle_cam_info)
        Utils.save_camera_metadata(capture_path + '/bottom_camera.yaml',capture_time,bottom_cam_info)

        print("Saved waypoint number " + str(self.goal_cnt) + " images and pose.")

    def right_cameras_cb(self, top_img, middle_img, bottom_img):

        self.right_capture_time = top_img.header.stamp

        try:
            self.tr_rgb_img = self.bridge.imgmsg_to_cv2(top_img, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            self.mr_rgb_img = self.bridge.imgmsg_to_cv2(middle_img, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            self.br_rgb_img = self.bridge.imgmsg_to_cv2(bottom_img, "bgr8")
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':

    args = rospy.myargv()
    
    if len(args) != 2:
        Utils.print_usage(1)
    rospy.init_node('store_planner_node', anonymous=True)
    
    store_planner = storePlanner(args)
    store_planner.run()

    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down store_planner_node")
