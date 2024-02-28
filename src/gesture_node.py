#!/usr/bin/env python
import rospy
import mediapipe as mp
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped , PointStamped , PoseWithCovarianceStamped , Pose2D
import numpy as np
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction , MoveBaseGoal
# import tf2_ros
# import tf2_geometry_msgs
import tf
import tf2_msgs.msg
# from pnp_cmd_ros import *
import moveit_commander
# import rospy
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import PointCloud2
from tf.transformations import quaternion_from_euler as qe
from tf.transformations import euler_from_quaternion as eq
from pal_interaction_msgs.msg import TtsActionGoal, TtsAction, TtsGoal
# import ros_numpy
import sys
import numpy

DES_DIST = 1

class gesture_recog:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.holistic = mp.solutions.holistic.Holistic()
        self.pose_connection  = mp.solutions.pose_connections.POSE_CONNECTIONS
        self.draw = mp.solutions.drawing_utils
        self.depth_image = None
        self.ac = actionlib.SimpleActionClient("/tts", TtsAction)
        self.ac.wait_for_server()
        self.goal = TtsActionGoal()
        # self.goal.goal.rawtext.text = "Hi_How_are_you"
        self.goal.goal.rawtext.lang_id = "en_GB"
        self.ac.send_goal(self.goal.goal)
        self.principal_point_x = 320.5
        self.principal_point_y = 240.5
        self.focal_length_x = 554.254691191187
        self.focal_length_y = 554.254691191187
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm = moveit_commander.MoveGroupCommander('arm')
        robot = moveit_commander.RobotCommander()
        # print(robot.get_group_names())
        # print(robot.get_current_state())
        self.reference_frame = 'arm_1_link'
        self.arm.set_pose_reference_frame(self.reference_frame)
        self.arm.allow_replanning(True)
        print(self.arm.get_end_effector_link())
        self.end_effector_link = self.arm.get_end_effector_link()
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = self.reference_frame
        # self.arm.set_named_target('init_pose')
        self.person_pose = PoseStamped()
        self.l = tf.TransformListener()
        self.robot_pose = Pose2D()
        # self.arm.set_named_target('init_pose')
        # self.arm.go()
        # self.l.waitForTransform('base_link' , 'map' , rospy.Time.now()-75 , rospy.Duration(4.0))
        self.p = PointStamped()
        self.goal_msg = MoveBaseGoal()
        self.client = actionlib.SimpleActionClient('/move_base' , MoveBaseAction)
        self.client.wait_for_server()
        self.counter  = 0
        self.table_joint_location = [0.27 , 0.30 , -1.47 , 1.63 , -0.70 , -0.11 , 0.94]
        # self.up_joint_location = [0.07 , 1.02 , -0.49 , 0.94 , -0.05 , 0.85 , -0.16]
        self.half_up_joint_location = [0.1 , 1.02 , -0.01 , 1.25 , -1.61 , 1.37 , 0.0]
        self.up_joint_location = [0.1 , 1.02 , -0.44 , 0.89 , -0.49 , 1.37 , 0.0 ]
        self.table1_joint_location  = [0.49 , 0.77 , -1.11 , 1.54 , -0.26 , 0.65 , -0.9]
        self.reach_cup_joint_location = [0.25 , 0.959 , -1.18 , 1.62 , -0.78 , 0.98 , -1.41]
        self.cup_joint_location  = [0.52 , 0.91 , -1.24 , 1.77 , -1.45 , 0.44 , -1.14 ]
        self.goal_msg.target_pose.header.frame_id ='map'
        self.p.header.frame_id = 'xtion_depth_optical_frame'
        self.person_pose.header.frame_id  = 'xtion_depth_optical_frame'
        self.pub = rospy.Publisher('/skel_pub' , Image , queue_size=10)
        self.pub1 = rospy.Publisher('/pose_pub' , PoseStamped , queue_size=10)
        self.cvb = cv_bridge.CvBridge()
        # self.move_to()
        # self.arm.go(self.half_up_joint_location , wait=True)
        # self.arm.go(self.up_joint_location , wait= True)
        # self.arm.go(self.table_joint_location , wait = True)
        # self.arm.go(self.table1_joint_location , wait = True)
        # self.arm.go(self.reach_cup_joint_location , wait=True)
        # self.arm.go(self.cup_joint_location , wait= True)
        # self.sub_time= rospy.Subscriber('/')
        self.robot_sub = rospy.Subscriber('/robot_pose' , PoseWithCovarianceStamped , self.robot_pose_cb)
        self.sub = rospy.Subscriber('/xtion/rgb/image_raw' , Image , self.cb)
        self.sub1 = rospy.Subscriber('/xtion/depth/image_raw' , Image , self.get_depth_image)

    def move_to(self):
        # a = [0.19992457376244457, -1.339974498508951, -0.19996410212184568, 1.9399594017573518, -1.5697936393455756, 1.3699139558317217, -9.088910280634702e-05]
        # print(self.arm.get_current_joint_values(wait=True))
        # a[0]  = a[0]
        # self.arm.go(self.up_joint_location , wait=True)
        self.arm.go(self.half_up_joint_location , wait=True)
        self.arm.go(self.up_joint_location , wait= True)
        # self.arm.go(self.table_joint_location , wait = True)
        # self.arm.go(self.table1_joint_location , wait = True)
        self.arm.go(self.reach_cup_joint_location , wait=True)
        self.arm.go(self.cup_joint_location , wait= True)
        rospy.sleep(30)
        self.goal.goal.rawtext.text = 'Please_take_the_bottle_from_my_hand'
        self.ac.send_goal(self.goal.goal  )
        rospy.sleep(200)

        # self.arm.set_joint_value_target(self.up_joint_location)
        # self.arm.go(a , wait=True)
        # self.location_wrt('arm_1_link' , 'arm_tool_link')
        # self.target_pose.pose.position.x = 0.699
        # self.target_pose.pose.position.y = -0.0399
        # self.target_pose.pose.position.z = 0.232
        # self.target_pose.pose.orientation.x = 0.760
        # self.target_pose.pose.orientation.y = -0.637
        # self.target_pose.pose.orientation.z = -0.075
        # self.target_pose.pose.orientation.w = 0.096
        # self.arm.set_start_state_to_current_state() 
        # self.arm.set_pose_target(self.target_pose , self.end_effector_link)
        # traj = self.arm.plan()
        # print(traj)
        # self.arm.execute(traj)
        # self.arm.go()
    def angle_calculator(self, joint_pose_array):
        # print(joint_pose_array[0].y < joint_pose_array[2].y)
        # print(self.counter)
        if joint_pose_array[0].y > joint_pose_array[2].y:
            a = np.array([joint_pose_array[0].x , joint_pose_array[0].y ])#, joint_pose_array[0].z])
            b = np.array([joint_pose_array[1].x , joint_pose_array[1].y ])#, joint_pose_array[1].z])
            c = np.array([joint_pose_array[2].x , joint_pose_array[2].y ])#, joint_pose_array[2].z])
            ba = a - b
            bc = c - b

            cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
            angle = np.degrees(np.arccos(cosine_angle))
            # print(angle)
            if (angle >50 and angle<130):
                self.counter += 1
                print('waving gesture')
                # if self.counter >20:
                #     self.get_human_position(joint_pose_array[0].x , joint_pose_array[0].y)
                    # if self.counter == 30:
                        # print('Moved to person' , goalState , result)
                #         self.goto_person([self.person_pose.pose.position.x , self.person_pose.pose.position.y , self.person_pose.pose.position.z])
                        # self.counter = 0
                return True
            else : 
                self.counter = 0
                return False        
            # print (np.degrees(angle))

    def get_depth_image(self , data):
        self.depth_image = self.cvb.imgmsg_to_cv2( data)
        self.start()

    def start(self):
        rospy.sleep(30)
        self.goal.goal.rawtext.text = 'Hi,_I_am_here_to_help_you._what_do_you_need?'
        self.ac.send_goal(self.goal.goal)
        rospy.sleep(15)
        self.goal.goal.rawtext.text = 'Okay_I_will_get_you_the_green_bottle_on_the_table'
        self.ac.send_goal(self.goal.goal  )
        rospy.sleep(30)
        self.move_to()
    def get_human_position(self  , x , y):
        x_pixel = round(x *640)
        y_pixel = round(y* 480)
        Zinm = self.depth_image[y_pixel][x_pixel]
        Xinm = (x_pixel - self.principal_point_x) * (Zinm / self.focal_length_x)
        Yinm = (y_pixel - self.principal_point_y) * (Zinm / self.focal_length_y)
        self.person_pose.pose.position.x = Xinm/1000
        self.person_pose.pose.position.y = Yinm/1000
        self.person_pose.pose.position.z = Zinm/1000
        # self.pose.pose.orientation.z = 1.57   
        self.pub1.publish(self.person_pose)
        # print(Xinm , Yinm , Zinm)
    def location_wrt(self, link1_name , link2_name):
        point = PoseStamped()
        point.header.frame_id = link2_name
        point = self.l.transformPose(link1_name , point)
        print(point.pose)
        # print(point)
    def goto_person(self, person_position_wrt_camera ):
        self.p.point.x = person_position_wrt_camera[0]
        self.p.point.y = person_position_wrt_camera[1]
        self.p.point.z = person_position_wrt_camera[2]
        human_wrt_map = self.l.transformPoint('map' , self.p)
        person_pos = np.array([human_wrt_map.point.x , human_wrt_map.point.y])
        robot_pos = np.array([self.robot_pose.x , self.robot_pose.y])
        # print(person_pos)
        vector_to_person = person_pos - robot_pos

        # Calculate the distance from the robot to the person
        distance_to_person = math.sqrt(vector_to_person[0]**2 + vector_to_person[1]**2)

        # Normalize the vector to the desired distance
        normalized_vector = vector_to_person / distance_to_person

        # Calculate the orientation needed to reach the person
        goal_orientation = math.atan2(normalized_vector[1], normalized_vector[0])

        # Calculate the goal position based on the desired distance
        goal_position = person_pos - DES_DIST * normalized_vector

        # return goal_position, goal_orientation    
        self.goal_msg.target_pose.header.stamp = rospy.Time.now()
        self.goal_msg.target_pose.pose.position.x = goal_position[0]
        self.goal_msg.target_pose.pose.position.y = goal_position[1]
        self.goal_msg.target_pose.pose.orientation.z = math.sin(goal_orientation/2)
        self.goal_msg.target_pose.pose.orientation.w = math.cos(goal_orientation/2)
        self.client.send_goal(self.goal_msg , done_cb=self.on_goto_done)


    # def on_speak_comple(self, goalState , result)
        
        
    def on_goto_done(self , goalState , result):
        print('Moved to person' , goalState , result)
        self.goal.goal.rawtext.text = 'Hi,_I_am_here_to_help_you._what_do_you_need?'
        self.ac.send_goal(self.goal.goal)
        rospy.sleep(15)
        self.goal.goal.rawtext.text = 'Okay_I_will_get_you_the_green_bottle_on_the_table'
        self.ac.send_goal(self.goal.goal  )
        rospy.sleep(120)
        self.move_to()
    def cb(self , data):
        cv_image = self.cvb.imgmsg_to_cv2(data)
        cv_image = cv2.cvtColor(cv_image , cv2.COLOR_BGR2RGB)
        results = self.holistic.process(cv_image)
        cv_image = cv2.cvtColor(cv_image , cv2.COLOR_RGB2BGR)
        # print(cv_image.shape)
        # print(len(results.pose_landmarks.landmark))
        # print(results.pose_landmarks.landmark[14])
        # print(type(results.pose_landmarks)  == None)
        if not (type(results.pose_landmarks)  == type(None)):
            if (self.angle_calculator([results.pose_landmarks.landmark[12] ,results.pose_landmarks.landmark[14] , results.pose_landmarks.landmark[16]] )):
                cv2.putText(cv_image , 'Wave Gesture' , (50 , 200) , cv2.FONT_HERSHEY_SIMPLEX , 1 , (120 , 120 , 120 )  , 2)

            self.draw.draw_landmarks(cv_image , results.pose_landmarks , self.pose_connection)
        self.pub.publish(self.cvb.cv2_to_imgmsg(cv_image , encoding='rgb8' ))

    def robot_pose_cb(self, r: PoseWithCovarianceStamped):
        q = (
            r.pose.pose.orientation.x,
            r.pose.pose.orientation.y,
            r.pose.pose.orientation.z,
            r.pose.pose.orientation.w
        )
        
        m = tf.transformations.quaternion_matrix(q)
        
        self.robot_pose.x = r.pose.pose.position.x
        self.robot_pose.y = r.pose.pose.position.y
        self.robot_pose.theta = tf.transformations.euler_from_matrix(m)[2]



if __name__ == '__main__':
    # p = PNPCmd()
    # p.begin()
    rospy.init_node('gesture_node')
    obj = gesture_recog()
    # obj.move_to() 
    # obj.move_to([ -0.13 ,0.08 ,-0.25])
    rospy.spin()
