#!/usr/bin/env python
import rospy
import mediapipe as mp
import cv2
import cv_bridge
from sensor_msgs.msg import Image
import numpy as np

class gesture_recog:
    def __init__(self):
        self.holistic = mp.solutions.holistic.Holistic()
        self.pose_connection  = mp.solutions.pose_connections.POSE_CONNECTIONS
        self.draw = mp.solutions.drawing_utils
        self.depth_image = None
        self.principal_point_x = 320.5
        self.principal_point_y = 240.5
        self.focal_length_x = 554.254691191187
        self.focal_length_y = 554.254691191187
        self.pub = rospy.Publisher('/skel_pub' , Image , queue_size=10)
        self.cvb = cv_bridge.CvBridge()
        self.sub = rospy.Subscriber('/camera/color/image_raw' , Image , self.cb)
        self.sub1 = rospy.Subscriber('/camera/depth/image_rect_raw' , Image , self.get_depth_image)

    def angle_calculator(self, joint_pose_array):
        # print(joint_pose_array[0].y < joint_pose_array[2].y)
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
                print('waving gesture')
                self.get_human_position(joint_pose_array[0].x , joint_pose_array[0].y)        
            # print (np.degrees(angle))

    def get_depth_image(self , data):
        self.depth_image = self.cvb.imgmsg_to_cv2( data)

    def get_human_position(self  , x , y):
        x_pixel = round(x *640)
        y_pixel = round(y* 480)
        Zinm = self.depth_image[y_pixel][x_pixel]
        Xinm = (x_pixel - self.principal_point_x) * (Zinm / self.focal_length_x)
        Yinm = (y_pixel - self.principal_point_y) * (Zinm / self.focal_length_y)
        print(Xinm , Yinm , Zinm)


    def cb(self , data):
        cv_image = self.cvb.imgmsg_to_cv2(data)
        cv_image = cv2.cvtColor(cv_image , cv2.COLOR_BGR2RGB)
        results = self.holistic.process(cv_image)
        cv_image = cv2.cvtColor(cv_image , cv2.COLOR_RGB2BGR)
        # print(cv_image.shape)
        # print(len(results.pose_landmarks.landmark))
        # print(results.pose_landmarks.landmark[14])
        self.angle_calculator([results.pose_landmarks.landmark[12] ,results.pose_landmarks.landmark[14] , results.pose_landmarks.landmark[16]] )
        self.draw.draw_landmarks(cv_image , results.pose_landmarks , self.pose_connection)
        self.pub.publish(self.cvb.cv2_to_imgmsg(cv_image , encoding='rgb8' ))


if __name__ == '__main__':
    rospy.init_node('gesture_node')
    obj = gesture_recog()
    rospy.spin()
