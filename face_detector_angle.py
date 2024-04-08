#!/usr/bin/env python3

#this code returns the angle of the face movement
from time import sleep
import rospy
import os
import cv2
import math
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

class FaceDetector:
    def __init__(self):
        rospy.init_node('face_detector', anonymous=True)
        rospy.loginfo('Detecting faces...')

        self.face_cascade = cv2.CascadeClassifier (os.path.join(os.path.dirname(__file__),'haarcascade_frontalface_default.xml'))
        self.img_sub = rospy.Subscriber('/rgbd_camera/rgb/image_raw', Image, self.image_callback, queue_size=10)
        self.depth_sub = rospy.Subscriber('/rgbd_camera/depth/image_raw', Image, self.depth_callback, queue_size=10)
        #self.face_pub = rospy.Publisher('/face_position', Point, queue_size=10)
        self.face_angle_pub = rospy.Publisher('/face_angle_position', Point, queue_size=10)
        self.bridge_object = CvBridge()

        # self.face_center = Point()
        # self.face_center.x = 0
        # self.face_center.y = 0

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge_object.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr(e)

    def image_callback(self, msg):
        try:
            list1=[0]
            list2=[0]
            
            # face_center.x=0
            # face_center.y=0
            cv_image = self.bridge_object.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
            print(f"faces: {faces}")
            result_radians = Point()
            list3=[0]

            if ( list1[-1]== 0) and (list2[-1] == 0):
                angle_z=list3[-1]-0.1
                list3.append(angle_z)
                result_radians.z =list3[-1]
                sleep()
            elif (list2[-1] <=300):
                angle_z=list3[-1]-0.1
                list3.append(angle_z)
                result_radians.z =list3[-1]
                sleep()
            else:
                result_radians.z=list3[-1]

            print(f"z: {result_radians.z}")
            if self.depth_image is not None:
                for (x, y, w, h) in faces:
                    face_center= Point()
                    face_center.x = x + w / 2
                    face_center.y = y + h / 2
                    list1.append(face_center.x)
                    list2.append(face_center.y)
                    #calculate the depth value
                    depth_value = self.depth_image[int(self.face_center.y), int(self.face_center.x)]
                    self.face_center.z = depth_value  # Assign the depth value
                    if (face_center.x>400):
                        result_radians.x=result_radians.x+0.01
                    elif( face_center.x<400):
                        result_radians.x=result_radians.x-0.01

                    ppi=0.03076/118
                    x_difference=face_center.x -400
                    y_difference=300- face_center.y
                    x_axis=ppi*x_difference
                    y_axis=ppi*y_difference
                    z_axis=face_center.z
                    # Calculate arctangent (in radians)
                    result_radians.x = math.atan(x_axis / z_axis)
                    result_radians.y = math.atan(y_axis / z_axis)
                    # Convert radians to degrees if needed
                    # result_x_degrees = math.degrees(result_x_radians)
                    # result_y_degrees = math.degrees(result_y_radians)

                    #self.face_pub.publish(self.face_center)
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)

            print(f"angles: {result_radians}")
            self.face_angle_pub.publish(result_radians)
            cv2.imshow('Face Detection', cv_image)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f'Error processing image: {e}')

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    face_detector = FaceDetector()
    face_detector.run()