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
        self.img_sub = rospy.Subscriber('/image_raw', Image, self.image_callback, queue_size=10)
        #self.depth_sub = rospy.Subscriber('/rgbd_camera/depth/image_raw', Image, self.depth_callback, queue_size=10)
        #self.face_pub = rospy.Publisher('/face_position', Point, queue_size=10)
        
        self.face_angle_pub = rospy.Publisher('/face_angle_position', Point, queue_size=10)

        self.bridge_object = CvBridge()
        # self.face_center.x = 0
        # self.face_center.y = 0
        self.list1=[0]
        self.list2=[0]
        self.list3=[0]
        self.list4=[0]
        self.list5=[0]

    # def depth_callback(self, msg):
    #     try:
    #         self.depth_image = self.bridge_object.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    #     except CvBridgeError as e:
    #         rospy.logerr(e)

    def image_callback(self, msg):
        try:
            
            # face_center.x=0
            # face_center.y=0
            cv_image = self.bridge_object.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
            print(f"faces: {faces}")
            result_radians = Point()

            if ( self.list1[-1]== 0) and (self.list2[-1] == 0):
                angle_1=self.list3[-1]-0.001
                angle_z=round(angle_1,3)
                self.list3.append(angle_z)
                result_radians.z = self.list3[-1]
                #sleep(1)
            else:
                result_radians.z = self.list3[-1]

            
            if cv_image is not None:
                for (x, y, w, h) in faces:
                    face_center= Point()
                    face_center.x = x + w / 2
                    face_center.y = y + h / 2
                    self.list1.append(face_center.x)
                    self.list2.append(face_center.y)
                    # depth_value = self.depth_image[int(face_center.y), int(face_center.x)]
                    # face_center.z = depth_value  # Assign the depth value

                    # print(f"y value: {self.list2[-1]}")
                    if (self.list2[-1]>250):
                        # if (self.list1[-1])>self.list1[-2]:
                        #     result_radians.x=self.list5[-1]+0.01
                        #     self.list5.append(result_radians.x)
                        # else:
                        self.list3.append(round((self.list3[-1]+0.001),3))
                        result_radians.z=self.list3[-1]
                    #elif(self.list1[-1]<400) and -1.57<=self.list5[-1]<=1.57:
                    elif (self.list2[-1]<230):
                        # if (self.list1[-1])>self.list1[-2]:
                        #     result_radians.x=self.list5[-1]+0.01
                        #     self.list5.append(result_radians.x)
                        # else:
                        self.list3.append(round((self.list3[-1]-0.001),3))
                        result_radians.z=self.list3[-1]
                        
                    #     result_radians.x=self.list5[-1]-0.01
                    #     self.list5.append(result_radians.x)
                    else:
                        result_radians.z=self.list3[-1]
                        self.list3.append(result_radians.z)

                    print(f"x value: {self.list1[-1]}")
                    print(f"y value: {self.list2[-1]}")
                    #if (self.list1[-1]>400) and -1.57<=self.list5[-1]<=1.57:
                    if (self.list1[-1]>330):
                        # if (self.list1[-1])>self.list1[-2]:
                        #     result_radians.x=self.list5[-1]+0.01
                        #     self.list5.append(result_radians.x)
                        # else:
                        result_radians.x=self.list5[-1]+0.01
                        self.list5.append(result_radians.x)
                    #elif(self.list1[-1]<400) and -1.57<=self.list5[-1]<=1.57:
                    elif (self.list1[-1]<310):
                        # if (self.list1[-1])>self.list1[-2]:
                        #     result_radians.x=self.list5[-1]+0.01
                        #     self.list5.append(result_radians.x)
                        # else:
                        result_radians.x=self.list5[-1]-0.01
                        self.list5.append(result_radians.x)
                    #     result_radians.x=self.list5[-1]-0.01
                    #     self.list5.append(result_radians.x)
                    else:
                        result_radians.x=self.list5[-1]
                        self.list5.append(result_radians.x)

                    # ppi=0.03076/118
                    # x_difference=face_center.x -400
                    # # y_difference=300- face_center.y
                    # y_difference=face_center.y -300

                    # x_axis=ppi*x_difference
                    # y_axis=ppi*y_difference
                    # z_axis=face_center.z
                    # # Calculate arctangent (in radians)
                    # result_radians.x = math.atan(x_axis / z_axis)
                    # result_radians.y = math.atan(y_axis / z_axis)
                    # Convert radians to degrees if needed
                    # result_x_degrees = math.degrees(result_x_radians)
                    # result_y_degrees = math.degrees (result_y_radians)

                    #self.face_pub.publish(self.face_center)
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)

                    # result_radians.y=self.list3[-1]
                    # result_radians.z=self.list4[-1]

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