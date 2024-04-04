#!/usr/bin/env python3

#this detects the face in 3d dimension
import rospy
import os
import cv2
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
        self.face_pub = rospy.Publisher('/face_position', Point, queue_size=10)
        self.bridge_object = CvBridge()

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge_object.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr(e)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)

            if self.depth_image is not None:
                for (x, y, w, h) in faces:
                    face_center = Point()
                    face_center.x = x + w / 2
                    face_center.y = y + h / 2

                    #calculate the depth value
                    depth_value = self.depth_image[int(face_center.y), int(face_center.x)]
                    face_center.z = depth_value  # Assign the depth value

                    self.face_pub.publish(face_center)
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)

            cv2.imshow('Face Detection', cv_image)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f'Error processing image: {e}')

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    face_detector = FaceDetector()
    face_detector.run()