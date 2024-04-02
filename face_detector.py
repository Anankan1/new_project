import rclpy
#import os
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import cv2
from mtcnn import MTCNN
from cv_bridge import CvBridge
import pyrealsense2 as rs # need to install pyrealsense2 library
import numpy as np
import math

class FaceDetector(Node):
    def __init__(self):
        super().__init__('face_detector')
        self.get_logger().info('Detecting faces...')

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)  # Open the default webcam (0)
        # Get the path to the current script
        #script_dir = os.path.dirname(os.path.abspath(__file__))

        # Load the pre-trained face detection model
        # cascade_path = os.path.join(script_dir, 'haarcascade_frontalface_default.xml')
        # print(f"Cascade path: {cascade_path}")
        # self.face_cascade = cv2.CascadeClassifier(cascade_path)
        #self.face_cascade = cv2.CascadeClassifier (os.path.join(os.path.dirname(__file__),'haarcascade_frontalface_default.xml'))
        self.detector = MTCNN()


        # Create a publisher for the video feed
        self.pub = self.create_publisher(Image, '/webcam_feed', 10)
        # Create a timer to read frames from the webcam and publish them
        self.timer = self.create_timer(0.033, self.publish_frame)  # 30 FPS

        self.img_sub = self.create_subscription(Image, '/webcam_feed', self.image_callback, 10)
        self.face_pub = self.create_publisher(Point, '/face_position', 10)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(img_msg)
        else:
            self.get_logger().warn('Failed to capture frame from webcam')

    def capture_distance_to_object(self):
    # Initialize the RealSense pipeline
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # Start streaming
        pipeline.start(config)

        try:
            while True:
                # Wait for a coherent pair of frames: depth and color
                frames = pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()

                # Convert depth frame to numpy array
                depth_image = np.asanyarray(depth_frame.get_data())

                # Calculate distance in meters
                depth_scale = pipeline.get_active_profile().get_device().first_depth_sensor().get_depth_scale()
                distances = depth_image * depth_scale

                # Calculate distance to object at center of the image
                center_x = depth_frame.get_width() // 2
                center_y = depth_frame.get_height() // 2
                distance_to_center = distances[center_y, center_x]

                # # Calculate the distance in meters
                # depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                # distance = depth_value * depth_intrin.depth_scale

                # depth_value = depth_frame.get_distance(center_x, center_y)
                return center_x

        finally:
            # Stop streaming
            pipeline.stop()


    known_distance= capture_distance_to_object() #know distance in centimeters, it will give by the camera package

    ppi=56 #need to have a value
    #convert pixel values to centimeters
    def pixel_to_CM_conversion(pixel,ppi):
        centimeter=(pixel*2.54)/(ppi*100)
        return centimeter
    
    def find_angle(self,d1,d2):
        width= d1-d2
        # ppi=ppi
        width_in_cm=self.pixel_to_CM_conversion(width,self.ppi)
        angle_value=width_in_cm / self.known_distance
        result_radians = math.atan(angle_value)
        angle_degree= math.degrees(result_radians)
        return angle_degree
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
             # Convert frame to RGB (required by MTCNN)
            rgb_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # Detect faces in the frame
            faces = self.detector.detect_faces(rgb_frame)
            list1=[]
            # gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            # faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
            if faces:
                face = faces[0]
                x, y, w, h = face['box']
                face_center = Point()
                face_center.x = x + w / 2
                face_center.y = y + h / 2
                list1.append([face_center.x,face_center.y])
                self.face_pub.publish(face_center)

                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

                cv2.imshow('Face Detection', cv_image)
                cv2.waitKey(1)
            else:
                return None
            
            max_length=608
            if len(list1)>=2:
                for i in range(0,max_length,1):
                    if faces:
                        x_rotation= self.find_angle(list1[-1][0],list1[-2][0])
                        y_rotation =self.find_angle(list1[-2][1],list1[-2][1])
                        y_lift=i
                        return (x_rotation, y_rotation,y_lift)
                    else:
                        continue
        
            
            # Draw rectangles around the faces
            # for face in faces:
            #     x, y, w, h = face['box']
            #     face_center = Point()
            #     face_center.x = x + w / 2
            #     face_center.y = y + h / 2
            #     self.face_pub.publish(face_center)
            #     cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    face_detector = FaceDetector()
    rclpy.spin(face_detector)
    face_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()