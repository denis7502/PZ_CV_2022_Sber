import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import torch
from facenet_pytorch import MTCNN, InceptionResnetV1
from scipy.spatial import distance


class image_converter(Node):
    def __init__(self):
      super().__init__("convert_node")
      self.get_logger().info("Starting work")
      self.image_sub = self.create_subscription(Image, "/image_raw",       self.callback, 1)
      self.image_pub = self.create_publisher(Int8, "/motion_flag", 1)
      #self.image_pub = rospy.Publisher("image_topic_2",Image)
      self.model = torch.hub.load('ultralytics/yolov5', 'yolov5x', force_reload=True).cuda()
      self.model_2  = MTCNN(keep_all=True)
      self.bridge = CvBridge()
      #self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)

    def _check_motion(self, frame):
        cup = False
        # Capture frame-by-frame
        frame = cv2.rectangle(frame, (0,0), (frame.shape[1], frame.shape[0]),  (0, 0, 255), 10)
        # Our operations on the frame come here
        gray = self.model(frame.copy())
        obj = gray.pandas().xyxy
        boxes, pt = self.model_2.detect(frame.copy())
        results = 0
        if not isinstance(boxes, type(None)):
            if len(boxes) > 0:
                boxes = boxes[0]
            if len(obj) > 0:
                for i in obj:
                    for j in zip(i['xmin'], i['ymin'], i['xmax'], i['ymax'], i['name']):
                        if j[4] == 'cup':
                            cup_min, cup_max = (int(j[0]), int(j[1])), (int(j[2]), int(j[3]))
                            frame = cv2.rectangle(frame, cup_min, cup_max,  (0, 255, 255), 5)
                            cup_x, cup_y = int((cup_min[0] + cup_max[0])/2), int((cup_min[1] + cup_max[1])/2)
                            frame = cv2.putText(frame, 'Cup', (cup_x, cup_y), cv2.FONT_HERSHEY_SIMPLEX, 
                                        1, (255, 0, 0), 2, cv2.LINE_AA)
                            cup = True
            if len(boxes) > 0:
                face_min, face_max = (int(boxes[0]), int(boxes[1])), (int(boxes[2]), int(boxes[3]))
                face_x, face_y = int((face_min[0] + face_max[0])/2), int((face_min[1] + face_max[1])/2)
                frame = cv2.rectangle(frame, face_min, face_max, (255, 255, 255), 5)
                frame = cv2.putText(frame, 'Face', (face_x, face_y), cv2.FONT_HERSHEY_SIMPLEX, 
                                    1, (255, 0, 0), 2, cv2.LINE_AA)
            if cup:
                if distance.euclidean((face_x, face_y), (cup_x, cup_y)) <= abs(cup_min[1] - cup_max[1]):
                    frame = cv2.rectangle(frame, (0,0), (frame.shape[1], frame.shape[0]),  (0, 255, 0), 10)
                    results = 1
            else:
                frame = cv2.rectangle(frame, (0,0), (frame.shape[1], frame.shape[0]),  (0, 0, 255), 10)
                results = 0
                # Display the resulting frame

        return frame, results

    def callback(self,data):
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          print(e)

        image, results = self._check_motion(cv_image)
        cv2.imshow("Image window", image)
        cv2.waitKey(3)
        msg = Int8()
        msg.data = int(results)

        try:
          self.image_pub.publish(msg)
        except CvBridgeError as e:
          print(e)

def main():
    rclpy.init(args=None)
    ic = image_converter()
    #rospy.init_node('image_converter', anonymous=True)
    try:
      rclpy.spin(ic)
    except KeyboardInterrupt:
      print("Shutting down")
      cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


