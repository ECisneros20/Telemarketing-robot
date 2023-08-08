#!/usr/bin/env python3

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from cv2.aruco as aruco
import sys

bridge = CvBridge()

def image_callback(ros_image):
  print('got an image')
  global bridge
  global id_pub
  global image_pub
  #convert ros_image into an opencv-compatible image
  try:
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
      print(e)

  markers_img, ids_list = detect_aruco(cv_image)

  if ids_list is None:
    id_pub.publish(ids_list)
  else:
    ids_str = ''.join(str(e) for e in ids_list)
    id_pub.publish(ids_str)

  try:
    image_pub.publish(bridge.cv2_to_imgmsg(markers_img, "bgr8"))
  except CvBridgeError as e:
    print(e)

  cv2.imshow("Image window", cv_image)
  cv2.waitKey(3)

def detect_aruco(self,img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
    output = aruco.drawDetectedMarkers(img, corners, ids)  # detect the sruco markers and display its aruco id.
    return output, ids

def main(args):
  global image_pub
  global id_pub
  rospy.init_node('image_converter', anonymous=True)
  image_topic="/camera_image/image_raw"#"/camera/rgb/image_raw/compressed"
  #for usb cam
  #image_topic="/usb_cam/image_raw"
  image_sub = rospy.Subscriber(image_topic,Image, image_callback)
  image_pub = rospy.Publisher("/detected_markers",Image, queue_size=1)
  id_pub = rospy.Publisher("/arudo_ID", String, queue_size=1)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)