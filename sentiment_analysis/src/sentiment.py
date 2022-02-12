#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np

import sys

#sys.path.insert(0, "/home/enzoc/.local/lib/python3.6/site-packages/tensorflow")

#import tensorflow as tf
#from tensorflow.keras.models import load_model

bridge = CvBridge()

path = './sentiment_analysis/miniXception.h5'

#new_model = load_model(path)

#new_model.summary()

def image_callback(ros_image):

    emo_dict = {0: 'happy', 1: 'sad', 2: 'surprise', 3: 'fear', 4: 'disgust-contempt', 5: 'anger'}

    global bridge

    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)

    width = 150
    height = 150

    cv2.imshow('Frame', cv_image)
    cv2.waitKey(1)
  
'''
    while (cap.isOpened()):
        ret, frame = cap.read()
        if ret == True:
            frame_resized = cv2.resize(frame, (width, height))
            frame_resized = frame_resized[None,:,:]

            predict = new_model.predict(frame_resized)
            predict = predict[0,:]

            print(predict)
            sentimiento = np.where(predict == max(predict))[0][0]

            print('The sentiment detect is: ' + emo_dict[sentimiento])

            cv2.imshow('Frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break
    cap.release()'''

def camera2Recv():

    rospy.init_node('sentiment_analysis_img', anonymous=True)
    sub_image = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':

    camera2Recv()