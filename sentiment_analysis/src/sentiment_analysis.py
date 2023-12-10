#!/usr/bin/env python3
# Check camera capabilities: v4l2-ctl --device=0 --list-formats-ext

import tensorflow as tf
from tensorflow.keras.models import load_model

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

import cv2
import numpy as np

# Publisher (1)
# /sentiment_detected     -   String     -   to GUI's remote PC     -   String with the detected sentiment and its percentage of confidence

# Subscriber (1)
# /usb_cam/image_raw      -   Image      -   from usb camera        -   Raw image for detecting sentiment purposes

class SentimentAnalysis:

    def __init__(self, path):

        # Model setup
        self.cascade_path = path + "/include/haarcascade_frontalface_alt.xml"
        self.classifier = cv2.CascadeClassifier(self.cascade_path)
        self.model_path = path + "/models/5.h5"
        self.new_model = load_model(self.model_path)
        # new_model.summary()
        self.emo_dict = {0: "neutral", 1: "happy", 2: "sad", 3: "surprise", 4: "anger"}
        self.width = 150
        self.height = 150

        # ROS setup
        rospy.init_node("sentiment_analysis_node")
        self.sub_image = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback_image)
        self.pub_sentiment = rospy.Publisher("/sentiment_detected", String, queue_size = 10)
        self.sentiment_msg = String(data = "")
        self.rate = rospy.Rate(1)
        self.bridge = CvBridge()

    def callback_image(self, msg):

        # Validate correct encoding
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.sentiment_msg.data = f"Error {e} - 0% of confidence"

        faces = self.classifier.detectMultiScale(cv_image, 1.3, 5)

        for (x, y, ancho, alto) in faces:

            cv2.rectangle(cv_image, (x, y), (x + ancho, y + alto), (0, 255, 0), 3)
            face = cv_image[x : x + ancho, y : y + alto]

            try:

                face_resized = cv2.resize(face, (self.width, self.height))
                face_resized = face_resized[None, :, :, :]

                predict = self.new_model.predict(face_resized)
                predict = predict[0, :]

                if (max(predict) > 0.4):

                    sentiment = np.where(predict == max(predict))[0][0]

                    percentage = max(predict)
                    percentage = "{:.0%}".format(percentage)

                    msg = self.emo_dict[sentiment] + " - " + percentage + " of confidence"
                    self.sentiment_msg.data = msg

                    font = cv2.FONT_HERSHEY_PLAIN
                    cv2.putText(cv_image, msg, (x, y + alto + 25), font, 1, (255, 255, 255), 2, cv2.LINE_AA)

            except:

                self.sentiment_msg.data = "Error - 0% of conficence"

        cv2.imshow("Frame", cv_image)
        cv2.waitKey(3)

    def publisherFunctions(self):

        while not rospy.is_shutdown():
            self.pub_sentiment.publish(self.sentiment_msg)
            rospy.loginfo("Executing!")
            self.rate.sleep()

        self.pub_sentiment.publish(String(data = ""))
        cv2.DestroyAllWindows()


if __name__ == "__main__":

    try:
        # Node initialization
        path = rospy.get_param("sentiment_analysis_node/path")
        sentiment = SentimentAnalysis(path)
        sentiment.publisherFunctions()

    except rospy.ROSInterruptException:
        pass
