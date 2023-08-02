#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

import tensorflow as tf
from tensorflow.keras.models import load_model

import cv2
import numpy as np

class sentimentAnalysis:

    def __init__(self):

        # Model setup
        self.cascade_path = "./sentiment_analysis/include/haarcascade_frontalface_alt.xml"
        self.classifier = cv2.CascadeClassifier(self.cascade_path)
        self.model_path = "./sentiment_analysis/models/5.h5"
        self.new_model = load_model(self.model_path)
        # new_model.summary()
        self.emo_dict = {0: "neutral", 1: "happy", 2: "sad", 3: "surprise", 4: "anger"}
        self.width = 150
        self.height = 150

        # ROS setup
        rospy.init_node("sentiment_analysis_node", anonymous = False)
        self.sub_image = rospy.Subscriber("/image_topic", Image, self.callback_image)
        self.pub_sentiment = rospy.Publisher("/sentiment_detected", String)
        self.sentiment_msg = String()
        self.bridge = CvBridge()

    def callback_image(self, msg):

        # Validate correct encoding
        try:
            cv_image = self.bridge.imgmsg_to_cv(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        faces = self.classifier.detectMultiScale(cv_image, 1.3, 5)

        for (x, y, ancho, alto) in faces:

            cv2.rectangle(cv_image, (x,y), (x+ancho, y+alto), (0,255,0), 3)
            face = cv_image[x:x+ancho, y:y+alto]

            try:

                face_resized = cv2.resize(face, (self.width, self.height))
                face_resized = face_resized[None,:,:,:]

                predict = self.new_model.predict(face_resized)
                predict = predict[0,:]

                if (max(predict) > 0.4):

                    sentiment = np.where(predict == max(predict))[0][0]

                    percentage = max(predict)
                    percentage = "{:.0%}".format(percentage)

                    msg = self.emo_dict[sentiment] + " - " + percentage + " of confidence"
                    self.sentiment_msg.data= sentiment
                    #print(msg)

                    font = cv2.FONT_HERSHEY_PLAIN
                    cv2.putText(cv_image, msg, (x,y+alto+25), font, 1, (255,255,255), 2, cv2.LINE_AA)

            except:

                pass

        cv2.imshow("Frame", cv_image)
        cv2.waitKey(3)

    def publisherFunctions(self):

        while not rospy.is_shutdown():
            self.pub_sentiment.publish(self.sentiment_msg)
            rospy.loginfo("Executing!")
            self.rate.sleep()

        cv2.DestroyAllWindows()

if __name__ == "__main__":

    try:
        print("Analyzing real time video")
        # Node initialization
        sentiment = sentimentAnalysis()
        rate = rospy.Rate(10)
        sentiment.publisherFunctions()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
