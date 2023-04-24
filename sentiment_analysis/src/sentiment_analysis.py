import cv2
import numpy as np

import tensorflow as tf
from tensorflow.keras.models import load_model

cascade = "./sentiment_analysis/include/haarcascade_frontalface_alt.xml"
classifier = cv2.CascadeClassifier(cascade)

path = "./sentiment_analysis/models/5.h5"
new_model = load_model(path)
# new_model.summary()

def camera2Recv():

    emo_dict = {0: "neutral", 1: "happy", 2: "sad", 3: "surprise", 4: "anger"}
    cap = cv2.VideoCapture(0)

    width = 150
    height = 150

    while (cap.isOpened()):

        ret, frame = cap.read()
        
        if ret == True:

            color = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            faces = classifier.detectMultiScale(color, 1.3, 5)

            for (x, y, ancho, alto) in faces:

                cv2.rectangle(frame, (x,y), (x+ancho, y+alto), (0,255,0), 3)
                face = frame[x:x+ancho, y:y+alto]

                try:

                    face_resized = cv2.resize(face, (width, height))
                    face_resized = face_resized[None,:,:,:]

                    predict = new_model.predict(face_resized)
                    predict = predict[0,:]

                    if (max(predict) > 0.4):

                        sentimiento = np.where(predict == max(predict))[0][0]

                        percentage = max(predict)
                        percentage = "{:.0%}".format(percentage)

                        msg = emo_dict[sentimiento] + " - " + percentage + " of confidence"
                        print(msg)

                        font = cv2.FONT_HERSHEY_PLAIN
                        cv2.putText(frame, msg, (x,y+alto+25), font, 1, (255,255,255), 2, cv2.LINE_AA)

                except:

                    pass

            cv2.imshow("Frame", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):

                break
        else:

            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":

    print("Analyzing real time video")
    camera2Recv()
