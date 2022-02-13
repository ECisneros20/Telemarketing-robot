import cv2
import numpy as np

import tensorflow as tf
from tensorflow.keras.models import load_model

path = './sentiment_analysis/miniXception.h5'

new_model = load_model(path)

new_model.summary()

def camera2Recv():

    emo_dict = {0: 'happy', 1: 'sad', 2: 'surprise', 3: 'fear', 4: 'disgust-contempt', 5: 'anger'}

    cap = cv2.VideoCapture(0)

    width = 150
    height = 150
  
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

            font = cv2.FONT_HERSHEY_PLAIN
            cv2.putText(frame, emo_dict[sentimiento], (220,440), font, 3, (255,255,255), 2, cv2.LINE_AA)
            cv2.imshow('Frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':

    print("Extrae imagen de la cámara PTZ")
    camera2Recv()
