import numpy as np
from PIL import Image
import cv2
import time
from keras.models import load_model
import pandas as pd
from csv import writer
import socket
import struct
from moviepy.editor import VideoFileClip
import threading

#Take potentiometer value from Raspbhome/Brushlerry Pi VIA direct ethernet connection UDP

bufferSize = 1024
msgFromServer = "SUP"
ServerPort = 5000
ServerIP= '192.168.1.25' #Jetson Host IP
ClientAddress = '192.168.1.14'
dataset_max_encoderValue = 15343 
dataset_min_encoderValue = 10709

#take video PATH



print('yes')	

#UDP stuff

#sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
RPIsocket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
# RPIsocket.bind((ServerIP,ServerPort))  
print('server is up and Listening . . .')
print('server is up and Listening . . .')

#load model 
model = load_model('/home/brushlessdc/Desktop/TESTING/model-Final.h5')
#####
stop_event = threading.Event()

def img_preprocess(img) :
    img = img[200:, :,:]          #cropping the image to the relevant information
    img = cv2.cvtColor(img, cv2.COLOR_RGB2YUV)
    img = cv2.GaussianBlur(img,  (3,3),0)
    img = cv2.resize(img, (66, 200))
    img = img.reshape(1, 66, 200, 3)
    img = img/255
    return img


def predict_and_UDP():
    video_path = "/home/brushlessdc/Desktop/TESTING/testing_Vids/road 2.mp4"
    video_capture = cv2.VideoCapture(video_path)
    video_capture.set(cv2.CAP_PROP_FPS, 1 )
    assert video_capture.isOpened(), "Failed to open video capture"
    while video_capture.isOpened():

        success, frame = video_capture.read()
        # cv2.imshow("Frame", frame      
        if not success:
            print('nope')
            break
        # print('success')

        image = np.asarray(frame)
        image = img_preprocess(image)
        image = np.array(image)
        # print(image.shape)
        steering_angle = float(model.predict(image))
        enc_steer_val= int(((steering_angle+1)/2)*(dataset_max_encoderValue-dataset_min_encoderValue)+dataset_min_encoderValue)
        # print(type(enc_steer_val))
        # print(enc_steer_val)
        encoded_value = struct.pack('i',enc_steer_val)
        bytesToSend = encoded_value
        RPIsocket.sendto(bytesToSend,(ClientAddress, ServerPort))
        time.sleep(0.5)

def show_vid():
    video_path = "/home/brushlessdc/Desktop/TESTING/testing_Vids/Road 2 (for 2nd thread).mp4"
    clip = VideoFileClip(video_path)
    while True:
        clip.preview()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        

thread1 =threading.Thread(target=predict_and_UDP)
thread1.start()

thread2 = threading.Thread(target=show_vid)

thread2.start()





