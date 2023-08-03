import pandas as pd
import cv2
import csv
from csv import writer
import os
import socket
import struct



print("SSH is Connected")
# Create a directory to store the frames if it doesn't exist
if not os.path.exists('frames'):
    os.makedirs('frames')

# Open a video capture object for the camera 
cap = cv2.VideoCapture(0)
print("/n/n/n/n CAMERA IS OPEN /n/n/n")
# Set the capture rate to X frames per second
cap.set(cv2.CAP_PROP_FPS, 20) 

#Take potentiometer value from Raspbhome/Brushlerry Pi VIA direct ethernet connection UDP
filename = '/home/brushlessdc/Desktop/TESTING/framezz.csv'
serverAddress = ('192.168.1.14',5000) #Hosts IP
clientAddress = ('192.168.1.25', 5000) #*** Change on client' IP
bufferSize = 1024
UDPClient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  
UDPClient.bind(clientAddress)

frame_number = 0

try:
    df = pd.read_csv(filename)
    print("Number of rows:", len(df))
    frame_number = len(df)
except FileNotFoundError:
    print("File not found")
except pd.errors.ParserError as e:
    print("Error parsing the CSV file:", str(e))
    frame_number = 0  # Set X to 0 in case of an error
except Exception as e:
    print("An error occurred:", str(e))
    frame_number = 0  # Set X to 0 in case of any other error
    print(frame_number)

with open(filename, 'a',newline= '') as f:
    # Write to CSV file
    
    cwriter = csv.writer(f)

    while True:
    # Read from the camera
        ret, frame = cap.read()

    # Display the window
        # cv2.imshow('Frame', frame)

    # Define the file address for the current frame
        file_address = f'frames/frame_{frame_number}.jpg'  #***Be more specific, Add on later
        data,address= UDPClient.recvfrom(bufferSize)
        bytes2unpack1 = data[:4]
        bytes2unpack2 = data[4:8]
        P_value,V_value = struct.unpack('ff',bytes2unpack1+bytes2unpack2)
        print(f"Data from Server, P = {P_value}, V = {V_value}") 
    # Save the frame as an image
        cv2.imwrite(file_address, frame)

        cwriter.writerow([file_address,P_value,V_value])
        frame_number += 1
   
    # Break the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
f.close()
# Release the video capture object and close the CSV file
cap.release()


# Close all OpenCV windows
cv2.destroyAllWindows()