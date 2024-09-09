import numpy as np
import serial
import time
import cv2

# Adjust the COM port as needed
try:
    ard = serial.Serial('COM4', 9600)
    time.sleep(2)
    print("Connected to Arduino...")
except Exception as e:
    print(f"Error connecting to Arduino: {e}")
    sys.exit()

# Load the face detection model
face_cascade = cv2.CascadeClassifier('D:/A/Face Detection and Servo Control System/dataset/haarcascade_frontalface_default.xml')

# Capture video from webcam
vid = cv2.VideoCapture(0)

while True:
    ret, frame = vid.read()
    if not ret:
        print("Failed to grab frame")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, minSize=(80, 80), minNeighbors=3)
    
    if len(faces) > 0:
        (x, y, w, h) = faces[0]
        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
        Xpos = x + (w / 2)  # X coordinate of the center of the face
        Ypos = y + (h / 2)  # Y coordinate of the center of the face
        
        if Xpos > 280:
            ard.write('R'.encode())
        elif Xpos < 360:
            ard.write('L'.encode())
        else:
            ard.write('S'.encode())
        
        if Ypos > 280:
            ard.write('U'.encode())
        elif Ypos < 200:
            ard.write('D'.encode())
        else:
            ard.write('S'.encode())
        
        # For debugging purposes
        text = f"Ypos = {Ypos:.2f} Xpos = {Xpos:.2f}"
        cv2.putText(frame, text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 1)
    
    cv2.imshow('frame', frame)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        break

# Cleanup
cv2.destroyAllWindows()
ard.close()
vid.release()
