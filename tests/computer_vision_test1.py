import cv2
import mediapipe as mp
import sys
import os

#Steps to import the HandGestures class
# 1. Get the directory of the current test file (.../tests)
current_dir = os.path.dirname(os.path.abspath(__file__))
# 2. Get the parent directory (.../DRONE_CONTROL_TOOLS)
parent_dir = os.path.dirname(current_dir)
# 3. Add the parent directory to Python's system path
sys.path.append(parent_dir)

# --- Import the HandGestures class ---
from src.hand_gestures import HandGestures

hand_1 = None


#Notes
"""
This code is intented to test the computer visioin related stuff used for the drone on the
computer before actually running it on the drone. When running on the drone specific things 
like the camera index might be different
"""


#Captures video
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

#Initializes mediapipe hands and drawing utilities
mp_hand = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

#Create a hands object
hand = mp_hand.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5)

hand_gestures = HandGestures()

while True:

    #ret - True if frame is read correctly, False otherwise
    #frame - the actual image array
    ret, frame = cap.read()

    if not ret:
        print("Please connect camera")
        break

    #Converts the frame from BGR to RGB as mediapipe requires RGB images
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    #Processes the frame and detects hands
    results = hand.process(rgb_frame)

    #Draws the hand landmarks on the frame
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hand.HAND_CONNECTIONS)

    if results.multi_hand_landmarks:
        hand_1 = results.multi_hand_landmarks[0]
  
    gesture_id = hand_gestures.get_gesture(hand_1)
    print(gesture_id)

    

   

    #Displays the video feed in a window
    cv2.imshow('Video Feed', frame)

    #Breaks the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

#Releases the video capture object and closes all OpenCV windows
cap.release()
cv2.destroyAllWindows()