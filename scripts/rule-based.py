import cv2
import mediapipe as mp
import serial
import time
import numpy as np

# Initialize Serial Communication
SERIAL_PORT = "/dev/ttyUSB0"  # Adjust for your system
ser = serial.Serial(SERIAL_PORT, 115200, timeout=1)
time.sleep(2)

# Initialize MediaPipe
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)

# Gesture Detection Function
def detect_gesture(landmarks):
    """Recognize hand gestures using rule-based conditions."""
    thumb_tip = landmarks[4]  # Thumb Tip
    index_tip = landmarks[8]  # Index Finger Tip
    middle_tip = landmarks[12]  # Middle Finger Tip
    ring_tip = landmarks[16]  # Ring Finger Tip
    pinky_tip = landmarks[20]  # Pinky Finger Tip
    palm_base = landmarks[0]  # Wrist

    # GRAB: All fingers curled (tips below MCP joints)
    if all(finger.y > landmarks[i - 3].y for i, finger in enumerate([index_tip, middle_tip, ring_tip, pinky_tip], start=8)):
        return "GRAB"

    # RELEASE: All fingers extended (tips above MCP joints)
    elif all(finger.y < landmarks[i - 3].y for i, finger in enumerate([index_tip, middle_tip, ring_tip, pinky_tip], start=8)):
        return "RELEASE"

    # PINCH: Thumb and index tip are close
    palm_width = abs(landmarks[17].x - landmarks[5].x)  # Distance between pinky MCP and index MCP
    pinch_threshold = palm_width * 0.2  # Dynamic threshold
    if abs(thumb_tip.x - index_tip.x) < pinch_threshold:
        return "PINCH"

    return "UNKNOWN"


# Capture Video
cap = cv2.VideoCapture(0)
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Convert to RGB for MediaPipe
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    result = hands.process(frame_rgb)

    if result.multi_hand_landmarks:
        for hand_landmarks in result.multi_hand_landmarks:
            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            # Convert landmarks to a list
            landmarks = hand_landmarks.landmark
            gesture = detect_gesture(landmarks)
            cv2.putText(frame, gesture, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Send detected gesture to ESP32
            if gesture != "UNKNOWN":
                ser.write((gesture + "\n").encode())

    cv2.imshow("Gesture Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
ser.close()
