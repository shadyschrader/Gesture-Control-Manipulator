import cv2
import mediapipe as mp
import serial
import time

# Serial Port (Change according to your OS)
SERIAL_PORT = "/dev/ttyUSB0"  # Linux/macOS
# SERIAL_PORT = "COM3"  # Windows (Uncomment for Windows)

# Open Serial Connection
try:
    ser = serial.Serial(SERIAL_PORT, 115200, timeout=1)
    time.sleep(2)  # Wait for ESP32 to initialize
    print("Connected to ESP32 on", SERIAL_PORT)

except Exception as e:
    print("Error opening serial port:", e)
    exit()


# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)

# index and thumb
def get_finger_angle_1(lm):
    """Calculate finger angle based on hand position"""
    x1, y1 = lm[4][0], lm[4][1]   # Thumb Tip
    x2, y2 = lm[8][0], lm[8][1]   # Index Finger Tip
    angle_1 = int(((x2 - x1) / 250.0) * 180)  # Normalize to 0-180
    angle_1 = max(0, min(angle_1, 180))  # Clamp value
    return angle_1

# middle and thumb
def get_finger_angle_2(lm):
    """Calculate finger angle based on hand position"""
    x1, y1 = lm[4][0], lm[4][1]   # Thumb Tip
    x2, y2 = lm[12][0], lm[8][1]   # Index Finger Tip
    angle_2 = int(((x2 - x1) / 250.0) * 180)  # Normalize to 0-180
    angle_2 = max(0, min(angle_2, 180))  # Clamp value
    return angle_2

# ring and thumb
def get_finger_angle_3(lm):
    """Calculate finger angle based on hand position"""
    x1, y1 = lm[4][0], lm[4][1]   # Thumb Tip
    x2, y2 = lm[16][0], lm[8][1]   # Index Finger Tip
    angle_3 = int(((x2 - x1) / 250.0) * 180)  # Normalize to 0-180
    angle_3 = max(0, min(angle_3, 180))  # Clamp value
    return angle_3

# pinky and thumb
def get_finger_angle_4(lm):
    """Calculate finger angle based on hand position"""
    x1, y1 = lm[4][0], lm[4][1]   # Thumb Tip
    x2, y2 = lm[20][0], lm[8][1]   # Index Finger Tip
    angle_4 = int(((x2 - x1) / 250.0) * 180)  # Normalize to 0-180
    angle_4 = max(0, min(angle_4, 180))  # Clamp value
    return angle_4

# Start Camera Capture
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Flip Image & Convert to RGB
    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Detect Hands
    results = hands.process(rgb_frame)
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            lm_list = []
            for lm in hand_landmarks.landmark:
                h, w, c = frame.shape
                lm_list.append((int(lm.x * w), int(lm.y * h)))

            if len(lm_list) > 8:
                angle_1 = get_finger_angle_1(lm_list)
                angle_2 = get_finger_angle_2(lm_list)
                angle_3 = get_finger_angle_3(lm_list)
                angle_4 = get_finger_angle_4(lm_list)

                angle = [angle_1, angle_2, angle_3, angle_4]

                for i, ang in enumerate(angle):
                    """Send command to move a specific servo."""
                    command = f"S{i}:{ang}\n"
                    ser.write(command.encode())
                    print(f"Sending: {command.strip()}")
                    


                # command = f"S1:{angle}\n"  # Format the data as "S1:90"
                # ser.write(command.encode())  # Send to ESP32
                # print(f"Sending: {command.strip()}")  # Debugging output

                # ack = ser.readline().decode().strip()  # Wait for acknowledgment
                # if ack == "OK":
                #     print("ESP32 received the angle")

                # response = ser.readline().decode().strip()
                # if response:
                #     print(f"ESP32 says: {response}")
                            
                time.sleep(1)  # Wait before sending again

                

            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

    # Show Video Feed
    cv2.imshow("Gesture Control", frame)

    # Quit on 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
ser.close()
