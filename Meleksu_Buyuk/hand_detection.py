import cv2 as cv
import numpy as np
import mediapipe as mp

def distance(a, b):
    return np.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)

def FingerOpen(tip, pip, dynamic_thres_hold):
    return int(distance(tip, pip) > dynamic_thres_hold)

def FingerOpenUsingWrist(tip, wrist, hand_size, threshold=0.3):
    
    return int(distance(tip, wrist) / hand_size > threshold)

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

cap = cv.VideoCapture(0)

landmark_drawing_spec = mp_drawing.DrawingSpec(color=(255, 0, 255), thickness=5, circle_radius=2)
connection_drawing_spec = mp_drawing.DrawingSpec(color=(255, 182, 193), thickness=2)

with mp_hands.Hands(min_detection_confidence=0.8, min_tracking_confidence=0.5) as hands:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Preprocess the frame
        image = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        image.flags.writeable = False
        result = hands.process(image)

        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                # Calculate hand size based on individual dimensions
                hand_width = distance(
                    hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_CMC],
                    hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP]
                )
                hand_height = distance(
                    hand_landmarks.landmark[mp_hands.HandLandmark.WRIST],
                    hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
                )
                hand_size = (hand_width + hand_height) / 2

                # Dynamic threshold based on individual hand size
                dynamic_thres_hold = hand_size * 0.2

                # Check if fingers are open (PIP Method)
                thumb_open_pip = FingerOpen(
                    hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP],
                    hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_IP], dynamic_thres_hold)
                index_open_pip = FingerOpen(
                    hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP],
                    hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP], dynamic_thres_hold)
                middle_open_pip = FingerOpen(
                    hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP],
                    hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP], dynamic_thres_hold)
                ring_open_pip = FingerOpen(
                    hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP],
                    hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_PIP], dynamic_thres_hold)
                pinky_open_pip = FingerOpen(
                    hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP],
                    hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_PIP], dynamic_thres_hold)

                # Check if fingers are open (WRIST Method)
                thumb_open_wrist = FingerOpenUsingWrist(
                    hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP],
                    hand_landmarks.landmark[mp_hands.HandLandmark.WRIST], hand_size)
                index_open_wrist = FingerOpenUsingWrist(
                    hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP],
                    hand_landmarks.landmark[mp_hands.HandLandmark.WRIST], hand_size)
                middle_open_wrist = FingerOpenUsingWrist(
                    hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP],
                    hand_landmarks.landmark[mp_hands.HandLandmark.WRIST], hand_size)
                ring_open_wrist = FingerOpenUsingWrist(
                    hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP],
                    hand_landmarks.landmark[mp_hands.HandLandmark.WRIST], hand_size)
                pinky_open_wrist = FingerOpenUsingWrist(
                    hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP],
                    hand_landmarks.landmark[mp_hands.HandLandmark.WRIST], hand_size)

                # Total number of open fingers (using WRIST method for demo)
                total = thumb_open_wrist + index_open_wrist + middle_open_wrist + ring_open_wrist + pinky_open_wrist

                # Display the number of open fingers on the frame
                cv.putText(frame, str(total), (245, 120), cv.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 255), thickness=5)

                # Draw landmarks and connections
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS,
                                          landmark_drawing_spec=landmark_drawing_spec,
                                          connection_drawing_spec=connection_drawing_spec)

        # Display the frame
        image.flags.writeable = True
        cv.imshow("Hand Tracking", frame)

        # Exit loop on 'q' key press
        if cv.waitKey(10) & 0xFF == ord('q'):
            break

cap.release()
cv.destroyAllWindows()
