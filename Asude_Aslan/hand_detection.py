import cv2
import mediapipe as mp


mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils


hands = mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5)


finger_tips = [4, 8, 12, 16, 20]
finger_mcp = [2, 5, 9, 13, 17]


cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("Kamera görüntüsü alınamıyor. Program durduruluyor.")
        break

    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    result = hands.process(rgb_frame)

    if result.multi_hand_landmarks:
        for idx, hand_landmarks in enumerate(result.multi_hand_landmarks):

            if result.multi_handedness:
                label = result.multi_handedness[idx].classification[0].label
            else:
                label = "Unknown"


            mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            landmarks = hand_landmarks.landmark
            fingers = []


            if label == "Right":
                if landmarks[finger_tips[0]].x < landmarks[finger_mcp[0]].x:
                    fingers.append(1)
                else:
                    fingers.append(0)
            elif label == "Left":  # Sol el
                if landmarks[finger_tips[0]].x > landmarks[finger_mcp[0]].x:
                    fingers.append(1)
                else:
                    fingers.append(0)
            else:
                fingers.append(0)


            for i in range(1, 5):
                if landmarks[finger_tips[i]].y < landmarks[finger_mcp[i]].y:
                    fingers.append(1)
                else:
                    fingers.append(0)


            count = sum(fingers)
            cv2.putText(frame, f'Count: {count}', (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow("Hand Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
