import cv2
import mediapipe as mp


mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils


def recognize_hand_sign(landmarks):
    
    
    tips = [4, 8, 12, 16, 20]  
    folded_fingers = []

    
    if landmarks[tips[0]].x < landmarks[2].x:  
        folded_fingers.append(False)  
    else:
        folded_fingers.append(True)  

    
    for tip in tips[1:]:
        if landmarks[tip].y > landmarks[tip - 2].y:
            folded_fingers.append(True)  
        else:
            folded_fingers.append(False)  
    
    count_open_fingers = folded_fingers.count(False)

    return str(count_open_fingers)

cap = cv2.VideoCapture(0)

with mp_hands.Hands(
    model_complexity=1,
    max_num_hands=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
) as hands:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        result = hands.process(rgb_frame)

        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
              
                mp_drawing.draw_landmarks(
                    frame, hand_landmarks, mp_hands.HAND_CONNECTIONS
                )

                sign = recognize_hand_sign(hand_landmarks.landmark)

                cv2.putText(
                    frame,
                    f"Sign: {sign}",
                    (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 255, 0),
                    2,
                )

        cv2.imshow("Hand Sign Recognition", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()