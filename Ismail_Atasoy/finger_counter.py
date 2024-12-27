import cv2
import mediapipe as mp

hand = mp.solutions.hands
hands = hand.Hands(min_tracking_confidence=0.5, min_detection_confidence=0.5)
drawing = mp.solutions.drawing_utils

finger_cordinates = [(8, 6), (12, 10), (16, 14), (20, 18)]
thumb_cordinate = (4, 3)

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    result = hands.process(rgb_frame)
    multi_landmarks = result.multi_hand_landmarks

    total_upcount = 0  # Toplam parmak sayısı

    if multi_landmarks:
        for landmarks in multi_landmarks:
            hand_points = []
            drawing.draw_landmarks(frame, landmarks, hand.HAND_CONNECTIONS)

            for idx, lm in enumerate(landmarks.landmark):
                h, w, c = frame.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                hand_points.append((cx, cy))

            if len(hand_points) >= 21:
                upcount = 0
                for cordinate in finger_cordinates:
                    if hand_points[cordinate[0]][1] < hand_points[cordinate[1]][1]:
                        upcount += 1

                if hand_points[thumb_cordinate[0]][0] > hand_points[thumb_cordinate[1]][0]:
                    upcount += 1

                total_upcount += upcount  # Her elin parmak sayısını toplama

    # Toplam parmak sayısını yazdır
    cv2.putText(frame, str(total_upcount), (100, 150), cv2.FONT_HERSHEY_SIMPLEX, 5, (255, 0, 0), 5)

    cv2.imshow("finger counter", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
