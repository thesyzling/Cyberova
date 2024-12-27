import cv2
import cvzone # pip install cvzone
import numpy as np

cap = cv2.VideoCapture(0)
detector = cvzone.HandDetector(detectionCon=0.8, maxHands=1)

while True:
    success, img = cap.read()
    img = cv2.flip(img, 1)
    img = detector.findHands(img)
    lmList, _ = detector.findPosition(img)
    if lmList:
        fingers = detector.fingersUp()
        cv2.putText(img, f'Fingers: {fingers}', (10, 70), cv2.FONT_HERSHEY_PLAIN, 3, (0, 255, 0), 3)

    cv2.imshow("Image", img)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
