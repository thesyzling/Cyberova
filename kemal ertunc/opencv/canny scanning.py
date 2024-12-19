import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while True:
    ret , frame = cap.read()
    frame = cv2.flip(frame,1)
    cross = cv2.Canny(frame,10,10)

    cv2.imshow("original",frame)
    cv2.imshow("cross",cross)

    if cv2.waitKey(5) & 0xFF==ord("q"):
        break

cap.release()
cv2.destroyAllWindows()