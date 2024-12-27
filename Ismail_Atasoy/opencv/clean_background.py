import cv2
import numpy as np

cap = cv2.VideoCapture(0)
sub = cv2.createBackgroundSubtractorMOG2(100,50,True)

while True:
    a,frame = cap.read()
    m = sub.apply(frame)

    if cv2.waitKey(20) & 0xFF == ord("q"):
        break

    cv2.imshow("o",frame)
    cv2.imshow("n",m)

cap.release()
cv2.destroyAllWindows()