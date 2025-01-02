import cv2
import numpy as np
cap= cv2.VideoCapture("y2mate.com - Double solid yellow lines Doing the speed limit Time to overtake_v720P.mp4")

while True:
    ret, frame= cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    ust= np.array([18,94,140],np.uint8)
    alt=np.array([48,255,255],np.uint8)
    mask= cv2.inRange(hsv,ust,alt)
    edges= cv2.Canny(mask,75,250)
    line= cv2.HoughLinesP( edges ,1,np.pi/180,50,5)
    for i in line:
        x1,y1,x2,y2=i[0]
        cv2.line(frame,(x1,y1),(x2,y2),(0,255,0),7)
    cv2.imshow("m",frame)
    if ret==0:
        break


    if cv2.waitKey(25) & 0xFF==ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
