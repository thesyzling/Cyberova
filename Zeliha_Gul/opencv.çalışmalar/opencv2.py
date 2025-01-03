import cv2
cap=cv2.VideoCapture(0,cv2.CAP_DSHOW)

while True:
    ret,frame=cap.read()
    frame=cv2.flip(frame,1)
    cv2.imshow("webcab",frame)
    if cv2.waitKey(1)&0xFF==("q"):
        break


        cv2.waitKey(30)

cap.release()
cv2.destroyAllWindows()
