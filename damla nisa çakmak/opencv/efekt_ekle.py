import cv2
import numpy as np

cap= cv2.VideoCapture(0)
#trackbar için fomksiyon zorunlu
def fonk(*args):
    pass
cv2.namedWindow("bar")
cv2.resizeWindow("bar",600,600)
#alt trackbar olusturma
cv2.createTrackbar("alt-h","bar",0,180,fonk)
cv2.createTrackbar("alt-s","bar",0,200,fonk)
cv2.createTrackbar("alt-v","bar",0,200,fonk)
#üst trackbar olusturma
cv2.createTrackbar("ust-h","bar",0,100,fonk)
cv2.createTrackbar("ust-s","bar",0,255,fonk)
cv2.createTrackbar("ust-v","bar",0,255,fonk)

cv2.setTrackbarPos("ust-h","bar",150)
cv2.setTrackbarPos("ust-s","bar",180)
cv2.setTrackbarPos("ust-v","bar",200)



while True:
    ret, frame= cap.read()
    frame= cv2.flip(frame,1)
    frame_hsv= cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    alt_h=cv2.getTrackbarPos("alt-h","bar")
    alt_s = cv2.getTrackbarPos("alt-s", "bar")
    alt_v = cv2.getTrackbarPos("alt-v", "bar")

    ust_h = cv2.getTrackbarPos("ust-h", "bar")
    ust_s = cv2.getTrackbarPos("ust-s", "bar")
    ust_v = cv2.getTrackbarPos("ust-v", "bar")

    alt_renk= np.array([alt_h,alt_s,alt_v])
    ust_renk= np.array([ust_h,ust_s,ust_v])

    mask=cv2.inRange(frame_hsv,alt_renk,ust_renk)
    cv2.imshow("masked",mask)
    cv2.imshow("normal",frame)
    if cv2.waitKey(20) & 0xFF== ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
