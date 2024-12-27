import cv2
import numpy as np
img = cv2.imread("images.png")

gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
gray = np.float32(gray)
corner = cv2.goodFeaturesToTrack(gray,40,0.01,10)

corner = np.int0(gray)

for i in corner:
    x,y = i.ravel()

    cv2.circle(img,(x,y),3,(255,0,0),5)


cv2.imshow("img",img)


cv2.waitKey(0)
cv2.destroyAllWindows()