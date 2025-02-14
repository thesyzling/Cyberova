import cv2
import numpy as np
img=cv2.imread("çizgi.jpg")

gri=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
kenar=cv2.Canny(gri,100,250)
çizgi=cv2.HoughLinesP(kenar,1,np.pi/180,50)
print(çizgi)

for i in çizgi:
    x1,y1,x2,y2=i[0]
    cv2.line(img,(x1,y1),(x2,y2),(255,0,0),5)

    cv2.imshow("a",img)
    cv2.waitKey(0)