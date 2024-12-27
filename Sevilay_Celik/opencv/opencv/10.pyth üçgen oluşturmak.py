import cv2
import numpy as np

canvas=np.zeros((500,500,3),np.uint8) +255

u1=(300,400)
u2=(400,450)
u3=(500,300)

cv2.line(canvas,u1,u2,(0,0,0),4)
cv2.line(canvas,u2,u3,(0,0,0),4)
cv2.line(canvas,u1,u3,(0,0,0),4)

cv2.imshow("pencere",canvas)
cv2.waitKey(0)
cv2.destroyedAllWindows()