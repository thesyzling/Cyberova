import cv2
import numpy as np

from opencv5 import canvas

canvas=np.zeros((600,600,3),np.uint8)+255

f1=cv2.FONT_ITALIC
f2=cv2.FONT_HERSHEY_PLAIN
f3=cv2.FONT_AVFOUNDATİON

cv2.putText(canvas,"merhaba",(130,200),f3,3,(255,0,0),cv2.LINE_AA)

cv2.imshow("pencere",canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()