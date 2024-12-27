import cv2
import numpy as np
canvas =np.zeros((600,600,3),np.uint8) + 255

f1=cv2.FONT_ITALIC
f2=cv2.FONT_ITALIC
f1=cv2.FONT_ITALIC

cv2.putText(canvas,"merhaba",(30,200),f1,6,(255,0,0),cv2.LINE_AA)

cv2.imshow("pencere",canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()
