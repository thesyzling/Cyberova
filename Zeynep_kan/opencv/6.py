import cv2
import numpy as np
canvas =np.zeros((500,500,3),np.uint8) + 255

cv2.line(canvas,(100,100),(300,300),(0,0,255),thickness=10)
cv2.line(canvas,(100,100),(340,4000),(255,0,0),thickness=10)


cv2.imshow("pencere",canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()
