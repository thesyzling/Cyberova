import cv2
import numpy as np

img=np.zeros((20,20,3),np.uint8) +255

img=cv2.resize(img,(300,300),interpolation=cv2.INTER_AREA)

img[0,0]=(0,0,0)
img[0,1]=(0,0,0)
img[0,2]=(0,0,0)
img[0,3]=(0,0,0)


cv2.imshow("pencere",img)
cv2.waitKey(0)
cv2.destroyAllWindows()