import cv2
import numpy as np

img=np.zeros((20,20),np.uint8)+255
img[0,0]=255
img[0,1]=200
img[0,2]=150
img[0,3]=100
img[0,4]=50
img[0,5]=0

img=cv2.resize(img,(500,500),interpolation=cv2.INTER_AREA)

cv2.imshow=("pencere",img)
cv2.waitKey(0)
cv2.destroyAllWindows()