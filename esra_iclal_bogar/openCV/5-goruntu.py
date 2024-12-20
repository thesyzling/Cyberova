import cv2
import numpy as np
"""
renkli tonlama
img = np.zeros((20,20,3),np.uint8) +255

img[0,0] = (0,0,0)
img[0,1] = (50,0,0)
img[0,2] = (100,0,0)
img[0,3] = (150,0,0)
img[0,4] = (200,0,0)
img[0,5] = (250,0,0)
"""

#gri tonlama
img = np.zeros((20,20,3),np.uint8) +255

img[0,0] = 255
img[0,1] = 200
img[0,2] = 150
img[0,3] = 100
img[0,4] = 50
img[0,5] = 0



img = cv2.resize(img,(300,300), interpolation=cv2.INTER_AREA)


print(img)
cv2.imshow("pencere", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
