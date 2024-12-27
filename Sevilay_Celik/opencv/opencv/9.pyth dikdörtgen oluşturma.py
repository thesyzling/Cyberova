import cv2
import numpy as np

canvas=np.zeros((500,500,3),np.uint8) +255

cv2.rectangle(canvas,(30,30),(60,60),(0,255,0),4)
#içi dolu bir dikdörtgen oluşturmak istersek thickness değerine -1 veriyoruz

""" eğer daire oluşturmak istersek
cv2.circle(canvas,(100,100),100,(255,0,0),4)
 
"""

cv2.imshow("pencere",canvas)
cv2.waitKey(0)
cv2.destroyedAllWindows()