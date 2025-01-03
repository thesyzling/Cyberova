import cv2
import numpy as np

img=cv2.imread("image.jpeg",0)

sat,sut=img.shape

m=cv2.getRotationMatrix2D((sut/5,sat/5),90,1)
d=cv2.warpAffine(img,m,(sut,sat))

cv2.imshow("resim",d)
cv2.waitKey(0)
cv2.destroyAllWindows()