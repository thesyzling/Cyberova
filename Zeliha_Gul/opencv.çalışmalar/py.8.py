import cv2
import numpy as np

img=cv2.imread("indir.png")

gri=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
ret,thresh=cv2.threshold(gri,75,200,cv2.THRESH_BINARY)
contur,h=cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

h=list()

for i in range(len(contur)):
    h.append(cv2.convexHull(contur[i],False))

z=np.zeros((thresh.shape[0],thresh.shape[1],3),np.uint8)

for i in range(len(contur)):
    cv2.drawContours(z,contur,i,(255,0,0),3,8)
    cv2.drawContours(z,h,i,(0,255,0),1,8)


cv2.imshow("resim",z)
cv2.waitKey(0)
cv2.destroyAllWindow()