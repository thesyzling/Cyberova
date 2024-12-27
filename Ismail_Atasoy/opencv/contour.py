import numpy as np
import cv2

img = cv2.imread("images.png")
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

ret,thresh = cv2.threshold(gray,100,200,cv2.THRESH_BINARY)
cont,a = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

cv2.drawContours(img,cont,-1,(0,255,0),2)

cv2.imshow("0",img)
cv2.waitKey(0)
cv2.destroyAllWindows()

