import cv2
import numpy as np

img = cv2.imread("images.png")
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

a,thresh = cv2.threshold(gray,200,300,cv2.THRESH_BINARY)
contur,b = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

for i in contur:
    e = 0.01*cv2.arcLength(i,True)
    approx = cv2.approxPolyDP(i,e,True)
    cv2.drawContours(img,[approx],0,5)

    x = approx.ravel()[0]
    y = approx.ravel()[1]

    print(approx)
    print(len(approx))

    if len(approx) ==3:
        cv2.putText(img,"ucgen",(x,y),cv2.FONT_ITALIC,1,1)

cv2.imshow("a",img)
cv2.waitKey(0)
cv2.destroyAllWindows()