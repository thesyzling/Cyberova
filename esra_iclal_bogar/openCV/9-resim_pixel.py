import cv2
import numpy as np

img = cv2.imread("a.jpg")

renk = img[200,300] #200-300 konumundaki rengi yazar. 0 eklersek 0. değeri yazar

print(renk) #rengini yazar
print(img.shape) #boyutunu yazar
"""
mavi = img[200,300,0]
print(mavi) #maviye ait
yesil = img[200,300,1]
print(yesil) #yesile ait
kirmizi = img[200,300,2]
print(kirmizi) #kirmiziye ait
"""

mavi1 =img.item(150,150,0)
print(mavi1) #maviye ait
img.itemset((150,150,0), 200)
print(img[150,150]) #maviye ait




cv2.imshow("renkk", img)

cv2.waitKey(0)
cv2.destroyAllWindows()
