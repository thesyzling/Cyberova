import cv2
import numpy as np

canvas=np.zeros((600,600,3),np.uint8) +255

f1=cv2.FONT_ITALIC
f2=cv2.FONT_HERSHEY_PLAIN
f3=cv2.CAP_AVFOUNDATION

cv2.putText(canvas,"merhaba",(30,200),f1,2,(255,0,0),cv2.LINE_AA)
#ilk yazdığımız parametre koordinasyon için , ikinci kısım font kısmı,üçüncü kısım boyut için son kısım yazının tipi
#openCV nin bize sunduğu fontlar var

cv2.imshow("pencere",canvas)
cv2.waitKey(0)
cv2.destroyedAllWindows()

