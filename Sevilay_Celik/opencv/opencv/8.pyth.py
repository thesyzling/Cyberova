import cv2
import numpy as np
canvas=np.zeros((500,500,3),np.uint8) +255

cv2.line(canvas,(100,100),(300,300),(0,0,255),(0,0,255),thickness=5)

cv2.line(canvas,(200,200),(300,300),(255,0,0),8)

# ilk yazdığımız başlangıç konumu, ikincisi bitiş ,üçüncüsü renk ,dördürcüsü ise kalınlığı
cv2.imshow("pencere",canvas)
cv2.waitKey(0)
cv2.destroyedAllWindows()