import cv2
import numpy as np

canvas = np.zeros((512,512,3), dtype= np.uint8) +255 #+255 renk vermek icin kullanilir
#8 bitlik tam sayilari tutan numpy dizisi

print(canvas)

cv2.imshow("pencere", canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()