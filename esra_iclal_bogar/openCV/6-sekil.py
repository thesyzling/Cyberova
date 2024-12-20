import cv2
import numpy as np

canvas = np.zeros((500,500,3), np.uint8) +255

cv2.line(canvas,(100,100), (300,300), (0,0,255), thickness=5)
#nerede , (baslangic konumu), (bitis), (renk), kalinlik
cv2.line(canvas,(300,350), (400,500), (255,0,0), 8)

cv2.rectangle(canvas,(160,160), (300,300),(0,255,0), -1)

cv2.circle(canvas,(100,100), 60, (255,0,0),4)

u1 = (300,500)
u2 = (400,400)
u3 = (500,300)

cv2.line(canvas,u1,u2,(0,0,0),4)
cv2.line(canvas,u2,u3,(0,0,0),4)
cv2.line(canvas,u1,u3,(0,0,0),4)


cv2.imshow("pencere",canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()