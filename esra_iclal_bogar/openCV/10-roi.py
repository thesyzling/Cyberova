import cv2
import numpy as np

img = cv2.imread("a.jpg")
roi = img[200:350, 150:300]#yukari-asagi, sag-sol


cv2.imshow("resim",img)
cv2.imshow("roi",roi)

cv2.waitKey(0)
cv2.destroyAllWindows()