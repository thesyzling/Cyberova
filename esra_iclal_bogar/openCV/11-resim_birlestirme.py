import cv2
import numpy as np

daire= np.zeros((512,512,3), np.uint8) +255
cv2.circle(daire, (256,256), 50, (255,0,0), -1)

kare = np.zeros((512,512,3), np.uint8) +255
cv2.rectangle(kare, (150,150),(350,350), (0,255,0), -1)
"""
topla = cv2.add(daire,kare)
print(topla[256,256])
"""
agirlik = cv2.addWeighted(daire,0.3,kare,0.7,0)# 0.3 ve 0.7 dairenin ve karenin agirligi

cv2.imshow('Daire', daire)
cv2.imshow('Kare', kare)
#cv2.imshow('toplam', topla)
cv2.imshow('agirlik', agirlik)


cv2.waitKey(0)
cv2.destroyAllWindows()