# Bu koddaki amacım görüntüyü okumak ve üzerine yazı eklemektir.

# Gerekli kütüphaneleri yükle
import cv2
import numpy as np
from matplotlib import pyplot as plt

# Resmi oku ve üzerine yazı ekle
img = cv2.imread('/home/fatih/Cyberova/Fatih_Bektaş/OpenCV/Apple.jpg', cv2.IMREAD_COLOR) # Resmi oku
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) # BGR'yi RGB'ye çevir
text = 'Apple' # Yazı
font = cv2.FONT_HERSHEY_DUPLEX # Font tipi
cv2.putText(img_rgb, text, (500, 600), font, 4, (0, 0, 0), 2, cv2.LINE_AA) # Yazı ekle

# Resmi göster
plt.imshow(img_rgb) # Resmi göster
plt.show() # Pencereyi göster

