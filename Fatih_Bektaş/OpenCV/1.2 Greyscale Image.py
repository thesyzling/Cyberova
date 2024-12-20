#Bu kodda bir resmi okuyup, griye çevirip gösteriyoruz.
import cv2
from matplotlib import pyplot as plt

img = cv2.imread('/home/fatih/Cyberova/Fatih_Bektaş/OpenCV/Apple.jpg', cv2.IMREAD_COLOR) # Resmi oku
img_greyscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # BGR'yi Gri'ye çevir
plt.imshow(img_greyscale, cmap='gray') # Resmi göster
plt.title('Siyah Beyaz Resim') # Başlık ekle
plt.axis('off') # Bu sefer eksenleri kapatıyorum çünkü bir yazı eklemiyorum.
plt.show() # Pencereyi göster