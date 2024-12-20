import cv2
import numpy as np
from matplotlib import pyplot as plt

path1 = '/home/fatih/Cyberova/Fatih_Bektaş/OpenCV/1st picture.jpg'
path2 = '/home/fatih/Cyberova/Fatih_Bektaş/OpenCV/2nd picture.jpg'

img1 = cv2.imread(path1, cv2.IMREAD_COLOR)
img2 = cv2.imread(path2, cv2.IMREAD_COLOR)

# Resimleri topla
added_img = cv2.add(img1, img2)

# Resimleri çıkar
subtracted_img = cv2.subtract(img1, img2)

plt.figure(figsize=(10, 6))  # Pencere boyutunu ayarla

# İlk resim
plt.subplot(1, 2, 1)
plt.imshow(cv2.cvtColor(added_img, cv2.COLOR_BGR2RGB))
plt.title('Toplanmış Resim')
plt.axis('off')

# İkinci resim
plt.subplot(1, 2, 2)
plt.imshow(cv2.cvtColor(subtracted_img, cv2.COLOR_BGR2RGB))
plt.title('Çıkarılmış Resim')
plt.axis('off')

plt.tight_layout()  # Subplotlar arası boşlukları düzenle
plt.show()




