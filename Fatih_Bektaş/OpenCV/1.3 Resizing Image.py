import cv2
import numpy as np
from matplotlib import pyplot as plt

# Resmi oku
img = cv2.imread('/home/fatih/Cyberova/Fatih_Bektaş/OpenCV/Apple.jpg', cv2.IMREAD_COLOR)

# Resmi yeniden boyutlandır
scale_percent = 50 # Yüzde kaç oranında küçültmek istediğimizi belirliyoruz.
width = int(img.shape[1] * scale_percent / 100) # Genişlik
height = int(img.shape[0] * scale_percent / 100) # Yükseklik
resized_img = cv2.resize(img, (width, height)) # Resmi yeniden boyutlandır

plt.figure(figsize=(12, 8))  # Pencere boyutunu ayarla

# Orijinal resim
plt.subplot(1, 2, 1)
plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
plt.title(f'Orijinal Resim\n({img.shape[1]}x{img.shape[0]})')  # Boyutları başlıkta göster
plt.axis('off')

# Yeniden boyutlandırılmış resim
plt.subplot(1, 2, 2)
plt.imshow(cv2.cvtColor(resized_img, cv2.COLOR_BGR2RGB))
plt.title(f'Yeniden Boyutlandırılmış Resim\n({width}x{height})')  # Boyutları başlıkta göster
plt.axis('off')

plt.tight_layout()  # Subplotlar arası boşlukları düzenle
plt.show()


