#Resmi bulanıklaştırmak için birden fazla yöntem vardır. 3 tanesini aynı pencerede göstermeye çalışacağım.

import cv2
import numpy as np
from matplotlib import pyplot as plt

path = '/home/fatih/Cyberova/Fatih_Bektaş/OpenCV/Apple.jpg'
img = cv2.imread(path)

# 1. Yöntem: Gaussian Blurring
gaussian_blur = cv2.GaussianBlur(img, (15, 15), 0)

# 2. Yöntem: Median Blurring
median_blur = cv2.medianBlur(img, 15)

# 3. Yöntem: Bilateral Filtering
bilateral_filter = cv2.bilateralFilter(img, 15, 75, 75)

plt.figure(figsize=(12, 6))  # Pencere boyutunu ayarla

# Gaussian Blurring
plt.subplot(1, 3, 1)
plt.imshow(cv2.cvtColor(gaussian_blur, cv2.COLOR_BGR2RGB))
plt.title('Gaussian Bulanıklığı')
plt.axis('off')

# Median Blurring
plt.subplot(1, 3, 2)
plt.imshow(cv2.cvtColor(median_blur, cv2.COLOR_BGR2RGB))
plt.title('Median Bulanıklığı')
plt.axis('off')

# Bilateral Filtering
plt.subplot(1, 3, 3)
plt.imshow(cv2.cvtColor(bilateral_filter, cv2.COLOR_BGR2RGB))
plt.title('Bilateral Filtreleme')
plt.axis('off')

plt.tight_layout()  # Subplotlar arası boşlukları düzenle
plt.show()