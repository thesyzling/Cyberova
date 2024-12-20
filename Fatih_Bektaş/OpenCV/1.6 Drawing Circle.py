import cv2
import numpy as np
from matplotlib import pyplot as plt

canvas_size = 500
canvas = np.ones((canvas_size, canvas_size, 3), dtype=np.uint8) * 255  # Beyaz bir tuval

# Daire çiz
center = (canvas_size // 2, canvas_size // 2)  # Merkez noktasını belirle
radius = 100
color = (0, 0, 255)
thickness = 3
cv2.circle(canvas, center, radius, color, thickness) #Kullanımı (resim, merkez, yarıçap, renk, kalınlık) şeklinde

# Çizgileri ayarla
line_color = (0, 0, 0)
line_thickness = 2

# Dikey çizgi
start_point = (canvas_size // 2, 0)
end_point = (canvas_size // 2, canvas_size)
cv2.line(canvas, start_point, end_point, line_color, line_thickness)

# Yatay çizgi
start_point = (0, canvas_size // 2)
end_point = (canvas_size, canvas_size // 2)
cv2.line(canvas, start_point, end_point, line_color, line_thickness)

# Daireyi göster
plt.imshow(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
plt.show()
