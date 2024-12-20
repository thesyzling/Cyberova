import cv2
img=cv2.imread("audi.jpg"cv2.IMREAD_GRAYSCALE)
# IMREAD_GRAYSCALE yerine 0 kullanabiliriz
cv2.namedWindow("image",cv2.WINDOW_NORMAL)
cv2.imshow("image",img)
cv2.imwrite("copy.jpg",img)
cv2.waitKey(0)

cv2.destroyAllWindows()