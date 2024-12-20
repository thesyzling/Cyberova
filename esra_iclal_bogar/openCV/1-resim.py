import cv2

img = cv2.imread("a.jpg")
img = cv2.resize(img,(400,300))
#img = cv2.imread("a.jpg",cv2.IMREAD_GRAYSCALE) #Resmi siyah beyaz tonlarda yapma 
#cv2.IMREAD_GRAYSCALE yerine 0 degeri de yazilabilir
cv2.namedWindow("image",cv2.WINDOW_NORMAL) #boyutunu ayarlama
cv2.imshow("image", img)
cv2.imwrite("copy.jpg", img) #resmi kopyalama
cv2.waitKey(5000) #resmin açık duracagi sure (milisaniye)
#0 yazarsak klavyeden girdi alana kadar acik kalir

cv2.destroyAllWindows() #uygulama bittiginde otomatik kapatma icin

