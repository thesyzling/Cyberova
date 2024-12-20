import cv2

img = cv2.imread("a.jpg")

img1 = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
img2 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
img3 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

cv2.imshow("resim", img)
cv2.imshow("rbg", img1)
cv2.imshow("hsv", img2)
cv2.imshow("grey", img3)

cv2.waitKey(0)
cv2.destroyAllWindows()