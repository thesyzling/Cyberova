import cv2
img = cv2.imread("images.png")
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

a1,thresh = cv2.threshold(gray,100,200,cv2.THRESH_BINARY)
m = cv2.moments(thresh)

x = int(m["m10"]/m["m00"])
y = int(m["m01"]/m["m00"])

cv2.circle(img,(x,y),5,(0,255,0),5)

cv2.imshow("s",img)
cv2.waitKey(0)
cv2.destroyAllWindows()