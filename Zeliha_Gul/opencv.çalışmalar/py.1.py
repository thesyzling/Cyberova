import cv2
img=cv2.imread("images(3).jpg")
roi=img[230:280,190:280]


cv2.imshow("resim",img)
cv2.imshow("roi",roi)

cv2.waitKey(0)
cv2.destroyAllWindows()