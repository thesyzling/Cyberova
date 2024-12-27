import cv2
import numpy as np

"""
FOTOĞRAF YAZDIRMA AYARLAMA

img = cv2.imread("images.jpg")
img = cv2.resize(img,(720,500))
cv2.namedWindow("resim",cv2.WINDOW_NORMAL)
cv2.imshow("resim",img)
cv2.imwrite("copy.jpg",img)
cv2.waitKey(0)
"""



"""
WEBCAMDEN KAYIT ALMA

capture = cv2.VideoCapture(0,cv2.CAP_DSHOW)
while True:
    ret, frame = capture.read()
    cv2.flip(frame, 1)
    cv2.imshow("webcam",frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

    cv2.waitKey(10)

capture.release()
cv2.destroyAllWindows()
"""



"""
TUVAL OLUŞTURMA

canvas = np.zeros((512,512,3),dtype=np.uint8)
print(canvas)
cv2.imshow("pencere",canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()
"""





"""
canvas = np.zeros((500,500,3),np.uint8) + 255

f1 = cv2.FONT_ITALIC
cv2.putText(canvas,"hi",(100,200),f1,5,(0,255,0),cv2.LINE_AA)

cv2.imshow("pencere",canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()
    
"""















