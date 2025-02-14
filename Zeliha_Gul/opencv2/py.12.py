import cv2

img=cv2.imread("face.jpg")


faces_cascade=cv2.CascadeClassifier(cv2.data.haarcascades+'haarcascade_frontalface_default.xml')
gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

faces=faces_cascade.detectMultiScale(gray,1.3,4)

for (x,y,w,h) in faces:
    cv2.rectangle(img,(x,y),(x+w,y+h),(250,0,0),2)

    cv2.imshow("Detected Faces",img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()