import cv2
img=cv2.imread("humonbody.jpeg")

vucud=cv2.CascadeClassifier("humonbody.xml")
gri=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

bodies=vucud.detectMultiScale(gri,1.1,3)

for x,y,w,h in bodies:
    cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)

cv2.imshow("1",img)
cv2.waitKey(0)
cv2.destroyAllWindows()
