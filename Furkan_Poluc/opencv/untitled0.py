# -*- coding: utf-8 -*-
"""
Created on Fri Dec 27 22:23:28 2024

@author: laptop1
"""
import cv2
import numpy as np
cap=cv2.VideoCapture(0)

def fonks(x):
    pass

cv2.namedWindow("bar")
cv2.resizeWindow("bar",600,600)

cv2.createTrackbar("alh-h","bar",0,180,fonks)
cv2.createTrackbar("alh-s","bar",0,255,fonks)
cv2.createTrackbar("alh-v","bar",0,255,fonks)

cv2.createTrackbar("ust-h","bar",0,180,fonks)
cv2.createTrackbar("ust-s","bar",0,255,fonks)
cv2.createTrackbar("ust-v","bar",0,255,fonks)

cv2.setTrackbarPos("ust-h","bar",150)
cv2.setTrackbarPos("ust-v","bar",200)
cv2.setTrackbarPos("ust-s","bar",200)

while True:
    ret,frame=cap.read()
    frame=cv2.flip(frame,1)
    frame_hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    
    alt_h=cv2.getTrackbarPos("alh-h", "bar")
    alt_s=cv2.getTrackbarPos("alh-s", "bar")
    alt_v=cv2.getTrackbarPos("alh-v", "bar")
    
    ust_h=cv2.getTrackbarPos("ust-h", "bar")
    ust_s=cv2.getTrackbarPos("ust-s", "bar")
    ust_v=cv2.getTrackbarPos("ust-v", "bar")
    
    alt_renk=np.array([alt_h,alt_s,alt_v])
    ust_renk=np.array([ust_h,ust_s,ust_v])
    
    mask=cv2.inRange(frame_hsv,alt_renk,ust_renk)
    
    cv2.imshow("o",frame)
    cv2.imshow("m",mask)
    
    if cv2.waitKey(20) & 0xFF==ord("q"):
        break
    
cap.release()
cv2.destroyAllWindows()
    
"""
daire = np.zeros((512,512,3),np.uint8) +255
cv2.circle(daire,(256,256),50,(255,0,0),-1)

kare = np.zeros((512,512,3),np.uint8) +255
cv2.rectangle(kare,(150,150),(350,350),(0,255,0),-1)

agirlik=cv2.addWeighted(daire, 0.5, kare, 0.5, 0)

cv2.imshow("agirlik", agirlik)



topla=cv2.add(daire,kare)
print(topla[256,256])

cv2.imshow("daire",daire)
cv2.imshow("kare",kare)
cv2.imshow("topla", topla)

cv2.waitKey(0)
cv2.destroyAllWindows()
"""







"""
import cv2
import numpy as np

img=cv2.imread("adana.jpg")


renk=img[200,300,]
print(renk)
#print(img.shape)
mavi=img[200,300,0]
print("mavi",mavi)
yesil=img[200,300,1]
print("yesil",yesil)
kirmizi=img[200,300,2]
print("kirmizi",kirmizi)

img[200,300,0]= 0
print(img[200,300])

mavi=img.item(150,150,0)
print(mavi)
img.itemset((150,150,0),200)
print(img[150,150])


roi=img[150:300,150:300]


cv2.imshow("roi",roi)
cv2.imshow("adana", img)

cv2.waitkey(0)
cv2.destroyAllWindows()
"""











"""
def fonks(x):
    pass

img=np.zeros((512,512,3),np.uint8)
cv2.namedWindow("image")

cv2.createTrackbar("r","image",0,255,fonks)
cv2.createTrackbar("g","image",0,255,fonks)
cv2.createTrackbar("b","image",0,255,fonks)

switch="0: OFF, 1: ON"
cv2.createTrackbar(switch,"image",0,1,fonks)

while True:
    cv2.imshow("image", img)
    if cv2.waitKey(1) & 0xFF==ord("q"):
        break
    
    r=cv2.getTrackbarPos("r","image")
    g=cv2.getTrackbarPos("g","image")
    b=cv2.getTrackbarPos("b","image")
    
    
    
    s=cv2.getTrackbarPos(switch, "image")
    if s==0:
        img[:]=[0,0,0]
    if s==1:
        img[:]=[b,g,r]



cv2.waitKey(0)
cv2.destroyAllWindows()
"""











"""
img = np.zeros((512,512,3),np.uint8)

#cv2.line(img, (0,0), (511,511), (255,0,0),5)
#cv2.line(img, (50,400), (400,50), (0,255,0),10)

#cv2.rectangle(img,(50,50),(300,300),(0,0,255),5)
#cv2.rectangle(img,(300,300),(511,511),(0,0,255),-1 )
#f1=cv2.FONT_ITALIC
f2=cv2.FONT_HERSHEY_PLAIN
f3=cv2.CAP_AVFOUNDATION
cv2.putText(img,"merhaba",(30,200),f1,2,(255,0,0),cv2.LINE_AA)

cv2.imshow("resim", img)
cv2.waitKey(0)
cv2.destroyAllWindows
"""










"""
import cv2

cam = cv2.VideoCapture(0)

fourrc = cv2.VideoWriter_fourcc(*'XVID')

out = cv2.VideoWriter("thinkinggrey.avi", fourrc, 30.0, (640,480))

while cam.isOpened():
    
    ret, frame = cam.read()
    
    if not ret:
        print("kameradan goruntu alinamadi")
        break
    
    out.write(frame)
    
    cv2.imshow("kamera", frame)
    
    if cv2.waitKey(1) == ord("q"):
        print("videodan ayrildiniz")
        break
    
cam.release()
out.release()
cv2.destroyAllWindows()
"""









"""
cam = cv2.VideoCapture("thinking.mp4")

while cam.isOpened():
    
    ret, frame = cam.read()
    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    
    if not ret:
        print("kameradan goruntu okunamıyor")
        break
    
    cv2.imshow("goruntu", frame)
    
    if cv2.waitKey(1) & 0xFF == ord("q"):
        print("video kapatildi.")
        break

cam.release()
cv2.destroyAllWindows()

"""







"""
import cv2

cam = cv2.VideoCapture(0)


print(cam.get(3))
print(cam.get(4))

cam.set(3, 320)
cam.set(4, 240)

if not cam.isOpened():
    print("kamera tanınmadı")
    exit()
    

while True:
    ret, frame = cam.read()
    
    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    
    
    if not ret:
        print("kameradan goruntu okunamiyor")
        break
    
    cv2.imshow("kamera",frame)
    
    if cv2.waitkey(1) & 0xFF == ord("q"):
        print("goruntu sonlandirildi...")
        break
    
cam.realease()
cv2.destroyAllWindows()
"""