import cv2


cap = cv2.VideoCapture("v.mp4")

while True:
    ret, frame = cap.read()
    if ret == 0: #Video bitince kapatir
        break
  

    frame = cv2.flip(frame,1)#ters cevirme
    cv2.imshow("webcam", frame)
    
 
    cv2.waitKey(30)#saniyede yakaladigi frame

cap.release()
cv2.destroyAllWindows()

