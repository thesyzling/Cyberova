import cv2
cap=cv2.VideoCapture(0, cv2.CAP_DSHOW)
dosyaadi="C:/video/1.avi"
codec=cv2.VideoWriter_fourcc('W','M','V','2')
framerate=30
resolution=(640,480)

output=cv2.VideoWriter(dosyaadi,codec,framerate,resolution)

while True:
    ret, frame=cap.read()
    frame=cv2.flip(frame,1)
    cv2.imshow("webcab", frame)
    if cv2.waitKey(1) & 0xFF ==ord("q"):
#"q" tuşuna basıldığında kamera kapanır
        break

    cv2.waitKey(30)

cap.release()
cv2.destroyAllwindows()