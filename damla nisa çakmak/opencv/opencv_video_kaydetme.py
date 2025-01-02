import cv2
import numpy as np

# cap= cv2.VideoCapture(0, cv2.CAP_DSHOW)
# while True:
#     ret, frame=cap.read()
#     frame= cv2.flip(frame, 1)
#     cv2.imshow("webcam", frame)
#     if cv2.waitKey(1) &  0xFF == ord("q"):
#         break
#
#     cv2.waitKey(30)
#
# cap.release()
# cv2.destroyAllWindows()

cap= cv2.VideoCapture(0,cv2.CAP_DSHOW)
name= "D:/opencv.deneme/video.avi"
codec= cv2.VideoWriter_fourcc('W','M','V','2')
framerate= 30
resolution= (640,480)
output= cv2.VideoWriter(name,codec,framerate,resolution)
while True:
    ret, frame=cap.read()
    frame= cv2.flip(frame,1)
    output.write(frame)
    cv2.imshow("webcam",frame)
    if cv2.waitKey(30) & 0xFF== ord("q"):
        break

output.release()
cap.release()