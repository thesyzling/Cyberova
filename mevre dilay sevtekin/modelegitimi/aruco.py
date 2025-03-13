import cv2
import cv2.aruco as aruco
import numpy as np

# ArUco dictionary seçimi
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
parameters = aruco.DetectorParameters()


cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Kamera görüntüsü alinamiyor.")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
    detector = aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(gray)  

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)  

    cv2.imshow("ArUco Marker Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'): 
        break

cap.release()
cv2.destroyAllWindows()



