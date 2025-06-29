import cv2
import cv2.aruco as aruco

def detect_aruco_markers():
   
    cap = cv2.VideoCapture(0)  
    if not cap.isOpened():
        print("Kamera açilamadi.")
        return

  
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)  
    parameters = aruco.DetectorParameters()  

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Kamera görüntüsü alinamiyor.")
            break

       
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
        
            aruco.drawDetectedMarkers(frame, corners, ids)

        
            for i in range(len(ids)):
                cv2.putText(frame, f"ID: {ids[i][0]}", (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    
        cv2.imshow('ArUco Marker Detection', frame)


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    detect_aruco_markers()