import cv2
import mediapipe as mp

cap= cv2.VideoCapture(0)
mpHands= mp.solutions.hands
hands= mpHands.Hands()
mpDraw= mp.solutions.drawing_utils
fingerCoordinates= [(8,6),(12,10),(16,14),(20,18)]
thumbCoordinate= (4,2)

while True:
    success, frame= cap.read()
    frame= cv2.flip(frame,1)
    #bgrden rgbye çevir mp ile çalışmak için
    frame_rgb= cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results= hands.process(frame_rgb)
    landmarks= results.multi_hand_landmarks
    if landmarks:
        handPoints=[]
        for handlms in landmarks:
            mpDraw.draw_landmarks(frame,handlms,mpHands.HAND_CONNECTIONS)

            for idx,lm in enumerate(handlms.landmark):
                h,w,c= frame.shape
                cx,cy = int(lm.x * w ), int(lm.y * h)
                handPoints.append((cx,cy))

        for point in handPoints:
            cv2.circle(frame,point,10,(0,255,0),cv2.FILLED)

        upCount=0
        for coordinate in fingerCoordinates:
            if handPoints[coordinate[0]][1] < handPoints[coordinate[1]][1]:
                upCount += 1
        if handPoints[thumbCoordinate[0]][0] < handPoints[thumbCoordinate[1]][0]:
            upCount +=1

        cv2.putText(frame, str(upCount),(30,130),cv2.FONT_HERSHEY_PLAIN,12,(255,0,0),12)





    cv2.imshow("finger counter",frame)
    cv2.waitKey(5)
    if cv2.waitKey(25) & 0xFF==ord("q"):
        break

cap.release
cv2.destroyAllWindows