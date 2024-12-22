import cv2

cap = cv2.VideoCapture(0)
body = cv2.CascadeClassifier("fullbody.xml")

while True:
    ret, frame = cap.read()
    frame = cv2.flip(frame,1)
    gri = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    bodies = body.detectMultiScale(frame,1.4,4)

    if cv2.waitKey(10) & 0xFF == ord("q"):
        break

    for x,y,w,h in bodies:
        cv2.rectangle(cap,(x,y),(x+w,y+h),(0,255,0),2)

    cv2.imshow("body", frame)

cap.release()
cv2.destroyAllWindows()