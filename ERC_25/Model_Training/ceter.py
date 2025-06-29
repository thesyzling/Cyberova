import cv2
from ultralytics import YOLO
import numpy as np

class SimpleKalman:
    def __init__(self, x, y):
        self.kalman = cv2.KalmanFilter(4, 2)
        self.kalman.measurementMatrix = np.array([[1,0,0,0],[0,1,0,0]], np.float32)
        self.kalman.transitionMatrix = np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]], np.float32)
        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
        self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.5
        self.kalman.statePre = np.array([[x],[y],[0],[0]], np.float32)
    def predict(self):
        pred = self.kalman.predict()
        return int(pred[0]), int(pred[1])
    def correct(self, x, y):
        measurement = np.array([[np.float32(x)], [np.float32(y)]])
        self.kalman.correct(measurement)


def live_detect_and_mark_center(model_path):
    print(f"Model yükleniyor: {model_path}")
    try:
        model = YOLO(model_path)
    except Exception as e:
        print(f"Model yüklenemedi: {e}")
        return

    print("Harici webcam açılıyor...")
    cap = cv2.VideoCapture(0)  # PC'nin kendi kamerası (ID: 0)
    if not cap.isOpened():
        print("PC'nin kamerası açılamadı! Lütfen bağlı olduğundan emin olun.")
        return
    print("PC'nin kamerası açıldı. Algılama başlıyor...")

    kalman = None
    detected = False

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Kameradan görüntü alınamadı!")
            break

        # Görüntüyü yatay eksende çevir (ayna efekti)
        frame = cv2.flip(frame, 1)

        try:
            results = model(frame)
        except Exception as e:
            print(f"Tahmin sırasında hata: {e}")
            break

        boxes = []
        for result in results:
            boxes_np = result.boxes.xyxy.cpu().numpy()
            boxes.extend(boxes_np)

        if boxes:
            x_min, y_min, x_max, y_max = map(int, boxes[0])
            center_x = int((x_min + x_max) / 2)
            center_y = int((y_min + y_max) / 2)
            if kalman is None:
                kalman = SimpleKalman(center_x, center_y)
            kalman.correct(center_x, center_y)
            detected = True
        elif kalman is not None:
            center_x, center_y = kalman.predict()
            detected = False

        if kalman is not None:
            kx, ky = kalman.predict()
            if detected:
                cv2.circle(frame, (center_x, center_y), 4, (0, 0, 255), -1)
            cv2.circle(frame, (kx, ky), 4, (0, 255, 0), -1)

        cv2.imshow("Live Target Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    model_path = "runs/detect/train_custom50epoch/weights/best.pt"
    live_detect_and_mark_center(model_path)
