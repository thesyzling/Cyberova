import cv2
from ultralytics import YOLO

# YOLOv8 modelini yükle
model = YOLO('/home/irem/Documents/PythonProjects/ObjectDetection/runs/detect/yolov8_model11_prob/weights/best.pt')
GERCEK_GENISLIK_CM = 29.7  # A3 kağıdının kısa kenarı (cm)
ODAK_UZAKLIGI_PX = 471.4    # Kameranız için kalibre edin!

# Kamerayı başlat
cap = cv2.VideoCapture(2)  # 0, varsayılan kamerayı temsil eder

if not cap.isOpened():
    print("Kamera açilamadi!")
    exit()


while True:
    ret, frame = cap.read()
    if not ret:
        print("Kare alınamadı, çıkılıyor...")
        break

    # YOLOv8 ile tahmin yap
    results = model(frame)

    # Tahmin edilen sonuçları çizin
    annotated_frame = results[0].plot()

    # Tespit edilen kutuların merkezine yeşil nokta koy ve mesafeyi yazdır
    # if results[0].boxes is not None and results[0].boxes.xyxy is not None:
    #     for box in results[0].boxes.xyxy.cpu().numpy():
    #         x1, y1, x2, y2 = box[:4]
    #         center_x = int((x1 + x2) / 2)
    #         center_y = int((y1 + y2) / 2)
    #         cv2.circle(annotated_frame, (center_x, center_y), 5, (0, 255, 0), -1)  # yeşil nokta

    #         pixel_genislik = x2 - x1
    #         if pixel_genislik > 0:
    #             mesafe = (GERCEK_GENISLIK_CM * ODAK_UZAKLIGI_PX) / pixel_genislik
    #             print(f"Tahmini mesafe: {mesafe:.1f} cm")
    #             cv2.putText(
    #                 annotated_frame,
    #                 f"{mesafe:.1f} cm",
    #                 (center_x, center_y - 10),
    #                 cv2.FONT_HERSHEY_SIMPLEX,
    #                 0.7,
    #                 (0, 255, 0),
    #                 2
    #             )

    # Çerçeveyi göster
    cv2.imshow("YOLOv8 Detection", annotated_frame)

    # 'q' tuşuna basıldığında çık
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Kaynakları serbest bırak
cap.release()
cv2.destroyAllWindows()