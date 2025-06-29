from ultralytics import YOLO

# YOLOv8 modelini yükle (örneğin, 'yolov8n' nano versiyonu)
model = YOLO('yolov8n.pt')  # 'yolov8s.pt', 'yolov8m.pt', 'yolov8l.pt' gibi farklı versiyonlar da kullanılabilir

# Modeli eğit
model.train(
    data='/home/irem/Documents/PythonProjects/ObjectDetection/prob_green_yellow.v2i.yolov8',  # Veri kümesi yapılandırma dosyasının yolu
    epochs=30,         # Eğitim için epoch sayısı
    batch=16,          # Batch boyutu
    imgsz=640,         # Giriş görüntü boyutu
    name='yolov8_model'  # Eğitim sonuçlarının kaydedileceği klasör adı
)

# Eğitim sonrası modeli değerlendirme
metrics = model.val()

# Modeli tahmin için kullanma (isteğe bağlı)
#results = model.predict(source='/home/irem/Documents/PythonProjects/ObjectDetection/PrecisionLanding.v1i.yolov8/test/images', save=True)  # Test görüntüleri üzerinde tahmin yapar ve sonuçları kaydeder
şw
# Modeli kaydetme (isteğe bağlı)
model.export(format='pt')  # pt formatında dışa aktarım 
