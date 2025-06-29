from ultralytics import YOLO

def main():
    # Eğitilmiş modeli yükle (en son eğitim klasörünü kullan)
    model_path = 'runs/detect/train_custom50epoch/weights/best.pt'  # Gerekirse güncelle
    model = YOLO(model_path)

    # Kameradan gerçek zamanlı tahmin yap
    model.predict(source=0, show=True, conf=0.5)

if __name__ == "__main__":
    main()
