# Last Robot SLAM System 🤖

robot_slam paketinden esinlenerek oluşturulmuş profesyonel SLAM sistemi. 6 tekerlekli robot ile Gazebo simülasyonunda harita oluşturma ve lokalizasyon.

## 🚀 Özellikler

### 📡 İki Aşamalı SLAM Yaklaşımı:
1. **Mapping Mode** - SLAM Toolbox ile harita oluşturma
2. **Localization Mode** - AMCL ile oluşturulan haritada konum belirleme

### ⚙️ Teknik Özellikler:
- **Robot:** 6 tekerlekli differential drive
- **Sensörler:** RPLidar S2 + Intel RealSense D435
- **SLAM:** SLAM Toolbox (Ceres solver)
- **Localization:** AMCL (Adaptive Monte Carlo Localization)
- **Simulation:** Gazebo + ROS2 Humble

## 📁 Dosya Yapısı

```
src/last_smooth_controller/
├── launch/
│   ├── slam_mapping.launch.py      # SLAM ile harita oluşturma
│   ├── amcl_localization.launch.py # AMCL ile lokalizasyon
│   └── gazebo_slam.launch.py       # Eski basit SLAM
├── config/
│   ├── slam_mapping_params.yaml    # SLAM parametreleri
│   ├── amcl_params.yaml           # AMCL parametreleri
│   └── slam_config.yaml           # Eski SLAM config
├── maps/                          # Oluşturulan haritalar
└── README.md
```

## 🎯 Kullanım

### 1️⃣ Harita Oluşturma (Mapping)

Yeni harita oluşturmak için:
```bash
ros2 launch last_smooth_controller slam_mapping.launch.py
```

Kaydedilmiş haritadan devam etmek için:
```bash
ros2 launch last_smooth_controller slam_mapping.launch.py use_saved_map:=true saved_map_path:=/path/to/saved_map
```

**Mapping sırasında:**
- Robot teleop ile hareket ettirin
- RViz'de haritanın oluşumunu izleyin
- Haritayı kaydetmek için SLAM Toolbox'ın "Serialize Map" butonunu kullanın

### 2️⃣ Lokalizasyon (AMCL)

Oluşturulan haritada konum belirleme için:
```bash
ros2 launch last_smooth_controller amcl_localization.launch.py
```

Özel harita dosyası ile:
```bash
ros2 launch last_smooth_controller amcl_localization.launch.py map_file:=/path/to/your/map.yaml
```

## 🔧 Parametre Optimizasyonları

### SLAM Parametreleri (slam_mapping_params.yaml):
- **Loop Closure:** Aktif
- **Scan Matching:** Gelişmiş algoritma
- **Resolution:** 0.05m
- **Transform Timeout:** 1.0s

### AMCL Parametreleri (amcl_params.yaml):
- **Particle Count:** 500-2000
- **Laser Model:** likelihood_field
- **Update Thresholds:** 0.2m / 0.5 rad

## 🛠️ Kurulum

1. **Bağımlılıkları yükleyin:**
```bash
sudo apt install ros-humble-slam-toolbox ros-humble-nav2-*
```

2. **Workspace'i build edin:**
```bash
cd ~/ros2_ws
colcon build --packages-select last_smooth_controller
source install/setup.bash
```

## 📊 Performans

- **LiDAR Frequency:** ~15 Hz
- **Map Update:** 5 saniyede bir
- **Transform Timeout:** 1.0s
- **Memory Usage:** ~40MB stack

## 🎮 Kontrolller

**Teleop tuşları:**
- `i` - İleri
- `k` - Geri  
- `j` - Sola dön
- `l` - Sağa dön
- `q` - Çıkış

## 🔍 Sorun Giderme

**SLAM mesaj düşürme:**
- `transform_timeout` artırın
- `tf_buffer_duration` artırın
- Robot hızını azaltın

**AMCL konverjans problemi:**
- Initial pose ayarlayın
- Particle sayısını artırın
- Laser model parametrelerini optimize edin

## 📈 İleri Seviye

### Harita Kaydetme:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

### Custom Parameters:
```bash
ros2 launch last_smooth_controller slam_mapping.launch.py slam_params_file:=/path/to/your/params.yaml
```

---

## 🏆 robot_slam Karşılaştırması

Bu sistem, orijinal `robot_slam` paketinin şu özelliklerini içerir:
- ✅ Mapping/Localization ayrımı
- ✅ Esnek parametre yönetimi  
- ✅ Map serialization
- ✅ AMCL entegrasyonu
- ✅ Professional launch file organization

**Ekstra özellikler:**
- 🚀 6-wheel differential drive support
- 🎯 RPLidar S2 + RealSense D435 integration
- ⚡ Optimized timing parameters
- 🎮 Gazebo simulation ready 