# Magang PGR: Perception di VRX

Materi praktik **Perception 1 (Computer Vision)** dan **Perception 2 (LiDAR dan Fusion)** Magang PGR Barunastra ITS.
Simulasinya memakai **VRX** (Virtual RobotX): kapal WAM-V di danau Sydney Regatta, ditambah lintasan buoy merah, hijau, oranye, dan hitam.

```
kamera WAM-V ──> HSV / YOLO ──> deteksi buoy (kelas + kotak pixel)
                                        │
LiDAR 3D WAM-V ──> filter ──> sektor dan cluster      ▼
                        └──────────────────> fusion ──> posisi dan jarak buoy
```

> **Wajib sebelum sesi Sabtu 20.00:** selesaikan Langkah 1 sampai 5 dan pastikan simulasi bisa jalan.
> Unduhan cukup besar (torch lebih dari 2 GB, model VRX sekitar 160 MB), jangan dikerjakan saat sesi.

---

## Isi repo

| Path | Isi |
|---|---|
| `barunastra_perception/hsv_detector.py` | Deteksi buoy dari warna (TODO 1, 2, 3) |
| `barunastra_perception/yolo_detector.py` | Deteksi buoy dengan YOLO (TODO 4) |
| `barunastra_perception/cloud_obstacles.py` | Filter point cloud, sektor, clustering (TODO 5, 6) |
| `barunastra_perception/buoy_ranger.py` | Fusion kamera dan LiDAR (TODO 7, 8) |
| `barunastra_perception/hsv_tuner.py` | Slider untuk mencari rentang HSV |
| `barunastra_perception/dataset_capture.py` | Menyimpan gambar kamera untuk dataset |
| `barunastra_perception/twist_to_thrust.py` | `/cmd_vel` ke thruster WAM-V (untuk teleop) |
| `barunastra_perception/auto_drive.py` | Kapal jalan sendiri menyusuri lintasan (alat bantu) |
| `launch/vrx.launch.py` | VRX + lintasan buoy + twist_to_thrust + RViz |
| `worlds/perception_regatta.sdf` | World VRX Sydney Regatta dengan lintasan buoy |
| `models/buoy_vrx_yolov8n.pt` | Model YOLO contoh (4 kelas) |
| `test/` | Unit test untuk mengecek TODO |
| `tools/` | Pembagi dataset, training YOLO, pembuat world dan dataset |

---

## Prasyarat

- Ubuntu 24.04 dan ROS 2 Jazzy (`ros-jazzy-desktop`) sudah terpasang dari pertemuan sebelumnya
- RAM disarankan 16 GB (8 GB bisa, jalankan simulasi dengan `headless:=True`), ruang disk kosong beberapa GB
- Internet untuk unduhan pertama

---

## Langkah 1: paket apt

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-cv-bridge \
  ros-jazzy-vision-msgs ros-jazzy-rqt-image-view ros-jazzy-teleop-twist-keyboard \
  libeigen3-dev python3-pip python3-pytest git
```

## Langkah 2: paket Python (versinya dikunci)

```bash
pip install --user --break-system-packages \
  "numpy<2" "opencv-python<4.12" "setuptools<80" ultralytics
```

Kenapa dikunci?
- `opencv-python` 4.12 ke atas memaksa numpy 2, padahal `cv_bridge` ROS Jazzy dibuat dengan numpy 1.x.
  Gejalanya: `A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x`.
- `setuptools` 80 ke atas membuat `colcon build --symlink-install` gagal dengan
  `error: option --editable not recognized`.

Cek:
```bash
python3 -c "import cv_bridge, ultralytics, numpy; print(numpy.__version__)"
```
Hasil benar: `1.26.x` tanpa error.

## Langkah 3: build VRX (sekali saja)

```bash
mkdir -p ~/vrx_ws/src && cd ~/vrx_ws/src
git clone -b jazzy https://github.com/osrf/vrx.git
cd ~/vrx_ws
source /opt/ros/jazzy/setup.bash
# hemat RAM: satu paket sekali jalan, 2 proses compile
MAKEFLAGS="-j2" colcon build --symlink-install --parallel-workers 1
```
Hasil benar: `Summary: ... packages finished` tanpa `failed`. Lama build bergantung laptop, mulai lebih awal.

Laptop dengan RAM 16 GB ke atas boleh tanpa `MAKEFLAGS` dan `--parallel-workers` supaya lebih cepat.

## Langkah 4: ambil repo ini dan build

```bash
mkdir -p ~/perception_ws/src && cd ~/perception_ws/src
git clone https://github.com/ahnafyura/magang-perception-PGR.git barunastra_perception
cd ~/perception_ws
source /opt/ros/jazzy/setup.bash
source ~/vrx_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Supaya tidak perlu `source` setiap buka terminal:
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/vrx_ws/install/setup.bash" >> ~/.bashrc
echo "source ~/perception_ws/install/setup.bash" >> ~/.bashrc
```

> Urutan source penting: ROS, lalu VRX, lalu workspace ini.
> `--symlink-install` membuat perubahan file `.py` langsung terpakai tanpa build ulang.
> Build ulang hanya kalau menambah file baru.

## Langkah 5: uji simulasi

```bash
ros2 launch barunastra_perception vrx.launch.py
```
- Jalankan pertama kali butuh internet: Gazebo mengunduh model danau dan buoy (sekitar 160 MB). Tunggu sampai jendela Gazebo menampilkan danau.
- Hasil benar: Gazebo menampilkan WAM-V dan buoy, RViz menampilkan titik LiDAR dan gambar kamera.
- Cek dari terminal lain:
  ```bash
  ros2 topic list | grep wamv/sensors
  ```
  harus ada `.../front_left_camera_sensor/image_raw` dan `.../lidar_wamv_sensor/points`.

Laptop berat? Jalankan tanpa jendela Gazebo:
```bash
ros2 launch barunastra_perception vrx.launch.py headless:=True
```

## Menggerakkan kapal

```bash
# kendali keyboard: i maju, j/l belok, k berhenti (klik terminalnya dulu)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# atau kapal jalan sendiri menyusuri lintasan (jalankan saat kapal masih di posisi awal)
ros2 run barunastra_perception auto_drive
```
`twist_to_thrust` sudah dijalankan oleh launch file: perintah `/cmd_vel` diubah menjadi gaya thruster kiri dan kanan.

---

## Topic penting

| Topic | Tipe | Isi |
|---|---|---|
| `/wamv/sensors/cameras/front_left_camera_sensor/image_raw` | `sensor_msgs/Image` | Kamera depan kiri, 1280 x 720 |
| `/wamv/sensors/cameras/front_left_camera_sensor/camera_info` | `sensor_msgs/CameraInfo` | fx, fy, cx, cy |
| `/wamv/sensors/lidars/lidar_wamv_sensor/points` | `sensor_msgs/PointCloud2` | LiDAR 3D, 16 baris |
| `/cmd_vel` | `geometry_msgs/Twist` | Perintah gerak (lewat twist_to_thrust) |
| `/perception/hsv/detections` | `vision_msgs/Detection2DArray` | Keluaran HSV |
| `/perception/yolo/detections` | `vision_msgs/Detection2DArray` | Keluaran YOLO |
| `/perception/lidar/obstacles` | `sensor_msgs/PointCloud2` | Titik LiDAR yang lolos filter |
| `/perception/buoys` | `vision_msgs/Detection2DArray` | Buoy hasil fusion, posisi di `wamv/wamv/base_link` |

---

## Hari 1 (Sabtu): Computer Vision

| TODO | Berkas | Tugas |
|---|---|---|
| 1 | `hsv_detector.py` | Rentang HSV merah (dua rentang) dan hijau |
| 2 | `hsv_detector.py` | Morphology opening dan closing |
| 3 | `hsv_detector.py` | Contour jadi bounding box dan `Detection2D` |
| 4 | `yolo_detector.py` | Hasil YOLO jadi `Detection2D` |

```bash
ros2 run barunastra_perception hsv_tuner           # cari rentang HSV, tekan p untuk cetak
ros2 run barunastra_perception hsv_detector
ros2 run rqt_image_view rqt_image_view /perception/hsv/debug
ros2 run barunastra_perception yolo_detector
ros2 topic echo /perception/yolo/detections --field detections
```

## Hari 2 (Minggu): LiDAR dan Fusion

| TODO | Berkas | Tugas |
|---|---|---|
| 5 | `cloud_obstacles.py` | Filter titik: tinggi dan jarak |
| 6 | `cloud_obstacles.py` | Sudut tiap titik untuk sektor kiri, depan, kanan |
| 7 | `buoy_ranger.py` | Proyeksi titik 3D ke pixel (pinhole) |
| 8 | `buoy_ranger.py` | Titik mana yang masuk bounding box |

```bash
ros2 run barunastra_perception cloud_obstacles
ros2 run barunastra_perception yolo_detector       # detektor dari hari 1
ros2 run barunastra_perception buoy_ranger
```
Lihat hasilnya di RViz: display "Obstacle (filter)", "Cluster LiDAR", dan "Buoy (fusion)".

## Cek TODO dengan unit test

```bash
cd ~/perception_ws/src/barunastra_perception
python3 -m pytest test -v            # semua
python3 -m pytest test -k todo5      # hanya TODO 5
```
Di awal, test `todo...` memang gagal. Tujuanmu membuat semuanya lulus.

---

## Dataset dan model sendiri

1. Simpan gambar sambil kapal digerakkan ke banyak posisi dan arah:
   ```bash
   ros2 run barunastra_perception dataset_capture --ros-args -p out_dir:=$HOME/dataset_buoy
   ```
2. Upload ke Roboflow (atau CVAT, Label Studio), beri kotak dengan kelas `red`, `green`, `orange`, `black`, ekspor format **YOLOv8**.
3. Training (GPU lokal atau Google Colab):
   ```bash
   python3 tools/train_yolo.py <folder_ekspor>/data.yaml --epochs 50
   # tanpa GPU: tambahkan --device cpu dan kurangi epoch
   ```
4. Pakai modelmu:
   ```bash
   ros2 run barunastra_perception yolo_detector --ros-args -p model:=<path>/best.pt
   ```

---

## Masalah yang sering muncul

| Gejala | Solusi |
|---|---|
| `compiled using NumPy 1.x cannot be run in NumPy 2.x` | Ulangi Langkah 2 |
| `error: option --editable not recognized` saat build | Ulangi Langkah 2 (setuptools &lt; 80) |
| `Package 'vrx_gz' not found` | `source ~/vrx_ws/install/setup.bash` sebelum build dan launch |
| Gazebo diam lama di awal | Sedang mengunduh model (pertama kali saja), tunggu |
| Kapal sangat lambat atau tersendat | Laptop berat: `headless:=True`, tutup aplikasi lain |
| RViz kosong | Fixed Frame harus `wamv/wamv/base_link`, tunggu beberapa detik |
| `buoy_ranger` tidak mengeluarkan apa pun | Detektor (YOLO atau HSV) harus jalan, dan ada buoy di depan kamera |
| Folder workspace mengandung spasi | Pindahkan ke path tanpa spasi (launch VRX memecah path di spasi) |

---

Barunastra ITS, Divisi Programming
