# .arena_mock

Selamat, kamu menemukannya. Ini arena tiruan 2D untuk latihan misi penuh tanpa Gazebo dan tanpa GPU. Cukup ROS 2 Jazzy dan RViz.

## Jalankan
    ./run.sh                              # simulator + RViz
    ./run.sh --ros-args -p capacity:=3    # kapasitas mekanisme 3 barang (sekali jalan)

Atau tanpa RViz: `python3 arena_sim.py`.

## Yang ada di dalamnya
Kapal start di dalam slot Spot 1. Kendalikan lewat `/asv/cmd_vel`. Dinamika orde satu (ada jeda respons), jadi PID-mu benar-benar perlu di-tuning.

| Topic | Tipe | Arah |
|---|---|---|
| `/asv/cmd_vel` | geometry_msgs/Twist | masuk (linear.x, angular.z) |
| `/asv/odom`, `/asv/pose`, TF `map` ke `base_link` | Odometry, PoseStamped | keluar |
| `/asv/battery` | sensor_msgs/BatteryState | keluar |
| `/arena/markers` | MarkerArray | keluar (untuk RViz) |
| `/mission_state` | String | keluar |
| `/mission/events` | String (JSON) | keluar (pick, place, gate_pass, collision, ftp_end, mission_done) |
| `/ftp/score` | String (JSON) | keluar |

Service Trigger: `/mission/start`, `/mission/stop`, `/mission/reset`, `/estop`, `/asv/pick`, `/asv/place`. Service SetBool: `/asv/buzzer`, `/asv/spray`.

## Aturan yang ditegakkan
- FTP: selesai 100, tiap tabrakan buoy -20, gagal boleh diulang maksimal 3 kali per arah.
- Pick hanya di slot Spot 1, place hanya di slot Spot 2 (jangkauan 1,4 m dari pusat kapal).
- Misi selesai bila semua barang di Spot 2, kapal docking di slot Spot 2, buzzer dan semprot menyala sekitar saat barang terakhir ditaruh.
- Dermaga dan tepi kolam bersifat padat. Buoy bisa ditembus tetapi dihitung tabrakan.

## Tidak ada
Kamera, deteksi, dan algoritma. Itu bagianmu. Arena ini hanya tahu geometri, bukan jawabannya.
