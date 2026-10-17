# KELO Otonom Navigasyon

[English](README.md) | **Türkçe**

KELO Robile çok yönlü (omnidirectional) platformu için ROS 2 Jazzy navigasyon yığını. NVIDIA Jetson
Orin Nano üzerinde çalışacak şekilde Docker imajı olarak paketlenmiştir.

| Bileşen | Teknoloji |
|---|---|
| Haritalama | slam_toolbox |
| Konumlandırma | AMCL, Marvelmind iç mekân konumlandırma |
| Navigasyon | Nav2 (MPPI; yollar ileri yönde izlenir, platformun kendisi çok yönlüdür) |
| Sensörler | Hokuyo URG-04LX (2B LiDAR), Intel RealSense D435 (derinlik), Marvelmind hedgehog |
| Güvenlik | İki çarpışma monitörü (LiDAR + derinlik), hız kapısı, watchdog, konum sapma monitörü |

Yığın, robotun düşük seviyeli sürücüsünü değiştirmez. Robotla arayüz `/cmd_vel`, `/odom` ve
`odom → base_link` dönüşümü ile sınırlıdır.

> **Güvenlik uyarısı.** Bu projedeki güvenlik işlevleri yazılım ile gerçeklenmiştir ve sertifikalı
> değildir. URG-04LX ve D435 güvenlik sınıfı sensörler değildir. İnsanlarla paylaşılan alanlarda
> çalışma için sertifikalı bir güvenlik lazer tarayıcısı gereklidir. Donanımsal acil durdurma bu
> yazılımdan bağımsız çalışır ve her testte erişilebilir olmalıdır.

## İçindekiler

1. [Başlarken](#başlarken)
2. [Proje yapısı](#proje-yapısı)
3. [Mimari](#mimari)
4. [Hız komutu yolu](#hız-komutu-yolu)
5. [Hızlı başlangıç](#hızlı-başlangıç)
6. [Yapılandırma](#yapılandırma)
7. [Jetson kurulumu](#jetson-kurulumu)
8. [Devreye alma prosedürü](#devreye-alma-prosedürü)
9. [Sensör arabası ile haritalama](#sensör-arabası-ile-haritalama)
10. [Kayıt ve yeniden oynatma](#kayıt-ve-yeniden-oynatma)
11. [Güvenlik sistemi](#güvenlik-sistemi)
12. [Simülatör](#simülatör)
13. [Test](#test)
14. [İşletim ve bakım](#i̇şletim-ve-bakım)
15. [Açık konular](#açık-konular)
16. [Bilinen kısıtlar](#bilinen-kısıtlar)

## Başlarken

Simülatör herhangi bir 64 bit Linux bilgisayarda çalışır (x86-64 veya ARM64, ör. Jetson). Robot veya
sensör gerekmez.

1. Docker Engine ve Docker Compose eklentisini kurun
   ([kurulum](https://docs.docker.com/engine/install/)), ardından kullanıcınıza Docker yetkisi verin:
   ```bash
   sudo usermod -aG docker $USER      # sonrasında oturumu kapatıp yeniden açın
   ```
2. Projeyi indirin:
   ```bash
   git clone https://github.com/beratbalcibaran/kelo_docker.git
   cd kelo_docker
   ```
3. İmajı derleyip kontrol panelini başlatın (ilk derleme 15–30 dk sürer, yaklaşık 5 GB yer kaplar):
   ```bash
   docker compose --profile gui up --build
   ```
4. Logda `KELO interface ready` göründüğünde
   `http://localhost:6080/vnc.html?autoconnect=true&resize=scale` adresini açın ve logda yazan VNC
   parolasını girin.
5. Panelde kaynağı `sim` bırakın ve şu adımları izleyin:

   **Harita çıkarma**
   1. **Start Mapping**: simülatör ve RViz açılır, harita oluşmaya başlar.
   2. Panel penceresine bir kez tıklayın, ardından WASD veya ok tuşlarıyla odanın her yerini dolaşın;
      RViz'de duvarlar kapanana kadar sürün.
   3. **Save Map**: bir isim verin (ör. `oda`). Logda `Saved: ...` ve `current.yaml -> ...` görünür.
   4. **Stop Mapping**.

   **Harita üzerinde navigasyon**
   1. **Start Navigation**: robot, harita çıkarmaya başladığı noktada yeniden başlar. Bu nokta
      haritada `x=0, y=0`'dır.
   2. **Set Initial Pose**: X `0`, Y `0`, Yaw `0` girin. Robot gerekirse yerinde döner; logda
      `LOCALIZED` görünene kadar bekleyin.
   3. **Send Goal**: haritada boş bir nokta girin, ör. X `0`, Y `2`, Yaw `0` (başlangıcın 2 m solu).
      Robot varınca logda `Goal reached.` görünür. Hedef RViz'den **2D Goal Pose** ile de verilebilir.
   4. İş bitince **Stop Navigation**.

   **Localize** (konum vermeden) Marvelmind hizalama dosyasına ihtiyaç duyar; yeni indirilen kopyada bu
   dosya yoktur, bu durumda tüm haritada arar ve yanlış yeri seçebilir. Simülatörde
   **Set Initial Pose** kullanın.
6. Durdurmak için `Ctrl+C` veya `docker compose --profile gui down`.

Depoda harita yoktur: önce **Start Mapping** / **Save Map** ile bir harita oluşturun. Haritalar
(`data/maps`), kayıtlar (`data/bags`) ve Marvelmind örnekleri yerelde kalır, depoya dahil değildir.

## Proje yapısı

```
kelo_docker/
├── README.md, README.tr.md     bu dokümantasyon (İngilizce, Türkçe)
├── Dockerfile                  imajı oluşturur: ROS 2 Jazzy, Nav2, sürücüler, bu paket
├── docker-compose.yml          başlatma profilleri (gui, sim, sim-nav, replay, production, mapping, ...)
├── docker/                     container içinde çalışan scriptler
├── scripts/                    testler ve Jetson kurulumu, ana bilgisayarda çalışır
├── systemd/kelo.service        robot yazılımını açılışta otomatik başlatır (Jetson)
├── data/                       haritalar ve kayıtlar, container'a /data olarak bağlanır
└── src/kelo_bringup/           ROS 2 paketi (robot yazılımının tamamı)
```

| Yol | Görevi |
|---|---|
| `docker/entrypoint.sh` | Container başlarken ROS ortamını hazırlar |
| `docker/start-gui.sh` | Tarayıcıdaki kontrol panelini başlatır (noVNC + panel + RViz) |
| `docker/run-stack.sh` | Yazılımı panelsiz başlatır (arayüzsüz profiller ve servis kullanır) |
| `docker/cyclonedds.xml` | ROS 2 iletişimi için ağ ayarları |
| `scripts/test_all.sh` | İmajı derler ve tüm testleri çalıştırır |
| `scripts/sim_e2e_test.sh` | Simülatörde uçtan uca testler |
| `scripts/replay_regression.sh` | Bir kayıttan haritayı yeniden çıkarır ve referansla karşılaştırır |
| `scripts/install_jetson.sh` | Projeyi Jetson'a kurar (`/opt/kelo`, udev, systemd) |
| `data/maps/` | Kayıtlı haritalar (yerel, depoda yok); `current.yaml` navigasyonda kullanılan haritayı gösterir |
| `data/mm_samples_*.csv` | Haritalama sırasında kaydedilen Marvelmind hizalama örnekleri (yerel) |
| `data/bags/` | Haritalama sırasında yapılan sensör kayıtları (depoda yok) |
| `data/hardware.yaml` | `config/hardware.yaml` için isteğe bağlı yerel ayar (depoda yok) |

`src/kelo_bringup/` içinde:

| Yol | Görevi |
|---|---|
| `config/hardware.yaml` | **Buradan başlayın.** Takılı sensörler, seri portlar, sensör konumları, robot boyutu, odometri düzeltmesi |
| `config/safety.yaml` | Hız sınırları, durma ve yavaşlama bölgeleri, derinlik kamerası filtresi |
| `config/nav2_params.yaml` | Konumlandırma (AMCL), yol planlama ve takip (Nav2), costmap'ler |
| `config/slam_toolbox.yaml` | Harita çıkarma ayarları |
| `config/sim_world.yaml` | Simülatör odası, engeller ve odometri hatası |
| `launch/bringup.launch.py` | Her şeyi başlatan tek giriş noktası |
| `gui.py` | Kontrol paneli (butonlar, WASD ile sürüş) |
| `urdf/kelo.urdf.xacro` | Robot modeli: gövde ve sensör konumları |
| `rviz/kelo.rviz` | RViz görünüm ayarları |
| `udev/99-kelo-serial.rules` | USB sensörler için sabit cihaz adları |
| `test/` | Birim testleri (pytest) ve referans verileri |

`src/kelo_bringup/kelo_bringup/` içindeki programlar:

| Dosya | Görevi |
|---|---|
| `stack.py`, `common.py` | Kaynağa, moda ve takılı sensörlere göre hangi programların başlayacağına karar verir |
| `cmd_vel_gate.py` | Robota hız komutu gönderen tek program; hızı sınırlar, komut gelmezse durdurur |
| `cloud_filter.py` | Derinlik görüntüsünü engel nokta bulutuna çevirir |
| `odom_filter.py` | Robotun tekerlek odometrisini düzeltir |
| `status_node.py` | Genel durumu raporlar ve konumlandırma durumunu tutar |
| `localize.py` | Robotun haritadaki konumunu bulur (GUI: Localize) |
| `send_goal.py` | Navigasyon hedefi gönderir (GUI: Send Goal) |
| `waypoint_tool.py` | İsimli konumları kaydeder ve aralarında sürer |
| `save_map.py` | Mevcut haritayı kaydeder (GUI: Save Map) |
| `marvelmind_bridge.py`, `mm_monitor.py`, `mm_recorder.py`, `mm_calibrate.py` | Marvelmind: konum dönüşümü, tutarlılık kontrolü, örnek kaydı, harita hizalaması |
| `odom_test.py` | Gerçek robotta odometri hatasını ölçer |
| `map_compare.py` | İki haritayı karşılaştırır |
| `wait_for_tf.py` | Açılışta gerekli koordinat sistemleri hazır olana kadar bekler |
| `logic.py` | Ortak hesaplamalar (birim testleriyle test edilir) |
| `sim_robot.py`, `sim/world.py` | Simülatör: robot, sensörler ve çarpışmalar |
| `e2e.py` | Uçtan uca test senaryoları |

### Temel kavramlar

| Terim | Anlamı |
|---|---|
| ROS 2 | Robot yazılım altyapısı; programlar (node) `/scan` gibi isimli topic'ler üzerinden mesajlaşır |
| Docker imajı / container | Tüm bağımlılıklarıyla paketlenmiş ortam; bilgisayara başka bir şey kurmak gerekmez |
| Profil | `docker-compose.yml` içindeki bir başlatma ayarı, `--profile` ile seçilir |
| Odometri | Tekerleklerden (veya LiDAR'dan) gelen konum tahmini |
| SLAM / haritalama | Sürerken harita çıkarmak |
| Konumlandırma (AMCL) | Robotun mevcut bir harita üzerindeki konumunu bulmak |
| Costmap | Yol planlamada kaçınılacak alanların haritası |
| Çarpışma monitörü | Etrafındaki bir bölgeye engel girince robotu durdurur veya yavaşlatır |
| TF / frame | Koordinat sistemleri (`map`, `odom`, `base_link`, sensörler) ve aralarındaki dönüşümler |

## Mimari

Gerçek robot, simülatör ve kayıt (bag) oynatma aynı launch dosyasını
(`launch/bringup.launch.py` → `kelo_bringup/stack.py`) aynı düğümler ve parametrelerle kullanır.
Yalnızca kaynak katmanı değişir. Simülatör tüm sensörleri donanımla aynı topic, frame ve mesaj
tipleriyle yayınlar.

```
                 ┌─ hardware: urg_node, realsense (yalnızca derinlik), marvelmind_ros2
 kaynak katmanı ─┼─ sim     : sim_robot (robot ve tüm sensörler, hatalı odometri modeli)
                 └─ replay  : ros2 bag play --clock (use_sim_time)
                        │  /scan  /odom  derinlik görüntüsü + camera_info  /hedgehog_pos_ang
                        ▼
 odometri       odom_source:=robot → odom_filter (hıza bağlı düzeltme) | odom_source:=lidar → rf2o
 ortak          robot_state_publisher, cloud_filter (derinlik → engel bulutu), marvelmind_bridge,
                cmd_vel_gate, collision_monitor_teleop, status_node
 mod            mapping: slam_toolbox (+ bag kaydı) | navigation: AMCL + Nav2 + mm_monitor | sensors

 Nav2 → velocity_smoother → collision_monitor ──→ /cmd_vel_safe ─────┐
 teleop/GUI → /cmd_vel_teleop → collision_monitor_teleop ────────────┴→ cmd_vel_gate → /cmd_vel → robot

 TF: map → odom → base_link → {laser, camera_link → camera_depth_optical_frame, hedgehog_link}
```

### Odometri sahipliği

`odom → base_link` dönüşümünün her zaman tek bir yayıncısı vardır.

| Yapılandırma | TF yayıncısı | Nav2'nin kullandığı odometri topic'i |
|---|---|---|
| `odom_source:=robot`, `odom_mode: corrected` (varsayılan) | `odom_filter` (sürücü TF'i kapalı olmalı) | `/odom_corrected` |
| `odom_source:=robot`, `odom_mode: driver` | robot sürücüsü (replay'de `/odom`'dan yeniden üretilir) | `/odom` |
| `odom_source:=lidar` | rf2o | `/odom_rf2o` |

MPPI kontrolcüsü dahil odometri okuyan tüm Nav2 düğümleri, aktif TF yayıncısının topic'ine abone olur.

### Sensör yapılandırması

Takılı sensörler `hardware.yaml → sensors` altında tanımlanır. Bir sensör devre dışı bırakıldığında
sürücüsü, costmap katmanı ve çarpışma monitörü kaynağı tüm kaynak tiplerinde birlikte kaldırılır.

## Hız komutu yolu

Yığın gövde hızları yayınlar; robot sürücüsü bunları teker komutlarına dönüştürür.

- **Topic:** `/cmd_vel`, tip `Twist` veya `TwistStamped` (`hardware.yaml → robot.cmd_vel_type`).
- **Alanlar:** `linear.x` ileri (m/s), `linear.y` yanal (m/s), `angular.z` dönüş hızı (rad/s).
- **Tek yayıncı:** `/cmd_vel` yayınlayan tek düğüm `cmd_vel_gate`'tir. Nav2, klavye ile sürüş ve
  konumlandırma dönüşü önce bir çarpışma monitöründen, ardından kapıdan geçer.
- **Kapı davranışı:** hızı 0,30 m/s ve 0,80 rad/s ile sınırlar; 0,5 s boyunca komut gelmezse, güvenlik
  durdurması aktifken ve kapanışta (SIGINT ve SIGTERM) sıfır hız yayınlar.
- **İstisna:** `odom_test calibrate-rotate` yalnızca kalibrasyon amacıyla doğrudan sürücüye yazar ve
  kapı çalışırken başlamayı reddeder.

## Hızlı başlangıç

### Grafik arayüz

Kontrol paneli noVNC üzerinden tarayıcıda çalışır.

```bash
docker compose --profile gui up --build            # iş istasyonu, varsayılan kaynak: sim
docker compose --profile gui-hardware up --build   # donanımlı Jetson, varsayılan kaynak: hardware
```

Logda `KELO interface ready` ve bir VNC parolası göründüğünde
`http://localhost:6080/vnc.html?autoconnect=true&resize=scale` adresini açın. Sabit parola için
`KELO_VNC_PASSWORD` tanımlayın; aksi halde her başlatmada rastgele parola üretilir.

Panelden kaynak (sim, hardware, replay) ve odometri kaynağı (robot, lidar) seçilir; Start Mapping,
Save Map, Stop Mapping, Start Navigation, Set Initial Pose, Localize, Send Goal ve Stop Navigation
işlevleri sunulur. WASD ve ok tuşları robotu `collision_monitor_teleop` ve `cmd_vel_gate` üzerinden
sürer.

RViz ayrı ve denetlenen bir süreç olarak çalışır. Sonlanırsa panel onu robot, sensör ve güvenlik
düğümlerini etkilemeden yeniden başlatır.

### Arayüzsüz profiller

| Profil | İşlev |
|---|---|
| `sim` | Haritalama ve bag kaydı ile simülatör |
| `sim-nav` | Kayıtlı harita üzerinde AMCL ve Nav2 ile simülatör |
| `replay` | Kayıtlı bag'den haritalama (`KELO_BAG`) |
| `production` | Robot navigasyonu (systemd tarafından kullanılır); `KELO_MODE=sensors` robotu sabit tutar |
| `mapping` | Robot ile haritalama; sensör arabası için `KELO_ODOM_SOURCE=lidar` |

`docker/run-stack.sh` tarafından okunan ortam değişkenleri:

| Değişken | Değerler |
|---|---|
| `KELO_SOURCE` | `hardware`, `sim`, `replay` |
| `KELO_MODE` | `navigation`, `mapping`, `sensors` |
| `KELO_ODOM_SOURCE` | `robot`, `lidar` |
| `KELO_MAP` | harita YAML yolu |
| `KELO_BAG` | bag dizini, ör. `/data/bags/<bag>` |
| `KELO_INITIAL_POSE` | `"x y yaw_derece"` (varsayılan `0 0 0`) |
| `KELO_EXTRA_ARGS` | ek launch argümanları |

## Yapılandırma

| Dosya | İçerik |
|---|---|
| `config/hardware.yaml` | Sensörler, topic'ler, frame'ler, montaj konumları, odometri düzeltme tabloları |
| `config/safety.yaml` | Hız kapısı, teleop çarpışma monitörü, derinlik filtresi, Marvelmind monitörü, durum |
| `config/nav2_params.yaml` | AMCL, Nav2, costmap'ler, Nav2 çarpışma monitörü |
| `config/slam_toolbox.yaml` | SLAM parametreleri |
| `config/sim_world.yaml` | Simülatör dünyası ve odometri hata modeli |

Yollar `src/kelo_bringup/` dizinine görelidir.

## Jetson kurulumu

```bash
sudo ./scripts/install_jetson.sh --enable-service
```

Betik projeyi `/opt/kelo` dizinine kurar, imajı derler, udev kurallarını yükler ve `kelo` systemd
servisini kaydeder.

Gereksinimler:

- Docker ve Docker Compose eklentisi içeren JetPack.
- Robot bilgisayarına kablolu ağ bağlantısı.
- `/etc/kelo/kelo.env` içindeki `ROS_DOMAIN_ID` değerinin robot bilgisayarıyla aynı olması.

DDS ara katmanı Cyclone DDS'tir (`docker/cyclonedds.xml`). Seri cihaz yolları
`hardware.yaml → serial` altında yapılandırılır (bkz. `ls /dev/serial/by-id/`).

## Devreye alma prosedürü

### 1. Donanım kontrolü (robot sabit)

```bash
KELO_MODE=sensors docker compose --profile production up
```

`ros2 topic hz /scan`, RViz'deki TF ağacı ve periyodik durum satırını
(`docker logs kelo-production | grep STATUS`) doğrulayın.

### 2. Odometri karakterizasyonu

Bu adımda robot hareket eder. Açık alan ve düşük hız kullanın, acil durdurma elinizin altında olsun.

```bash
docker compose run --rm production ros2 run kelo_bringup odom_test straight --speed 0.1 --distance 2.0
docker compose run --rm production ros2 run kelo_bringup odom_test rotate --rate 0.6 --turns 1
docker compose run --rm production ros2 run kelo_bringup odom_test summary
```

Ölçülen KELO odometrisi yaw açısını belirgin şekilde ve hıza bağlı olarak eksik raporlar: bir fiziksel
tur 0,6 rad/s'de yaklaşık 68° (k ≈ 5,3), 1,5 rad/s'de 42° (k ≈ 8,6) olarak görülür. Doğrusal hata
0,06 m/s'de −%0,6 ile 0,15 m/s'de −%1,8 arasındadır.

Sonuçları `hardware.yaml → odom_correction` altına girin (`yaw_k: wz/k`, `linear_error: v/e`). Noktalar
arasında doğrusal interpolasyon yapılır, tablo dışında uç değerler kullanılır. `odom_filter` ters
düzeltmeyi her adımda uygular.

**Kapı sınırının üzerinde kalibrasyon.** 0,80 rad/s üzerinde k ölçmek için yığını durdurup çalıştırın:

```bash
docker compose down
docker compose run --rm production ros2 run kelo_bringup odom_test calibrate-rotate --rate 1.5 --turns 3
```

Bu mod `cmd_vel_gate`, hız sınırı ve çarpışma monitörlerini devre dışı bırakır. Yerinde dönüşle sınırlıdır
(≤ 1,6 rad/s, ≤ 3 tur), onay için `CALIBRATE` yazılmasını gerektirir, her çıkışta sıfır hız yayınlar ve
kapı aktifken çalışmayı reddeder. İvmelenme rampası k'yı bir miktar düşük ölçtürdüğünden en az üç tur
kullanın. Üretimde kullanılmaz.

### 3. Haritalama

```bash
docker compose --profile mapping up
docker compose exec mapping ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_teleop
docker compose exec mapping ros2 run kelo_bringup save_map warehouse
```

Yavaş sürün ve döngüyü kapatın. `save_map`, `/data/maps/warehouse_<tarih>.*` dosyalarını (PGM, YAML ve
slam_toolbox poz grafiği) yazar ve `current.yaml`'ı günceller. Geçici nesneleri kaldırmak ve yasak
bölgeleri işaretlemek için PGM'i düzenleyin.

Haritalama sırasında sensör verisi `/data/bags/map_<kaynak>_<tarih>` altına kaydedilir (tarama, odometri,
derinlik görüntüsü ve camera_info, Marvelmind, TF; zstd ile MCAP, yaklaşık 1 MB/s). Nokta bulutu
oynatma sırasında derinlik görüntüsünden yeniden üretildiği için kaydedilmez. Kaydı kapatmak için
`KELO_EXTRA_ARGS=record:=false` kullanın. Marvelmind etkinse hizalama örnekleri
`/data/mm_samples_<tarih>.csv` dosyasına yazılır.

### 4. Marvelmind hizalaması

```bash
docker compose exec mapping ros2 run kelo_bringup mm_calibrate /data/mm_samples_<tarih>.csv
```

Araç RMS, ortalama, maksimum ve 95. yüzdelik hataları raporlar. RMS 0,15 m'yi aşarsa hizalama
reddedilir. Başarılı olursa `/data/marvelmind_alignment.yaml` yazılır ve başlangıçta
`map → marvelmind_map` statik dönüşümü olarak yayınlanır.

### 5. Navigasyon

```bash
docker compose --profile production up -d     # veya: systemctl start kelo
```

AMCL `current.yaml` üzerinde konumlandırma yapar. Nav2, `map → base_link` mevcut olduğunda başlar.

**Başlangıç pozu.** Tek bir Marvelmind hedgehog konum sağlar, yön (heading) sağlamaz.

| Yöntem | Davranış |
|---|---|
| `localize` (GUI: Localize) | Konum Marvelmind'dan, yön bilinmiyor; robot AMCL yakınsayana kadar güvenlik zinciri üzerinden yerinde 360° döner, ardından `LOCALIZED` yayınlar |
| `localize --global` | Dönüşlü AMCL global konumlandırma; Marvelmind konumu yoksa otomatik seçilir |
| `localize --yaw 90` (GUI: Set Initial Pose) | Konum Marvelmind'dan, yön kullanıcıdan |
| `localize --x 1 --y 2 --yaw 90` | Tam poz kullanıcıdan |
| `KELO_INITIAL_POSE="x y yaw_derece"` | Başlangıçta sabit poz (ör. şarj istasyonu) |

Robot düşük belirsizlikle zaten `LOCALIZED` durumundaysa `localize` AMCL'e dokunmaz ve 3 koduyla çıkar;
GUI `localize --force` çalıştırmadan önce onay ister. Durum `status_node` tarafından
`/kelo/localization_state` üzerinde tutulur. `send_goal` ve `waypoint_tool`, durum `LOCALIZING` veya
`FAILED` iken hedef göndermez; RViz'den gönderilen hedefler bu kontrole tabi değildir.

İkinci bir hedgehog (Marvelmind eşli işaretçiler, ≥ 0,3 m aralıklı) yönü doğrudan sağlar ve
konumlandırma dönüşüne gerek bırakmaz.

### 6. Ara noktalar

Ara noktalar harita koordinatlarında `/data/waypoints.yaml` dosyasında saklanır.

```bash
docker compose exec production ros2 run kelo_bringup waypoint_tool add dock
docker compose exec production ros2 run kelo_bringup waypoint_tool add-click shelf_a
docker compose exec production ros2 run kelo_bringup waypoint_tool list
docker compose exec production ros2 run kelo_bringup waypoint_tool go dock shelf_a
docker compose exec production ros2 run kelo_bringup waypoint_tool loop dock shelf_a --count 10
```

`add` robotun mevcut pozunu kaydeder; `add-click` RViz'de seçilecek bir nokta veya hedefi bekler.

## Sensör arabası ile haritalama

LiDAR, derinlik kamerası ve Marvelmind hedgehog bir arabaya monte edilerek robot olmadan harita
oluşturulabilir. Bu durumda odometri LiDAR'dan (rf2o) sağlanır.

1. Sensörleri LiDAR yatay ve tercihen robottakiyle aynı yükseklikte (yaklaşık 20 cm) olacak şekilde
   monte edin. Ölçülen konumları `hardware.yaml → mounts` altına girin.
2. Sensörleri USB ile Jetson'a bağlayın.
3. Haritalamayı başlatın:
   - GUI: `docker compose --profile gui-hardware up --build`, kaynak `hardware`, odometri `lidar`.
   - Arayüzsüz: `KELO_ODOM_SOURCE=lidar docker compose --profile mapping up`.
4. ≤ 0,3 m/s hızla ilerleyin, yavaş dönün ve döngüyü kapatmak için başlangıç noktasına dönün.
5. Haritayı kaydedin ve `mm_calibrate` çalıştırın.
6. Haritayı robotta kullanmadan önce `mounts` değerlerini robotun ölçüleriyle güncelleyin.

Simülasyonda araba ile haritalama, robot odometrisiyle yapılan haritalamayla pratikte özdeş bir harita
üretmiştir (F1 0,999). Bu mod henüz fiziksel sensörlerle doğrulanmamıştır.

## Kayıt ve yeniden oynatma

Haritalama kayıtları aynı yığın üzerinden yeniden oynatılabilir. Kayıttaki `/tf` oynatılmaz; SLAM veya
AMCL `map → odom`'u, odometri katmanı `odom → base_link`'i yeniden üretir.

```bash
KELO_BAG=/data/bags/map_hardware_<tarih> docker compose --profile replay up
./scripts/replay_regression.sh data/bags/map_hardware_<tarih> data/maps/<referans>.yaml
./scripts/replay_regression.sh <bag> <referans.yaml> --odom lidar --min-f1 0.85
```

`replay_regression.sh` bag'den harita üretir ve `map_compare` ile referansla karşılaştırır (0,10 m
toleransla dolu hücre kesinlik, duyarlılık ve F1).

## Güvenlik sistemi

| Durum | Tepki |
|---|---|
| 0,5 s komut yok | `cmd_vel_gate` sıfır hız yayınlar |
| Durma bölgesinde engel | Çarpışma monitörü robotu durdurur (Nav2 ve teleop) |
| Yavaşlama bölgesinde engel (merkezden 1,025 m önde) | Hız %40'a düşürülür |
| 1,2 s (Nav2) / 1,0 s (teleop) içinde öngörülen çarpışma | Hız azaltılır (`FootprintApproach`) |
| LiDAR düzlemi dışındaki engel | Derinlik kamerası ile algılanır (0,30–2,5 m, yükseklik 0,06–1,20 m) |
| LiDAR veya kamera verisi 1,0 s'den eski | Veri gelene kadar robot durdurulur |
| Engel kalktı | Temizleme bulutu ile costmap'ten silinir; navigasyon devam eder |
| Marvelmind ile AMCL 2 s boyunca > 1,0 m farklı | `/kelo/safety_stop` etkin, sıfır hız |
| Marvelmind verisi yok | `MM_LOST` olarak raporlanır; durdurma yapılmaz |
| Container veya servis kapanışı | Sıfır hız yayınlanır; hata durumunda otomatik yeniden başlatma |

Hız sınırları `safety.yaml` (0,30 m/s, 0,80 rad/s) ≥ hız yumuşatıcı ≥ MPPI sırasıyla uygulanır; bu sıra
bir birim testi ile doğrulanır.

**Footprint ve durma bölgesi.** Footprint 0,85 × 0,53 m'dir. Durma bölgesi yöne bağlıdır
(`velocity_polygon`): ileri merkezden 0,76 m (ön kenardan 0,335 m ötede), yanal 0,465 m, geri 0,625 m ve
yerinde dönüş için yaklaşık 0,59 m yarıçaplı sekizgen. Sabit bir ön bölge robotun duvardan dönerek veya
geri giderek uzaklaşmasını engellerdi; yöne bağlı bölge yalnızca engele doğru hareketi engeller.

**Sensör kapsamı.** LiDAR önde ortada (tarama düzlemi yerden 0,20 m) takılıdır ve ±92° kullanır
(`hardware.yaml → lidar`); daha geniş ışınlar robotun kendi ön yüzüne çarpar. Derinlik kamerası öne
bakar. Bu nedenle yanlar ve arka hiçbir sensör tarafından görülmez.

**Kamera kör bölgesi.** D435 0,30 m altında güvenilir ölçüm yapmaz ve yatay görüş açısı 87°'dir. Yalnızca
kameranın gördüğü bir engelin durma bölgesine girmeden kaybolmaması için durma bölgesinin tüm ön kenarı
`min_range` ötesinde ve görüş açısı içinde olmalıdır (`test_config.py::test_camera_blind_zone_rule`).

**Simülasyonda ölçülen tepki** (yaklaşık 0,20 m/s, engel ön kenardan 0,50 m ötede beliriyor):

| Senaryo | Sonuç |
|---|---|
| Nav2 ile sürüş | 0,5–0,8 s içinde durma, 0,39–0,44 m mesafe |
| İleri tuşu basılı teleop | 0,7–0,9 s içinde durma, 0,34–0,38 m mesafe; tuş basılı kaldıkça hareket yok |
| Önden geçen kişi | Temas yok |
| Kamera veya LiDAR bağlantısı kesik | 1,25–1,35 s içinde durma |

**İzleme topic'leri:** `/diagnostics`, `/kelo/gate_state`, `/kelo/mm_state`, `/kelo/localization_state`,
`/collision_monitor_state`. Her 10 s'de bir özet satırı loglanır
(`docker logs kelo-production | grep STATUS`).

## Simülatör

`sim_robot` (`config/sim_world.yaml` ile yapılandırılır) bir fizik motoru değil, yazılım zincirini test
etmek için hafif bir simülatördür. Sağladıkları:

- Kutular, masalar ve bölmeler içeren 3B dünya.
- Sensör yüksekliğinde ışın izleme ile LiDAR.
- D435 eşdeğeri derinlik görüntüleri (16UC1, optik frame, zemin dahil).
- Gürültü, sıçrama ve kesinti içeren Marvelmind konumları.
- İvme sınırlı ve 0,5 s komut zaman aşımlı çok yönlü robot.
- Çarpışma algılama.
- Ölçülmüş KELO odometri hata modeli (varsayılan) veya ideal odometri (`sim_odom_model:=ideal`).

Test arayüzleri: `/sim/ground_truth`, `/sim/contacts`, `/sim/obstacles` ve
`ros2 param set /sim_robot camera_enabled false`. Bu topic'ler yalnızca testlerde kullanılır.

## Test

```bash
./scripts/test_all.sh                 # imaj derleme, birim ve uçtan uca testler (45–60 dk)
./scripts/test_all.sh --unit-only     # yalnızca birim testleri (~5 s)
./scripts/sim_e2e_test.sh A B         # seçili uçtan uca aşamalar
```

**Birim testleri (pytest)** yapılandırma tutarlılığını (footprint, hız sınırları, bölgeler, kör bölge,
kaynaklar), odometri modeli ve tersini, simülatör ışın izleme ve çarpışmayı, tek launch yolu kurallarını
(özdeş ortak düğümler, tek TF sahibi, tutarlı odometri topic'leri) ve konumlandırma mantığını kapsar.
Tüm sensörleri gerektiren testler yığını bütün sensörler açık olacak şekilde kurar; bu nedenle sonuçlar
`hardware.yaml → sensors` altındaki mevcut ayarlara bağlı değildir.

**Uçtan uca aşamalar** (`sim_e2e_test.sh`):

| Aşama | Kapsam |
|---|---|
| A | Haritalama, kapı, watchdog, sınırlar, güvenlik durdurması, TF, sensörler, harita, hizalama, bag |
| B | Navigasyon, bilinmeyen yönden konumlandırma, engel ve sensör kesintisi senaryoları, duvara yaklaşma, engel etrafından dolaşma, hedef hareketi, global konumlandırma, tekrarlı konumlandırma |
| C | `test/data/sim_reference`'a karşı replay regresyonu |
| D | Odometri modeli ve düzeltme, kalibrasyon modu |

Container içinde tekil senaryolar `kelo_e2e <senaryo>` ile çalıştırılır: `odom`, `tour`, `localization`,
`obstacle`, `dropout`, `teleop_obstacle`, `teleop_dropout`, `near_wall`, `goal_motion`, `detour`, `relocalize`.

## İşletim ve bakım

**İlk bağlantı kontrol listesi**

1. [Açık konuları](#açık-konular) netleştirin ve ölçümleri `hardware.yaml`'a girin.
2. Sensör modunda (`KELO_MODE=sensors`) başlatın; `/scan`, derinlik, Marvelmind, `/odom` ve TF ağacını doğrulayın.
3. `ros2 topic info -v /cmd_vel` ile tek yayıncının `cmd_vel_gate` olduğunu doğrulayın.
4. Robot kaldırılmış veya açık alandayken kısa tuş basışlarıyla hareket yönlerini doğrulayın.
5. Tuşu bırakın ve robotun 0,5 s içinde durduğunu doğrulayın.
6. Donanımsal acil durdurmayı yazılımdan bağımsız olarak doğrulayın.
7. Bir engele yavaşça yaklaşın ve durma mesafesini ölçün.
8. Yavaş hareket sırasında kamera veya LiDAR bağlantısını kesin ve robotun durduğunu doğrulayın.
9. Odometri karakterizasyonunu yapın ve `odom_correction`'ı güncelleyin.
10. Haritayı oluşturun, Marvelmind'ı hizalayın, konumlandırın ve navigasyonu düşük hızda test edin.

**Loglar ve servis**

```bash
docker logs -f kelo-production          # döndürülen, 5 × 20 MB
systemctl status|restart|stop kelo
journalctl -u kelo -f
```

**Tanı kaydı**

```bash
docker compose exec production ros2 bag record -s mcap --storage-preset-profile zstd_fast \
  -o /data/bags/run --max-bag-duration 600 \
  /scan /odom /odom_corrected /tf /tf_static /cmd_vel /cmd_vel_safe /cmd_vel_nav /amcl_pose \
  /particle_cloud /marvelmind/pose /hedgehog_pos_ang /camera/camera/depth/image_rect_raw \
  /camera/camera/depth/camera_info /kelo/mm_state /kelo/mm_divergence_m /kelo/gate_state \
  /collision_monitor_state /diagnostics /local_costmap/costmap /plan
```

**Yeniden haritalama.** `mapping` profilini ve `save_map`'i çalıştırın. Önceki haritalar tarih ekiyle
saklanır ve `current.yaml` en yenisini gösterir. Geri dönmek için:
`ln -sfn <onceki>.yaml data/maps/current.yaml`. Yeniden haritalamadan sonra `mm_calibrate`'i tekrar
çalıştırın.

## Açık konular

Robot ekibiyle netleştirilecekler:

- `/cmd_vel` topic adı ve mesaj tipi (`Twist` veya `TwistStamped`).
- `/odom` frame'i ve frekansı, sürücü tarafı komut zaman aşımı, `ROS_DOMAIN_ID`.
- Sürücünün `odom → base_link` dönüşümünün kapatılıp kapatılamayacağı (`odom_mode: corrected` için
  gereklidir; aksi halde yaw düzeltmesi olmadan `odom_mode: driver` kullanılır).

Robot üzerinde doğrulanacaklar:

- Sensör montaj konumları (`hardware.yaml → mounts`, `base_link`'ten metre cinsinden). Şu anki
  değerler: kamera x 0,42 z 0,68, 10° aşağı eğik; LiDAR x 0,45 z 0,20 (tahmini); hedgehog x 0 z 0,70.
- Footprint 0,85 × 0,53 m; `nav2_params.yaml` (3 yer) ve `safety.yaml` (1 yer) içinde özdeş tutulmalıdır.
  Durma ve yavaşlama bölgeleri footprint'i kapsamalıdır (`test_config.py` ile doğrulanır). LiDAR ön
  kenardan yaklaşık 5 cm dışarı taşar.
- Marvelmind `hardware.yaml → sensors` içinde kapalıdır; hedgehog ve beacon'lar kurulunca açılmalıdır.
  Uçtan uca test aynı ayarı izler (`$E2E_DIR/data/hardware.yaml` ile değiştirilebilir): Marvelmind
  kapalıyken onun kontrolleri `SKIP` olarak raporlanır ve B aşaması konumu operatörün verdiği noktadan
  bulur (`localize --x --y`).

Faydalı komutlar: `ros2 topic info -v /cmd_vel`, `ros2 topic info -v /odom`,
`ros2 run tf2_ros tf2_monitor odom base_link`.

## Bilinen kısıtlar

- Kontrolcü ayarı, AMCL gürültü parametreleri ve durma mesafeleri fiziksel robot üzerinde
  doğrulanmamıştır.
- Sensör montaj konumları yaklaşıktır; LiDAR konumu tahminidir.
- Kamera 10° aşağı eğikken 6–20 cm boyundaki engeller ön kenarın yaklaşık 0,6–0,8 m yakınında iki
  sensörden de kaybolur (LiDAR düzleminin altında, kamera görüşünün dışında). Daha fazla eğim (25–30°)
  bu bölgeyi küçültür.
- Yanlar ve arka hiçbir sensör tarafından görülmez (önde LiDAR, öne bakan kamera). Bu nedenle Nav2
  `DiffDrive` hareket modelini kullanır (yalnızca ileri hareket ve dönüş, yana hareket yok); çok yönlü
  hareket robotta doğrulandıktan sonra `nav2_params.yaml` içinde `motion_model: "Omni"` yapılabilir.
- Marvelmind olmadan global konumlandırma simetrik bir ortamda benzer görünen başka bir yere oturabilir
  (simülatörde görüldü: karşı köşe, 180° dönük). **Localize** sonrasında konumu RViz'de kontrol edin veya
  konumu **Set Initial Pose** ile verin.
- Engelin etrafından dolaşmak yavaştır: simülatörde 2,5 m'lik bir sapma yaklaşık 40–60 s, engel durma
  bölgesinin içinde aniden belirdiğinde yaklaşık 75–85 s sürer (robot devam etmeden önce yerinde
  dönmelidir). Ani engel durumlarının yaklaşık üçte birinde robot engelin önünde 2,5 dakikadan uzun
  süre durup kalır (çarpışma olmaz).
- Odometri düzeltme tabloları 0,6–1,5 rad/s ve 0,06–0,15 m/s aralığını kapsar; bu aralık dışında uç
  değerler kullanılır.
- Derinlik kamerası ön köşelerin yanlarını (87° yatay görüş) görmez; LiDAR yalnızca kendi düzlemini
  kapsar.
- LiDAR odometrisi (rf2o) uzun veya özniteliksiz koridorlarda kayar; slam_toolbox döngü kapatma ile
  düzeltir.
- Tek hedgehog yön bilgisi vermez; başlangıç yönü dönüşle tahmin edilir veya elle girilir.
- Yazılımsal render (Mesa llvmpipe) ile RViz başlangıçta bir GLSL sampler doğrulama uyarısı loglar;
  harita doğru çizilir.
- Kabul testleri (tekrarlanabilirlik, durma mesafesi, kaçırılmış robot kurtarma, bir saatlik dayanıklılık)
  fiziksel robot üzerinde henüz yapılmamıştır.
