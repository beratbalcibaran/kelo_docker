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

   **Localize** (konum vermeden) hizalama dosyası varsa Marvelmind konumunu kullanır. Dosya yoksa önce
   kaydedilmiş son konumu dener, olmazsa yerinde dönerek tüm haritada arar; sonuç ancak LiDAR taraması
   oradaki harita duvarlarına uyuyorsa kabul edilir. Simetrik bir odada yine de **Set Initial Pose**
   gerekebilir.
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
| `docker/detector/Dockerfile` | Yalnızca Jetson: GPU'da nesne dedektörü (TensorRT, YOLOX-s), `detector` profili |
| `src/kelo_bt_nodes/` | Nav2 davranış ağacı düğümü `IsPersonBlocking` (C++): yoldaki insanı bekle |
| `scripts/test_all.sh` | İmajı derler ve tüm testleri çalıştırır |
| `scripts/sim_e2e_test.sh` | Simülatörde uçtan uca testler |
| `scripts/replay_regression.sh` | Bir kayıttan haritayı yeniden çıkarır ve referansla karşılaştırır |
| `scripts/camera_layer_eval.sh` | Simülatörde iki katmanlı harita değerlendirmesi (haritalar, kapsama, navigasyon hedefleri) |
| `docs/task1/` | Simülatörde iki katmanlı harita sonuçları (tablolar ve yan yana haritalar) |
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
| `config/sim_world_tables.yaml` | Masa, raf ve alçak eşyalı simülatör odası (iki katmanlı harita değerlendirmesi) |
| `launch/bringup.launch.py` | Her şeyi başlatan tek giriş noktası |
| `gui.py` | Kontrol paneli (butonlar, WASD ile sürüş) |
| `behavior_trees/navigate_wait_then_detour.xml` | Nav2 davranışı: yoldaki engel için dur ve bekle, sonra etrafından dolaş |
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
| `camera_mapper.py`, `camera_grid.py` | İki katmanlı harita: haritalama sırasında kamera engellerini ikinci bir harita katmanında biriktirir |
| `camera_layer_rebuild.py` | Bitmiş bir haritanın kamera katmanını haritalama bag'inden yeniden oluşturur (konumlar son haritayla uyumlu) |
| `odom_filter.py` | Robotun tekerlek odometrisini düzeltir |
| `status_node.py` | Genel durumu raporlar ve konumlandırma durumunu tutar |
| `localize.py` | Robotun haritadaki konumunu bulur (GUI: Localize) |
| `robot_check.py` | Robota ilk bağlantı kontrolü (topic'ler, tipler, frekanslar, TF sahipliği, saat farkı; robotu hareket ettirmez) |
| `loc_monitor.py`, `scan_match.py` | Konumlandırmayı haritaya göre denetler (tarama uyumu), `LOST` bildirir, son konumu kaydeder ve geri yükler |
| `send_goal.py` | Navigasyon hedefi gönderir (GUI: Send Goal) |
| `waypoint_tool.py` | İsimli konumları kaydeder ve aralarında sürer |
| `save_map.py` | Mevcut haritayı kaydeder (GUI: Save Map) |
| `marvelmind_bridge.py`, `mm_monitor.py`, `mm_recorder.py`, `mm_calibrate.py` | Marvelmind: konum dönüşümü, tutarlılık kontrolü, örnek kaydı, harita hizalaması |
| `odom_test.py` | Gerçek robotta odometri hatasını ölçer |
| `map_compare.py` | İki haritayı karşılaştırır |
| `map_side_view.py` | RViz: kamera katmanı LiDAR haritasının yanında (yalnızca görüntü) |
| `camera_view.py` | KELO Camera penceresi (renkli ve derinlik kamera görüntüsü tam çözünürlükte) ve KELO Surround penceresi (çevre görünümü / haritalama kapsaması); çift tık = oraya git |
| `parking_view.py` | Ön kamera görüntüsü: komut edilen harekete göre bükülen kılavuz çizgiler, işaretli engeller, zeminde Nav2 rotası, isimli nesneler, renkli ve derinlik (yalnızca görüntü) |
| `surround_view.py` | Tesla ekranı gibi çevre görünümü: robot 3B olarak duvarlar, nesneler, park asistanı mesafeleri ve rota ile (yalnızca görüntü) |
| `objects.py`, `detector.py` | LiDAR + kameradan robotun çevresindeki nesneler, renkli görüntüde YOLOX ile isimlendirilir, takip edilir; engel hafızası |
| `detector_service.py`, `trt_detector.py` | TCP servisi olarak dedektör; Jetson GPU'su için TensorRT arka ucu (kelo-detector container'ı) |
| `social.py` | İnsana duyarlı navigasyon: costmap'lerde kişisel alan ve tahmini yürüme yolu, insan yakınında hız limiti, yoldaki insan |
| `coverage_view.py` | Haritalama kapsaması: kamera kapsaması, yalnızca uzaktan görülen alanlar, sınırlar, sıradaki hedef (yalnızca görüntü) |
| `video_recorder.py` | Sürüş görünümlerinin mp4 kaydı ve dashcam klibi (`/data/videos`) |
| `web_view.py` | Telefon tarayıcısında sürüş görünümleri, durum, dokunarak hedef ve joystick (port 8080) |
| `scene.py`, `drive_inputs.py` | Sürüş görünümlerinin ortak geometrisi ve girdileri (hareket tahmini, rota şeridi, park asistanı sektörleri, kümeler, takip) |
| `layer_eval.py` | Simülatörde iki katmanlı harita değerlendirmesi (referans harita, nesne bazında kapsama, hayalet hücreler, yan yana harita görseli) |
| `wait_for_tf.py` | Açılışta gerekli koordinat sistemleri hazır olana kadar bekler |
| `logic.py` | Ortak hesaplamalar (birim testleriyle test edilir) |
| `sim_robot.py`, `sim/world.py` | Simülatör: robot, sensörler ve çarpışmalar |
| `e2e.py` | Uçtan uca test senaryoları |
| `sim_person.py` | Simülatör test yardımcısı: robotun önünde planlanan yola bir insan çıkar (GUI: Person on path) |

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

**RViz görünümü ve kamera penceresi.** RViz iki haritayı yan yana gösterir: robot, costmap'ler ve rota
ile LiDAR haritası ve sağında kamera katmanı (`/map_camera`, robot ok olarak). RViz'in altındaki
**KELO Camera** penceresi (`camera_view`) ön kamera görüntüsünü tam çözünürlükte gösterir: solda renkli,
sağda derinlik görüntüsü. Kontrol panelinin altındaki **KELO Surround** penceresi çevre görünümünü
gösterir; **Coverage** butonuyla haritalama kapsamasını gösterir. Nesneler ve sürüş görünümleri
(`objects`, `parking_view`, `surround_view`, haritalamada `coverage_view`) yığında çalışır ve yalnızca
biri izlerken çizer; panel RViz ile birlikte `map_side_view` ve `camera_view` pencerelerini başlatır. Sanal ekran 2240 × 1260'tır (16:9, `SCREEN_RES`): RViz sol
üstte, kamera penceresi onun altında, kontrol paneli sağ üstte, çevre penceresi onun altında. RViz
nesneleri de gösterir (`Objects`, kutular ve isimler).

```
+------------------------------+---------------+
| RViz: LiDAR map | camera map | control panel |
+------------------------------+---------------+
| KELO Camera: colour | depth  | KELO Surround |
+------------------------------+---------------+
```

**Kamera görüntüsü** (`parking_view`, arabalardaki park kamerası gibi). Zeminde robot genişliği artı
her yanda 0,10 m aralıkta, ön kenardan 2,5 m ileriye kadar kılavuz çizgiler; 0,5 m'de bir çubuk
(0,5 m'ye kadar kırmızı, 1,0 m'ye kadar sarı, ötesi yeşil). Çizgiler, arabada direksiyonu takip eden
çizgiler gibi komut edilen hızı (`/cmd_vel`) takip eder: robot mevcut hareketini sürdürürse ön
köşelerinin nereye gideceğini gösterir (ileri, yana, dönerek; en fazla 2 rad dönüş); dururken, yerinde
dönerken veya geri giderken düz ileriyi gösterir. 3B noktası (derinlik görüntüsünden) zeminden
0,06–1,20 m yüksekte olan pikseller işaretlenir: robotun çizgiler boyunca taradığı alanın içinde
kırmızı, yanında 0,4 m'ye kadar turuncu. Nav2 bir hedefe giderken rota zeminde hedefe doğru akan oklu
mavi bir şerit olarak çizilir; engellerin arkasından geçer (zeminin önünde engel olan pikseller
boyanmaz) ve hedef görüş alanındaysa kalan mesafeyle bir iğne işareti hedefi gösterir. Renkli
görüntüde dedektörün isimlendirdiği nesneler köşe çerçevesi, isim ve robota mesafeyle gösterilir
(`PERSON 1.5 m`); dedektör kaybettikten sonra LiDAR'ın hâlâ takip ettiği insanlar çerçevelerini korur
(3B kutuları görüntüye izdüşürülür). Alt satır ön kenardan çizgiler içindeki en yakın engele mesafeyi gösterir (yoksa `CLEAR`); bu
değer `/parking_view/nearest` üzerinde de yayınlanır. İki görüntü `/parking_view/color_image` ve
`/parking_view/image` olarak da vardır (RViz Image ekranları `ParkingViewColor`, `ParkingViewDepth`,
varsayılan kapalı). Görüntü yalnızca operatör içindir; durma ve kaçınma costmap'ler ve collision monitor
ile yapılır. Kamera zemini ön kenardan yaklaşık 0,9 m ileriden itibaren görür (10° eğim), ilk çubuklar
görüntünün altında kalır. Renkli akış (`hardware.yaml → camera.color`, `color_profile`) yalnızca bu
görüntü için kullanılır; Jetson'da USB bant genişliği gerekirse `color: false` yapın.
`camera.color_offset_y` renkli sensörün `camera_link` içindeki konumudur (D435: derinlik orijininin
15 mm yanında); işaretini robotta kameranın derinlik-renk dış parametreleriyle
(`/camera/camera/extrinsics/depth_to_color`) kontrol edin.

**Çevre görünümü** (`surround_view`, Tesla'nın sürüş ekranı gibi; `view:=top` tam yukarıdan bakar).
Robot arkadan ve yukarıdan 3B olarak: harita duvarları bloklar halinde, haritaya sabit zemin ızgarası,
LiDAR noktaları (0,3 / 0,6 m'den yakınsa kırmızı / turuncu), zemine yatırılmış kamera engel noktaları,
insanlar ayakta figür, diğer nesneler mesafeleriyle turuncu kutu (hareket ediyorsa okla), komut edilen
hızın kılavuz çizgileri ve navigasyon sırasında akan oklu mavi yol olarak rota ve hedef iğnesi.
**Park asistanı**: taban izinin çevresinde 24 yönde, 1,2 m içindeki en yakın engel kendi yerinde bir yay
olarak (0,3 m altı kırmızı, 0,6 m altı turuncu, 1,0 m altı sarı, ötesi yeşil) ve en yakın mesafe cm
olarak. LiDAR yalnızca önde ±92° görür: arkasında harita duvarları (0,8 m içinde) kullanılır ve yay
kesikli çizilir, çünkü orada canlı sensör yoktur. Üst çubuk hızı, hedefe mesafeyi ve en yakın engeli
gösterir. OpenCV ile çizilir (960 × 720, 10 Hz), GPU gerekmez.

**Nesneler** (`objects`). LiDAR taraması ve kamera engel bulutu, harita duvarlarındaki noktalar
olmadan (0,25 m büyütülmüş), 0,1 m'lik ızgarada gruplanır; 2 m'ye kadar olan gruplar nesne olur, bir
harita duvarına değen isimsiz gruplar atlanır (duvar uçları). YOLOX-tiny (COCO, resmi ONNX çıktısı,
Dockerfile tarafından `/opt/kelo/models` içine indirilir ve kontrol edilir) renkli görüntüde OpenCV DNN
ile CPU'da çalışır (`hardware.yaml → perception`, 3 Hz, bu VM'de görüntü başına ~50 ms); tespit
kutusunun içindeki kamera noktaları konumunu verir ve en yakın grup ismi alır. Odom çerçevesindeki
takipçi nesneleri yumuşatır ve kamera kaybettikten sonra ismi 5 s tutar. Çıktılar: `/kelo/objects`
(`vision_msgs/Detection3DArray`, odom çerçevesi), `/kelo/detections` (`vision_msgs/Detection2DArray`,
renkli görüntü pikselleri) ve `/kelo/objects/markers` (RViz). Simülatörde insanlar `/sim/people`'dan
gelir (basit renkli çizimde YOLOX'un tanıyacağı bir şey yoktur). İnsanları insana duyarlı navigasyon
kullanır (aşağıda); diğer isimler yalnızca görüntü içindir. Model dosyası yoksa veya
`perception.detector: false` ise nesneler isimsiz kalır. Dedektör servisi cevap verdiğinde
(`perception.service`; Jetson'da `detector` profili: GPU'da TensorRT ile YOLOX-s, bkz.
[Jetson kurulumu](#jetson-kurulumu)) tespit orada çalışır; vermezse burada CPU'da, servis 10 s'de bir
yeniden denenir.

**İnsana duyarlı navigasyon** (`social`, navigasyon modu, `hardware.yaml → navigation.people_aware`).
İnsan kutudan farklı ele alınır:

| | İnsan | Kutu / diğer engel |
|---|---|---|
| Bırakılan mesafe | kişisel alan: insanın 0,55 m çevresi yüksek maliyet (geçilmez değil), 1,2 m'ye kadar azalan maliyet: yer varsa rota geniş, dar geçitte yakından geçer; geçilmez olan yalnızca LiDAR ve kameranın gördüğü | taban izi + 0,70 m şişirme |
| Hız | taban izine 1,0 m içinde %40, 2,0 m'de tekrar %100 (`/speed_limit`); collision monitor yavaşlatırken üstüne eklenmez | değişmez (collision monitor yavaşlama bölgesi) |
| Önündeki yolda | robot durur ve insanın kenara çekilmesini **bekler** (0,5 s'de bir bakar, en fazla 10 s), sonra düz devam eder; yalnızca kalan bir insanın kişisel alanı korunarak yanından geçilir | 5 s bekler, sonra **etrafından dolaşır** |
| Yürüyen insan | önümüzdeki 3 s'de yürüyeceği yer costmap'lerdedir (1,5 s'ye kadar geçilmez): robot onun yoluna plan yapmaz, karşıdan geçerken yol verir | – |

Social düğümü `/kelo/objects` içindeki insanları (hızlarıyla) alır ve şunları yayınlar:
`/kelo/people_map` (`/map` ile aynı geometri; global costmap'in `people_layer` StaticLayer'ı okur,
böylece planlayıcı insanların yanından geniş geçer; yerel costmap insanı LiDAR ve kameradan alır:
oradaki kademeli maliyetler kontrolcüyü insanın yanında sürünme hızına düşürüyordu),
`/speed_limit`, `/kelo/person_blocking` (planın önümüzdeki 2,5 m'sinin 0,8 m yakınında bir insan veya
yürüme yolu) ve `/kelo/people/markers` (RViz: halkalar ve tahmini yollar). Davranış ağacı
`/kelo/person_blocking`'i `IsPersonBlocking` düğümüyle (`src/kelo_bt_nodes`) okur. İnsan katmanları
yalnızca maliyet ekler: dedektörün kaçırdığı bir insan LiDAR ve kamera katmanlarında yine engeldir ve
kutu gibi ele alınır.

**Engel hafızası.** Nesne düğümü LiDAR ve kamera engel noktalarını sensörler görmeyi bıraktıktan sonra
da odom çerçevesinde tutar (`/kelo/obstacle_memory`, 5 cm hücreler, alanlar x y z age). Bir hücre, bir
sensör yeniden içinden gördüğünde (daha uzakta biten bir LiDAR ışını; LiDAR'ın üstünden geçtiği alçak
nesneler için kamera), 30 s sonra veya 6 m'den uzaklaşınca unutulur. Çevre görünümü bunları 30 s'de
solan gri noktalar olarak çizer ve park asistanı LiDAR'ın bakmadığı arka tarafta bunları (kesikli)
kullanır. Yalnızca görüntü; costmap'ler kendi engellerini tutar.

**Tıklayarak sürüş.** Navigasyon modunda çevre görünümünde veya renkli ya da derinlik görüntüsünde
zemine çift tık robotu oraya gönderir (Nav2 hedefi; yönü robottan dışarı doğru, en fazla 15 m); sağ tık
hedefi iptal eder. Görüntüyü çizen görünüm pikseli zemin noktasına çevirir (`/kelo/view_click`),
görünümde kısa bir not ne olduğunu söyler.

**Haritalama kapsaması** (`coverage_view`, haritalama modu, KELO Surround → Coverage). Oluşan haritanın
üstten görünüşü: yeşil = LiDAR ve kameranın gördüğü boş zemin, turuncu-sarı = yalnızca LiDAR (alçak
nesneler kamera katmanında eksik olabilir), gri-mavi = yalnızca 3 m'den uzaktan görülmüş, turuncu =
keşfedilmemiş alana sınırlar, en yakını mesafesiyle sıradaki hedef olarak işaretli, ve robotun izi.
Üst satırlar haritalanan alanı ve oranları verir. Sınır kalmayana ve sarı az olana kadar haritalayın.

**Video** (`video_recorder`). Paneldeki (veya web görünümündeki) Record / Stop Recording, çevre
görünümünü ve renkli kamerayı yan yana `/data/videos/kelo_<tarih>_<saat>.mp4` dosyasına yazar (10 fps,
varsa H.264; 30 dk sonra veya 1 GB'tan az boş yer kalınca kendiliğinden durur).
`recording.dashcam_s: 60` ile son dakika bellekte tutulur ve **Save Clip** onu bir dosyaya yazar, bir
olaydan sonra araç kamerası gibi (CPU harcar: görünümler sürekli çizilir).

**Web görünümü** (`web_view`), telefon veya tablet için: `http://<robot>:8080` (GUI container'ı:
`http://localhost:8080`), kullanıcı `kelo`, şifre `KELO_WEB_PASSWORD` (ayarlı değilse logda rastgele
bir şifre yazılır). Canlı görünümler (Surround, Camera, Depth, Coverage; yalnızca izlenirken çizilir),
hız, hedef mesafesi, insan ve kayıt durumu; zemine çift dokunuş = oraya git, Cancel goal, Record, Save
clip ve basılı tutarak sürülen joystick (`web.allow_drive`): klavye gibi teleop collision monitor ve
kapıdan geçer, parmak bırakılınca veya bağlantı kopunca (0,3 s) durur. Şifresiz HTTP: yalnızca robotun
kendi ağında kullanın.

**Person on path (sim)** (varsayılan işaretli, yalnızca simülatör): Start Navigation ile `sim_person`
de başlar. Her hedefte robot hareket edince planlanan yolun yaklaşık 1,8 m önüne bir insan çıkar; robot
geçince, hedef bitince veya 60 s sonra kaldırılır. Diğer modlar:
`ros2 run kelo_bringup sim_person --ros-args -p mode:=cross` (yolun karşısına bir kez yürür) veya
`-p mode:=block` (odayı kapatan duvar: robot beklemek zorunda).

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
| `config/sim_world_tables.yaml` | İki katmanlı harita değerlendirmesi için simülatör dünyası |

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

**GPU'da nesne tespiti (isteğe bağlı).** `detector` profili YOLOX-s'i TensorRT (FP16) ile kendi
container'ında çalıştırır (`docker/detector/Dockerfile`, NVIDIA L4T taban imajı, ROS yok); nesne düğümü
renkli görüntüleri `127.0.0.1:5810` üzerinden gönderir, servis cevap vermezken CPU'da YOLOX-tiny'ye
döner:

```bash
cat /etc/nv_tegra_release        # R36.4 -> varsayılan taban imaj; diğerleri: docker/detector/Dockerfile
docker compose --profile detector build      # başka JetPack için --build-arg L4T_IMAGE=...
docker compose --profile detector up -d      # ilk açılış motoru derler (birkaç dakika, /data/models)
docker logs kelo-detector                     # "[detector] tensorrt ... listening on 127.0.0.1:5810"
```

Nesne düğümü `object detection: using the detector service (tensorrt)` yazar. Jetson'da henüz test
edilmedi (geliştirme VM'inde GPU yok): protokol ve CPU'ya dönüş burada test edildi, TensorRT kısmı ancak
robotta test edilebilir.

DDS ara katmanı Cyclone DDS'tir (`docker/cyclonedds.xml`). Donanım profilleri (`production`, `mapping`,
`gui-hardware`) DDS'i `KELO_DDS_IFACE` arayüzüne (varsayılan `enP8p1s0`) bağlar; arayüz kapalıysa
logda uyarı yazar ve otomatik seçime döner. Jetson robot Ethernet'inde `192.168.1.10/24` sabit IP
kullanır ve chrony ile robota saat sunar (`allow 192.168.1.0/24`, `local stratum 10`). Seri cihaz yolları
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

#### İki katmanlı harita (LiDAR + kamera)

LiDAR yerden yaklaşık 20 cm yükseklikte tek bir düzlem görür. Bu yüzden masalar LiDAR haritasında
yalnızca ayaklarıyla görünür; raflar, çıkıntılar ve bu düzlemin altındaki nesneler haritada yoktur.
Kamera katmanı açıkken D435'in haritalama sırasında gördükleri ikinci bir harita katmanı olarak
kaydedilir ve global planlayıcı tarafından kullanılır.

- **Konum yalnızca LiDAR'dan gelir.** slam_toolbox değişmedi. Kamera noktaları tarama eşleştirmeye
  girmez.
- **Biriktirme.** `camera_mapper` yalnızca haritalama modunda çalışır. `cloud_filter`'ın yükseklik
  filtreli engel noktalarını (`/camera/points_filtered`) ve temizleme ışınlarını
  (`/camera/points_map_clearing`) görüntü anındaki slam_toolbox konumuyla `map` çerçevesine taşır.
  Ardından bunları log-odds voksel sütunlarında biriktirir: `cloud_filter` `min_height` ile
  `max_height` (0,06–1,20 m) arasında 0,10 m'lik yükseklik dilimleri. Izgara LiDAR haritasıyla
  aynıdır: çözünürlük, başlangıç noktası ve boyut `/map`'i izler. Bir hücrede herhangi bir dilim doluysa
  hücre engeldir; böylece masanın altındaki zemin boş görülse de masa üstü engel olarak kalır.
  Temizleme ışınları geçtikleri dilimleri azaltır; ancak her voksel kendisine isabet eden en yüksek
  noktayı hatırlar ve vokselin içinden bu noktanın üstünde geçen bir ışın onu azaltmaz. Aksi halde alçak
  bir nesnenin üstünü sıyırıp arkadaki zemine giden ışınlar nesneyi silerdi (simülatörde 12 cm'lik bir
  kutu haritada büyük ölçüde boş görünüyordu ve robot ona çarpıyordu). Önünden geçen bir insan ise, alan
  yeniden boş görüldüğünde insanın yüksekliğinden geçen ışınlarla yine silinir. Temizleme ışınları, geçerli derinlik piksellerinin 4 m'ye kısaltılmış halidir
  (`map_clearing_range`); böylece 2,5 m'lik işaretleme menzilinin ötesindeki boş alan da hesaba katılır.
  Tek bir gürültülü ölçüm hücreyi engel yapmaz, yaklaşık iki tutarlı ölçüm gerekir. İşleme hızı saniyede
  5 karedir. Katman canlı olarak `/map_camera` konusunda RViz için yayınlanır.
- **Kaydetme.** `save_map <isim>`, LiDAR haritasıyla tam olarak aynı boyut ve başlangıç noktasına
  kırpılmış veya genişletilmiş `<isim>_<tarih>_camera.yaml/.pgm` dosyalarını da yazar.
- **Navigasyon.** Haritanın yanında `<harita>_camera.yaml` varsa ikinci bir `map_server`
  (`map_server_camera`) onu `/map_camera` konusunda yayınlar. `current.yaml` gibi sembolik bağlar
  çözülür. Global costmap bu katmanı `camera_static_layer` olarak yükler: `static_layer`'ın hemen
  arkasında ikinci bir `StaticLayer`. Katman yalnızca engel ekleyebilir: boş hücreleri hiçbir LiDAR
  duvarını silmez, bilinmeyen hücreleri LiDAR haritasını değiştirmez. Nav2 Jazzy'nin `StaticLayer`'ı
  `use_maximum` (ve `trinary_costmap`) ayarını katmandan değil costmap'ten okur; bu yüzden
  `use_maximum: true` iki costmap'te de ayarlıdır. Katman bazındaki ayar yok sayılır ve katman LiDAR
  haritasını ezer (22.10.'da düzeltildi: önceden kamera haritasında olmayan LiDAR duvarları global
  costmap'ten kayboluyordu). Kamera haritaları `free_thresh: 0.196` ile kaydedilir, böylece bilinmeyen
  pikselleri (205) bilinmeyen olarak yüklenir. 22.10.'dan önce kaydedilen kamera haritalarında `0.25`
  vardır (bilinmeyen boş yüklenir): `_camera.yaml` dosyalarındaki bu satırı değiştirin. Yerel
  costmap aynı katmanı eklenti listesinin başında yükler; böylece kontrolcü, kameranın kör bölgesindeki
  ya da robotun yanında ve arkasındaki (kameranın bakmadığı) haritalanmış alçak nesneleri unutmaz (bu
  katman olmadan robot simülatörde 12 cm'lik bir kutunun yanında dönerken ona çarptı). Bu katman AMCL
  konumunu izler: `localize`, konum henüz bilinmezken yerinde dönüşler boyunca onu kapatır ve sonra
  yeniden açar; collision monitor ve canlı sensör katmanları açık kalır. AMCL önceden olduğu gibi
  yalnızca LiDAR haritasını kullanır.
- **Canlı kamera engelleri.** İki costmap'teki canlı `camera_layer` bir `VoxelLayer`'dır (1,28 m'ye kadar
  0,08 m'lik vokseller). 2D `ObstacleLayer` kullanıldığında, alçak bir nesnenin üstünden arkadaki zemine
  giden temizleme ışını, nesne kameranın kör bölgesine girer girmez onu siliyordu ve robot nesneye
  çarpıyordu.
- **Anahtar.** `hardware.yaml → mapping.camera_layer` (varsayılan `true`). `camera_layer: false`,
  `sensors.camera: false` ya da `_camera.yaml` dosyası yoksa sistem bu özellik hiç yokmuş gibi başlar:
  `camera_mapper`, ikinci harita sunucusu ve ek costmap katmanı çalışmaz.
- **Düzenleme.** Geçici nesneleri kaldırmak için LiDAR PGM'ini düzenlerseniz `<isim>_camera.pgm`'i de
  aynı şekilde düzenleyin. `_camera.*` dosyalarını silmek o harita için katmanı kapatır.

- **Haritalamadan sonra yeniden oluşturma (önerilen).** Haritalama sırasında hücreler görüldükleri
  andaki slam_toolbox konumuyla yerleştirilir. Sonradan gelen bir döngü kapatma LiDAR haritasını
  düzeltir, ancak daha önce biriktirilmiş kamera hücrelerini düzeltmez. `save_map`'ten sonra katmanı
  haritalama bag'inden yeniden oluşturun:

  ```bash
  docker compose exec mapping ros2 run kelo_bringup camera_layer_rebuild /data/bags/map_hardware_<tarih>
  ```

  Bag, bitmiş LiDAR haritası (`current.yaml` ya da ikinci argüman olarak verilen harita) üzerinde
  oynatılır ve `<isim>_camera.yaml/.pgm`, son haritayla uyumlu konumlarla yeniden yazılır. Robotun
  konumunu, kaydedilmiş pose graph üzerinde (`<isim>.posegraph`, haritalamadaki tarama eşleştiricisiyle
  aynı) slam_toolbox bulur. Pose graph yoksa AMCL kullanılır (`--localizer amcl` bunu zorlar;
  simülatörde slam_toolbox'tan daha az hassastır ve daha kötü bir katman verir, aşağıdaki sonuçlara
  bakın).
  Haritalama sırasındaki katman `<isim>_camera_live.yaml/.pgm` olarak saklanır. Başlangıç konumu
  bag'deki TF'ten okunur. Oynatma kendi ROS alanında çalışır (`--domain`, varsayılan 99, yalnızca
  localhost), bu yüzden çalışan bir sistemin yanında çalıştırılabilir; `--rate` oynatmayı hızlandırır.
  Aynı oynatma `mode:=camera_layer source:=replay` olarak da kullanılabilir.

- **Değerlendirme.** `scripts/camera_layer_eval.sh` karşılaştırmanın tamamını simülatörde,
  `config/sim_world_tables.yaml` içinde çalıştırır (üç masa, ayaklı bir raf, 0.12 ve
  0.15 m yüksekliğinde iki kutu bulunan 8 × 6 m'lik bir oda; haritalama sırasında yanından bir kişi
  geçer). Tek bir haritalama sürüşü kaydedilir ve aynı bag iki kez haritalanır: `camera_layer: false`
  ve `true` ile. Ardından kamera katmanı bag'den yeniden oluşturulur. Her harita, dünyadan çizilen bir
  referansla karşılaştırılır (`map_compare`, `layer_eval`) ve dünyadaki 10 navigasyon hedefi her
  haritada sürülür. `--bag <dizin>` kaydedilmiş bir bag'i yeniden kullanır, `--skip-nav` haritalardan
  sonra durur. Sonuçlar `data/eval/task1_<tarih>/results.md` dosyasına yazılır.

##### Sonuçlar (simülatör)

Tabloların tamamı: [docs/task1/sim_results.md](docs/task1/sim_results.md). Siyah: LiDAR haritası,
turuncu: kamera katmanının eklediği hücreler.

![Referans, yalnızca LiDAR, LiDAR + kamera katmanı (haritalama sırasında), LiDAR + kamera katmanı (yeniden oluşturulmuş)](docs/task1/sim_maps.png)

| harita | precision | recall | F1 | hayalet hücre |
|---|---|---|---|---|
| Yalnızca LiDAR | 0.982 | 0.681 | 0.804 | 16 |
| LiDAR + kamera katmanı (haritalama sırasında) | 0.841 | 0.909 | 0.874 | 48 |
| LiDAR + kamera katmanı (yeniden oluşturulmuş) | 0.862 | 0.904 | **0.882** | 17 |

| nesne (dış hattının haritada bulunan payı) | yalnızca LiDAR | kamera katmanı (haritalama sırasında) | kamera katmanı (yeniden oluşturulmuş) |
|---|---|---|---|
| masalar (üst yüzey 0.60 m) | 0.25 | 0.96–1.00 | 0.95–0.99 |
| table_c | 0.26 | 0.53 | 0.51 |
| raf (en alt raf 0.30 m) | 0.21 | 0.61 | 0.57 |
| alçak kutular (0.12 m, 0.15 m) | 0.00 | 1.00 | 1.00 |

Navigasyon: dünyadaki 10 hedef her haritada, iki koşu (her koşu için ulaşılan hedef ve çarpışma):

| harita | ulaşılan | çarpışma | collision monitor durması | yeniden rota |
|---|---|---|---|---|
| Yalnızca LiDAR | 5/10, 5/10 | 33, 42 | 19, 0 | 7, 1 |
| LiDAR + kamera katmanı (haritalama sırasında) | 6/10, 6/10 | **0, 0** | 5, 2 | 1, 0 |
| LiDAR + kamera katmanı (yeniden oluşturulmuş) | 8/10, 6/10 | **0, 3** | 21, 1 | 0, 0 |
| karşılaştırma, kamera katmanı yalnızca global costmap'te (haritalama sırasında, yeniden oluşturulmuş; tek koşu) | 6/10, 6/10 | 12, 14 | 3, 3 | 0, 0 |

Sayıların yorumu:

- LiDAR haritasında neredeyse hiç yanlış hücre yoktur (precision 0.98), ancak masaların, rafın ve
  alçak kutuların çoğu eksiktir (recall 0.68). Kamera katmanı bunları ekler (recall 0.90–0.91).
- Precision düşer, çünkü referansta her nesnenin yalnızca dış hattı vardır; kamera ise masa üstünün
  ya da kutunun içini de doldurur ve `map_compare` dolu her iç hücreyi yanlış sayar. Yanlış engeller
  için daha iyi ölçü hayalet hücrelerdir (her nesneden 0.10 m'den uzak dolu hücreler): yeniden
  oluşturulmuş haritada 17, LiDAR haritasında 16. Haritalama sırasındaki katmanda 48 tane vardır;
  çoğu, döngü kapatma LiDAR haritasını kaydırmadan önce yerleştirilmiş duvar boyu hücrelerdir.
  Yeniden oluşturma bunları giderir.
- Yalnızca LiDAR haritasıyla planlayıcı rotayı alçak kutuların üstünden ve masa kenarlarının altından
  geçirir. Robot bunları canlı kamerayla geç görür, kör bölgede de çarpar (33 ve 42 çarpışma). Kamera
  katmanıyla etraflarından dolaşır.
- Kamera katmanı yerel costmap'te de gereklidir: yalnızca global costmap'teyken (karşılaştırma satırı)
  robot, alçak bir kutunun yanında dönerken kutu kameranın kör bölgesine girdiğinde ona çarptı.
- İkinci koşuda yeniden oluşturulmuş haritadaki 3 temas `table_c`'nin güney kenarıyladır. Haritalama
  rotası bu kenara hiç dönük geçmedi, bu yüzden kenar katmanda yoktur (kapsama 0.51). Katman yalnızca
  kameranın baktığı yerleri içerir: haritalama sırasında her masanın ve alçak nesnenin önünden ona
  dönük geçin.
- 4. ve 7. hedefler iki koşuda da her haritada başarısız oldu. İkisi de alçak bir kutu ile bir masa
  arasındaki yaklaşık 1 m'lik boşluklardadır. Son metrede collision monitor robotu yavaşlatır,
  kontrolcü 150 s sınırına kadar *failed to make progress* bildirir; bu bir harita farkı değildir.
  2. hedefte (`desk_b` ile kuzey duvarı arasında) ikinci koşuda her haritada süre doldu; kamera
  haritalarında çarpışma olmadı. İlk koşuda yeniden oluşturulmuş haritadaki 21 collision monitor
  durmasının hepsi 7. hedeftedir.
- Yeniden oluşturmada slam_toolbox yerine AMCL daha kötü bir katman verdi (ilk bir koşuda F1 0.830'a
  karşı 0.901). Bu yüzden pose graph varsa yeniden oluşturma slam_toolbox ile yapılır.
- Simülatörde her yapılandırma için iki koşu: ulaşılan hedef sayısı koşudan koşuya bir iki değişir,
  bu yüzden onda, sürede ya da durma sayısındaki küçük farklar anlamlı değildir. Kapsama kazancı
  geliştirme sırasındaki her koşuda aynıydı. Çarpışma sonucu, ızgaradaki en yüksek isabet kuralı,
  `VoxelLayer` ve yerel costmap'teki katmandan beri geçerlidir: bunlardan önce temizleme ışınları alçak
  kutuları siliyordu ve kamera haritalarında da çarpışma oluyordu.

##### KELO üzerinde değerlendirme

Simülatör robot üzerindeki testin yerini tutmaz. Değerlendirmeyi gerçek bir odada tekrarlamak için:

1. Robot: `hardware.yaml` içinde `robot.odom_mode: driver` (oradaki düzeltme tablosu robotta
   ölçülmedi), `mapping.camera_layer: true`. Haritalama sürüşünden önce KELO PC ile Jetson'ın saatleri
   chrony ile senkronize edilmeli (`chronyc tracking`, sapma birkaç ms'nin altında).
2. Oda: masalar, bir raf ve 20 cm'den alçak en az iki nesne. Konumlarını şerit metreyle ölçüp bir
   referans harita çizin (aynı çözünürlük, `map` çerçevesi = başlangıç konumu) ya da LiDAR PGM'sinin
   bir kopyasını elle düzenleyip LiDAR'ın görmediği nesnelerin dış hatlarını ekleyin.
3. Haritalama: `docker compose --profile mapping up` ile tek sürüş, `save_map oda`, ardından
   `camera_layer_rebuild /data/bags/map_hardware_<tarih>`.
4. Aynı bag'den kamera katmanı olmadan ikinci harita: bag'i `camera_layer: false` içeren bir
   `hardware.yaml`'ın yanına kopyalayın ve değerlendirme betiğindeki gibi
   `source:=replay mode:=mapping` ile oynatın.
5. İki harita için `map_compare <harita>.yaml reference.yaml [--camera]` ve
   `python3 -m kelo_bringup.layer_eval figure maps.png reference.yaml ...`.
6. Navigasyon: her haritada aynı 10 hedef, `send_goal` ya da RViz ile; ulaşılan hedefler, yeniden
   rotalar, collision monitor durmaları ve temaslar sayılır (gözetmen her teması not eder).

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
`/kelo/localization_state` üzerinde tutulur. `send_goal`, `waypoint_tool` ve görünümlerde tıklanan
hedefler, durum `LOCALIZING`, `FAILED` veya `LOST` iken hedef göndermez; RViz'den gönderilen hedefler bu
kontrole tabi değildir.

**Tarama uyumu.** AMCL kovaryansı yalnızca parçacıkların birbiriyle uyuştuğunu söyler; benzer köşeleri
olan bir odada yanlış köşede de uyuşabilirler. `scan_match.py`, AMCL pozunda LiDAR taramasının haritaya
ne kadar uyduğunu ölçer: harita duvarı üzerine düşen tarama noktalarının oranı eksi haritadaki bir
duvarın içinden geçen ışınların oranı (insanlar ve yeni nesneler ışınları yalnızca kısaltır, uyumu biraz
düşürür). `localize` bir pozu ancak uyum en az %60 ise kabul eder (parçacık bulutu biraz genişse, xy <
0,25 m ve yön < 12°, en az %75); AMCL taramanın uymadığı bir yere oturursa global arama yeniden başlar (en
fazla 3 tur). Uyum `/kelo/localization_fit` üzerinde yayınlanır.

**Kayıp (LOST).** `loc_monitor` uyumu saniyede bir denetler. Robot `LOCALIZED` iken uyum 8 s boyunca
%35'in altında kalırsa (robot itildi veya taşındı ya da AMCL kaydı) durum `LOST` olur, çalışan hedef iptal
edilir ve görünümler `LOCALIZATION LOST - PRESS LOCALIZE` gösterir.

**Son konum.** Robot iyi uyumla `LOCALIZED` iken `loc_monitor` pozu (ait olduğu haritayla birlikte)
`/data/last_pose.yaml` dosyasına kaydeder. Bir sonraki açılışta (Marvelmind yok, `KELO_INITIAL_POSE` yok)
bu poz AMCL'e verilir ve yalnızca tarama orada uyuyorsa tutulur: durduğu yerde kapatılan robot dönmeden
yeniden `LOCALIZED` olur. Arada yeri değiştiyse log bunu yazar ve **Localize** gerekir. Konumsuz
**Localize** de global aramadan önce son konumu dener (`--force` bunu atlar).

**Konumlandırma sırasında dönüş.** Local costmap'in haritaya bağlı katmanları (haritanın kamera katmanı)
AMCL pozunu izler; poz henüz yanlışken hücreleri robotun altına düşüp dönüşü durdurabilir (`Collision
Ahead`). `localize` bunları Nav2 başladıktan sonra (Nav2 ancak AMCL'in bir pozu olunca başlar) ilk
dönüşten hemen önce kapatır, sonunda yeniden açar. Yine de erken duran bir dönüş log'a yazılır.

İkinci bir hedgehog (Marvelmind eşli işaretçiler, ≥ 0,3 m aralıklı) yönü doğrudan sağlar ve
konumlandırma dönüşüne gerek bırakmaz.

**Yoldaki engel: bekle, sonra etrafından dolaş** (`behavior_trees/navigate_wait_then_detour.xml`).

- Yol her hedef için bir kez planlanır ve boş olduğu sürece korunur.
- Saniyede iki kez yolun sonraki 2 m'si global costmap'e karşı kontrol edilir (hedeften önceki son
  0,5 m kontrol edilmez; duvar veya masa yanındaki hedefler de çalışır). Üzerinde yeni bir şey durursa
  yol takibi durdurulur ve robot durur.
- Robot 5 s bekler ve yeniden planlar. Engel gittiyse yeni yol doğrudan yoldur; hâlâ oradaysa yeni yol
  etrafından dolaşır. Robot yeni yolu takip etmeden önce yerinde ona doğru döner (MPPI'nin önünde
  `RotationShimController`, yol yönü 0,5 rad'dan fazla farklıysa).
- Dolaşacak yer yoksa (geçit kapalı) planlama başarısız olur ve robot yol açılana, hedef değiştirilene
  veya iptal edilene kadar 5'er saniye beklemeye devam eder.
- Kameranın robot önüne yakın kör kamaları (görüntünün altında LiDAR düzlemine kadar ve görüntünün
  üstünde engel yükseklik sınırına kadar), LiDAR ışınının o yönde serbestçe geçtiği yerlerde temizlenir
  (`cloud_filter`, `blind_clearing`): robotun hemen önünde dururken giden bir insan veya kutu artık
  costmap'te kalmaz. LiDAR düzleminden alçak nesneler hücrelerini korur; düzlemin üstüne uzanan gerçek
  bir nesne, kameranın gördüğü kısmından yeniden işaretlenir. 2,5 m'den uzağa giden derinlik ışınları da
  2,5 m'ye kadar temizler (önceden atılıyordu; uzak bir duvarın önünde yürüyen kişi iz bırakıyordu).
- Her ikinci bekleme her iki costmap'i de temizler ve sensörlere gerçekte orada olanı işaretlemeleri
  için 2 s verir. Bu olmadan, kameranın kör bölgesinde (kameraya 0,3 m'den yakın) dururken giden bir
  insan veya kameranın aşağı bakan ışınlarının hiç kesmediği bir insanın üst kısmı costmap'te kalır ve
  robotu sonsuza kadar bekletir. Haritalanmış alçak nesneler kalır (haritanın kamera katmanı); LiDAR
  düzleminden alçak, haritada olmayan ve o anda kamera kör bölgesinde bulunan bir nesne unutulur.
- Yolun kontrol edilen kısmında durmadan robotun önünden geçen bir insan, önceden olduğu gibi collision
  monitor (durma bölgesi) tarafından ele alınır.

Simülatör sonuçları (varsayılan dünya, `sim_person`, `3.0 1.0`'dan koridor hedefi `5.9 1.0`, insan
1,3 m önde; süreler `3.0 1.0`'dan itibaren tüm sürüştür):

| durum | sonuç | sürüş |
|---|---|---|
| insan yolda duruyor | 1,3 m önce durur, 5 s bekler, döner, etrafından dolaşır (en fazla 1 m yana) | 51 s |
| insan yolun karşısına bir kez yürüyor | durma bölgesi durdurur, insan gidince doğrudan yoldan devam eder | 62 s |
| insan bekleme sırasında gidiyor | devam eder; ilk yeni yol yine insanın durduğu yerden kaçınır (üst kısmı costmap'te) | 40 s |
| odayı kapatan duvar (dolaşacak yer yok), 30 s sonra kaldırılır | bekler, sonra devam eder | 81 s |

Hiçbir durumda çarpışma yok. Bu hedefe son metre her durumda yavaştır: doğu duvarına 1,1 m uzaklıkta,
yavaşlama bölgesinin içindedir.

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
| Engel kalktı | Temizleme bulutu ile costmap'ten silinir (LiDAR'ın boş gördüğü kamera kör kamasında da); navigasyon devam eder |
| LiDAR taraması 8 s boyunca AMCL pozunda haritaya uymuyor | Durum `LOST`, çalışan hedef iptal, **Localize** yapılana kadar yeni hedef yok |
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
- `camera.color` açıkken kamera görüntüsü için basit bir renkli görüntü (damalı zemin, her nesne bir
  renk, insanlar mavi).
- Gürültü, sıçrama ve kesinti içeren Marvelmind konumları.
- İvme sınırlı ve 0,5 s komut zaman aşımlı çok yönlü robot.
- Çarpışma algılama.
- Ölçülmüş KELO odometri hata modeli (varsayılan) veya ideal odometri (`sim_odom_model:=ideal`).

Test arayüzleri: `/sim/ground_truth`, `/sim/contacts`, `/sim/obstacles` ve
`ros2 param set /sim_robot camera_enabled false`. Bu topic'ler yalnızca testlerde ve `sim_person`
(GUI: Person on path) tarafından kullanılır.

## Test

```bash
./scripts/test_all.sh                 # imaj derleme, birim ve uçtan uca testler (45–60 dk)
./scripts/test_all.sh --unit-only     # yalnızca birim testleri (~5 s)
./scripts/sim_e2e_test.sh A B         # seçili uçtan uca aşamalar
E2E_NAME=kelo-e2e-2 E2E_DOMAIN=78 IMG=kelo-slam:dev ./scripts/sim_e2e_test.sh B   # çalışan GUI'nin yanında
```

**Birim testleri (pytest)** yapılandırma tutarlılığını (footprint, hız sınırları, bölgeler, kör bölge,
kaynaklar), odometri modeli ve tersini, simülatör ışın izleme ve çarpışmayı, tek launch yolu kurallarını
(özdeş ortak düğümler, tek TF sahibi, tutarlı odometri topic'leri) ve konumlandırma mantığını kapsar.
Tüm sensörleri gerektiren testler yığını bütün sensörler açık olacak şekilde kurar; bu nedenle sonuçlar
`hardware.yaml → sensors` altındaki mevcut ayarlara bağlı değildir.

**Uçtan uca aşamalar** (`sim_e2e_test.sh`; `E2E_NAME` ve `E2E_DOMAIN` container adını ve ROS domain'ini
belirler, böylece test GUI'deki simülatörün (domain 77) yanında çalışabilir):

| Aşama | Kapsam |
|---|---|
| A | Haritalama, kapı, watchdog, sınırlar, güvenlik durdurması, TF, sensörler, harita, hizalama, bag |
| B | Navigasyon, bilinmeyen yönden konumlandırma, engel ve sensör kesintisi senaryoları, duvara yaklaşma, engel etrafından dolaşma, insanlar (duran, giden, karşıdan geçen), hedef hareketi, global konumlandırma, tekrarlı konumlandırma, taşınan robot (`LOST`, yeniden konumlandırma), son kaydedilen konumla yeniden başlatma |
| C | `test/data/sim_reference`'a karşı replay regresyonu |
| D | Odometri modeli ve düzeltme, kalibrasyon modu |

Container içinde tekil senaryolar `kelo_e2e <senaryo>` ile çalıştırılır: `odom`, `tour`, `localization`,
`obstacle`, `dropout`, `teleop_obstacle`, `teleop_dropout`, `near_wall`, `goal_motion`, `detour`, `people stand|leave|cross`, `relocalize`, `kidnap`, `restored`, `world_tour <dünya>`, `world_goals <dünya> [out.json]`.

## İşletim ve bakım

**İlk bağlantı kontrol listesi**

1. [Açık konuları](#açık-konular) netleştirin ve ölçümleri `hardware.yaml`'a girin.
2. Sensör modunda (`KELO_MODE=sensors`) başlatın; `/scan`, derinlik, Marvelmind, `/odom` ve TF ağacını doğrulayın.
   `ros2 run kelo_bringup robot_check` bunu robotu hareket ettirmeden otomatik yapar: `/odom` frekansı,
   frame'leri ve saat farkı (chrony), `odom → base_link`'i kimin yayınladığı (`odom_mode` ile uyumlu
   olmalı), sürücünün `/cmd_vel` abonesinin tipi ve başka bir node'un bu topic'e yayın yapmadığı, LiDAR,
   derinlik kamerası, Marvelmind ve harita. Sürücünün kendi dönüşümlerini görmek için bir kez KELO stack
   kapalıyken (yalnızca robot sürücüsü) çalıştırın; herhangi bir `FAIL` varsa çıkış kodu 1'dir.
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
- Marvelmind olmadan global konumlandırma simetrik bir ortamda benzer görünen başka bir yere oturabilir.
  Tarama uyumu duvarların tutmadığı yerleri reddeder, ancak 180° döndürüldüğünde neredeyse aynı kalan
  bir odada yanlış yer de uyabilir. **Localize** sonrasında konumu RViz'de kontrol edin veya konumu
  **Set Initial Pose** ile verin.
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
