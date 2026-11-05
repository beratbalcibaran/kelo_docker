# Robota bağlama: adım adım

[English](robot_startup.md) | **Türkçe**

Bu sayfa, Jetson'daki KELO yığınını gerçek robota bağlayıp haritalama ve navigasyona kadar götürmek
için gereken adımları sırasıyla anlatır. Kurulumun kendisi (Docker, robot bilgisayarındaki sürücü,
ağ ayarları) [README → Sıfırdan kurulum](../README.tr.md#sıfırdan-kurulum-jetson-ve-robot) bölümündedir.

Robot bilgisayarına hiçbir şey kurulmaz ve orada otomatik çalışan bir şey yoktur. Sürücü her zamanki
gibi `scripts/robot.sh` ile Jetson'dan başlatılır. Geri kalan her şey Jetson tarafında kendiliğinden
olur: kablo takılınca robot bulunur, ayarlar uygulanır, harita varsa navigasyon başlar ve robot
kendisi konumlanır.

## 0. Kodu güncelleme ve image'ı build etme (her güncellemede bir kez)

```bash
cd ~/Desktop/kelo_docker
sudo systemctl stop kelo                      # üretim servisi çalışıyorsa
git fetch
git checkout gelistirme-v1
git pull
docker compose --profile gui-hardware build   # Jetson'da 15-30 dk
```

Robotun ön yönü (`base_link` +x sensörlerin tarafı) değiştiyse, değişiklikten önce çıkarılmış harita
artık doğru değildir. Kenara alın, yoksa "Automatic" açıkken navigasyon o haritayla başlar:

```bash
mv data/maps/current.yaml data/maps/eski_current.yaml
```

## 1. Robot bilgisayarı

```bash
./scripts/robot.sh status      # container ve sürücü durumu
./scripts/robot.sh deploy      # yalnız robot/ değiştiyse
./scripts/robot.sh build       # yalnız deploy'dan sonra
./scripts/robot.sh start       # sürücüyü başlat
./scripts/robot.sh time        # robot saatini Jetson'a eşitle (sudo şifresi sorar)
```

Saat eşitlemesi zorunlu değildir ama en temiz yoldur. Yapılmazsa yığın odometri zaman damgalarını
Jetson saatine taşır (`odom_mode: corrected` ve `relay`); `driver` modunda bunu yapamaz ve panel
"robot clock is … off: run scripts/robot.sh time" diye uyarır.

## 2. Bağlantıyı hareket ettirmeden kontrol (önerilir)

Yığın kapalıyken, yalnız sürücü açıkken:

```bash
docker compose run --rm production ros2 run kelo_bringup robot_check
```

Bakılacaklar: `/odom` hızı (~20 Hz), saat farkı, `odom → base_link`'i kimin yayınladığı, `/cmd_vel`
tipi. `FAIL` varsa önce onu çözün.

## 3. Paneli açma ve kabloyu takma

```bash
docker compose --profile gui-hardware up
```

1. Panelde **Automatic** işaretli olsun (varsayılan).
2. Ethernet kablosunu takın. Panel robotu arar, sürücünün ayarlarını okur ve uygun `odom_mode`'u
   (`driver`, `relay` ya da `corrected`) sormadan `/data/hardware.yaml`'a yazar. Sürücü henüz açık
   değilse 10 s'de bir yeniden arar.
3. Logda `odom_mode` satırına bakın.
4. Harita yoksa logda "Automatic: no map … build one with Start Mapping -> Save Map" görünür. Bu beklenen durumdur, 4. adıma geçin.
   Harita varsa navigasyon kendiliğinden başlar, 5. adıma geçin.

## 4. Harita çıkarma

1. **Start Mapping**.
2. Robotu klavyeyle (WASD / ok tuşları) ya da `robot.sh joy safe` ile odanın tamamında yavaşça sürün.
   Duvarları ve köşeleri en az bir kez görsün, mümkünse başladığınız yere dönün.
3. **Save Map**, sonra **Stop Mapping**.

## 5. Navigasyon

1. **Start Navigation** (harita varken "Automatic" bunu kendisi yapar).
2. Robotu düz bir yerde, önü 2 m boş olarak birkaç saniye bekletin. Bu sırada derinlik kamerası zemine
   bakarak kendi eğimini ölçer; logda `camera tilt from the floor: roll … pitch …` satırı çıkar.
3. 15 s içinde kimse **Localize**'a basmazsa robot kendisi konumlanır. Gerekirse yerinde döner:
   **çevresinin boş olduğundan emin olun.**
4. Üstteki **Robot:** satırı **READY** (yeşil) olmalı. Değilse satır nedenini söyler (odometri yok,
   saat farkı, iki TF yayıncısı, LiDAR yok, konumlanmadı, navigasyon düğümü yeniden başlıyor …).
5. Önce 1-2 m'lik kısa bir hedef verin (**Send Goal** ya da RViz'de *2D Goal Pose*), sonra uzun bir
   hedef. Robot beklerse satır nedenini gösterir (`WAITING: …`).

## 6. Neye bakılır

| Yer | Ne söyler |
|---|---|
| **Robot:** satırı | READY / NOT READY: neden / WAITING: neden / DRIVING |
| **Health:** satırı | Düşen veya yeniden başlayan düğümler, CPU, GPU, RAM, sıcaklık, en çok CPU kullanan 3 süreç |
| **Nodes** düğmesi | Tüm düğümler, durumları ve kaç kez yeniden başladıkları |
| Log: `camera tilt from the floor` | Ölçülen kamera eğimi (1-2° üzeri: montaj ya da `hardware.yaml` değeri kontrol edilmeli) |
| Log: `robot clock` | Robot ile Jetson arasındaki saat farkı |
| Log: `incident … saved` | Sorun anının kaydı `/data/incidents/` altında |

Bir sorun olursa `data/incidents/` klasörünü ve logu saklayın:

```bash
docker logs kelo-gui-hardware 2>&1 | grep -E "STATUS|LOST|robot clock|odom_conflict|camera tilt|health_monitor" > olay.txt
```

Kayıtlar simülatörde `ros2 bag play` ile tekrar oynatılabilir.

## 7. Panelsiz çalıştırma (her şey doğrulandıktan sonra)

```bash
sudo systemctl enable --now kelo
journalctl -u kelo -f
```

Üretim servisi açılışta sürücüyü bulana kadar 10 s'de bir arar, ayarları `/data/hardware.yaml` ve
`/data/robot.env`'e yazar, sonra yığını başlatır. Kapatmak için `/etc/kelo/kelo.env` içinde
`KELO_AUTO=0`.

## Sorun giderme

| Belirti | Bakılacak yer |
|---|---|
| `NOT READY: no odometry from the robot` | Kablo, `robot.sh status`, `ROS_DOMAIN_ID` 42 |
| `NOT READY: robot clock is … off` | `./scripts/robot.sh time` |
| `NOT READY: two publishers of odom -> base_link` | **Find robot → Apply settings** (`odom_mode: driver`) |
| `NOT READY: not localized` | **Localize**; olmazsa **Set Initial Pose** |
| `WAITING: obstacle on the path … at (x, y)` | RViz'de o noktada costmap'e bakın; gerçek bir engel yoksa kamera eğimini kontrol edin |
| `WAITING: collision monitor stop` | Robotun hemen önünde bir şey var (ya da kamera/LiDAR verisi eski) |
| `goal given up after 60 s without progress` | Satırdaki neden; `data/incidents/` kaydı |
| **Health:** kırmızı | **Nodes** listesinde düşen düğüm; birkaç saniyede kendisi geri gelmeli |
| Robot ileri yerine geri gidiyor | Sürücü ayarındaki ön yön; harita yeniden çıkarılmalı |
