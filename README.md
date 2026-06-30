# Glider Soaring — RL Thermal Soaring Simulation

JSBSim + FlightGear ile termal arama yapan otonom planör simülasyonu.
Reddy et al. (2018) "Glider soaring via reinforcement learning in the field" (Nature) makalesindeki policy'yi temel alır.

## Kurulum

### 1. Sistem paketleri (Ubuntu)
```bash
sudo apt update
sudo apt install flightgear
```

### 2. Python ortamı
```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### 3. ASK21 glider modeli (FlightGear için)
```bash
cd /usr/share/games/flightgear/Aircraft/
sudo wget https://github.com/viktorradnai/flightgear-ask21/archive/refs/heads/master.zip -O ask21.zip
sudo unzip ask21.zip
sudo mv flightgear-ask21-master ASK21
sudo rm ask21.zip
```

### 4. data_output klasörü
```bash
mkdir -p data_output
cat > data_output/flightgear.xml << 'XML'
<?xml version="1.0"?>
<output name="localhost" type="FLIGHTGEAR" port="5550" rate="60" protocol="UDP">
</output>
XML
```

## Çalıştırma

**Terminal 1 — FlightGear:**
```bash
fgfs --aircraft=ask21 --fdm=null \
     --native-fdm=socket,in,60,localhost,5550,udp \
     --timeofday=noon --season=summer \
     --disable-real-weather-fetch --fog-disable \
     --disable-ai-traffic --disable-sound \
     --geometry=1024x768 \
     --lat=39.9483187 --lon=32.6899477 \
     --altitude=9000 --heading=90
```

**Terminal 2 — Simülasyon (FlightGear açıldıktan sonra):**
```bash
source venv/bin/activate
python main.py
```

**Terminal 3 — Üstten görünüm haritası:**
```bash
source venv/bin/activate
python thermal_viz.py
```

## Kontroller

Açılan "Soaring Control" penceresine tıkla, sonra:

| Tuş | Fonksiyon |
|-----|-----------|
| W / S | Elevator yukarı / aşağı |
| A / D | Aileron sol / sağ |
| Q / E | Rudder sol / sağ |
| 1 | MANUAL mod |
| 2 | RL_SOARING mod |
| 3 | TAKEOFF mod |
| ESC | Çıkış |

PS5 DualSense bağlıysa otomatik algılanır (X=Manual, O=RL, △=Takeoff).

## Proje Yapısı

```
.
├── main.py              # Ana simülasyon döngüsü, JSBSim+FlightGear bridge, RL controller
├── thermal_viz.py        # Üstten 2D görselleştirme (pygame)
├── requirements.txt
└── data_output/
    └── flightgear.xml    # JSBSim → FlightGear UDP output config
```

## Bilimsel Arkaplan

RL controller, Reddy et al. (2018, Nature 562:236-239) makalesindeki policy tablosunu kullanır:
- **State**: (az, ω, μ) — dikey rüzgar ivmesi, roll-wise tork, mevcut bank açısı
- **Action**: bank açısını ±15° değiştir veya sabit tut
- **az** ve **ω** adaptif eşiklerle (±0.8 × rolling std) 3 seviyeye discretize edilir

## Notlar / Bilinen Kısıtlar

- Termaller şu an sabit konumlu Gaussian profil (gerçekte rüzgarla kayar)
- Uçak fiziği JSBSim'in built-in c172p modeli (Ranger 2400 custom XML üzerinde çalışılıyor)
- az tahmini basitleştirilmiş (climb rate türevi), makaledeki tam pitch-bias düzeltmesi henüz yok
