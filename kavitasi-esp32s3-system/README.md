# 🌊 Sistem Monitoring Kavitasi — ESP32-S3

Dashboard monitoring kavitasi berbasis IoT dengan ESP32-S3, sensor WPT83G + MPU6050 + ACS712, kontrol Fuzzy-PID, dan web dashboard via GitHub Pages.

---

## 📦 Komponen Sistem

```
esp32-kavitasi/
├── esp32_firmware/
│   └── main.ino              ← Firmware ESP32-S3
├── web/
│   └── index.html            ← Web Dashboard
├── data/
│   └── sensor_data.json      ← Data sensor (di-update ESP32)
└── .github/workflows/
    └── deploy.yml            ← Auto-deploy ke GitHub Pages
```

---

## 🔧 Hardware

| Komponen       | Keterangan                              | Pin ESP32-S3      |
|----------------|-----------------------------------------|-------------------|
| WPT83G         | Sensor tekanan air 0–10 Bar (0.5–4.5V)  | GPIO34 (ADC)      |
| MPU6050        | Akselerometer 3-axis + Gyro             | SDA=GPIO21, SCL=GPIO22 |
| ACS712-5A      | Sensor arus ±5A                         | GPIO35 (ADC)      |
| Optocoupler UP | Relay kecepatan NAIK                    | GPIO25             |
| Optocoupler DN | Relay kecepatan TURUN                   | GPIO26             |

### Wiring WPT83G
```
VCC  → 5V (atau 3.3V sesuai datasheet)
GND  → GND
OUT  → GPIO34 (melalui voltage divider jika 5V output)
```

### Wiring ACS712
```
VCC  → 5V
GND  → GND
OUT  → GPIO35 (output ~2.5V saat 0A, sensitivitas 185mV/A)
```

---

## ⚙️ Setup Arduino IDE

### Library yang diperlukan:
```
- MPU6050 by Electronic Cats (v0.7.0+)
- ArduinoJson (v6.x)
- WiFi (built-in ESP32)
- HTTPClient (built-in ESP32)
```

### Board Manager:
```
URL: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
Board: ESP32S3 Dev Module
Upload Speed: 921600
Flash Size: 4MB
```

### Konfigurasi `main.ino`:
```cpp
const char* WIFI_SSID     = "NAMA_WIFI_ANDA";
const char* WIFI_PASSWORD = "PASSWORD_WIFI";
const char* GITHUB_TOKEN  = "ghp_TOKEN_ANDA";   // Personal Access Token
const char* GITHUB_REPO   = "username/nama-repo";
```

---

## 🔑 GitHub Token Setup

1. Buka: https://github.com/settings/tokens/new
2. Note: `ESP32 Kavitasi Monitor`
3. Expiration: sesuai kebutuhan
4. Scopes: ✅ `repo` (full control)
5. Copy token → paste ke `GITHUB_TOKEN` di firmware

---

## 📊 Web Dashboard Setup

### 1. Buat Repository GitHub Baru
```bash
git init
git remote add origin https://github.com/USERNAME/REPO.git
```

### 2. Tambah semua file
```bash
git add .
git commit -m "Initial: Kavitasi Monitor System"
git push -u origin main
```

### 3. Aktifkan GitHub Pages
- Settings → Pages
- Source: **GitHub Actions**
- Tunggu deploy selesai (~2 menit)

### 4. Akses Dashboard
```
https://USERNAME.github.io/REPO/
```

---

## 🧠 Algoritma Fuzzy-PID

### Membership Functions (Error)
```
NB: [-10, -10, -3, -1]   → Negative Big
NS: [-3, -2, -1, 0]      → Negative Small  
ZE: [-0.5, 0, 0, 0.5]    → Zero
PS: [0, 1, 2, 3]         → Positive Small
PB: [1, 3, 10, 10]       → Positive Big
```

### Rule Base → Adaptasi Kp, Ki, Kd
| Error | ΔKp | ΔKi | ΔKd |
|-------|-----|-----|-----|
| NB    | +0.8 | +0.3 | +0.1 |
| NS    | +0.4 | +0.2 | +0.05 |
| ZE    | 0    | +0.1 | 0    |
| PS    | +0.4 | +0.2 | +0.05 |
| PB    | +0.8 | +0.3 | +0.1 |

### Kontrol Optocoupler
```
PID > +0.5  → OPTO_UP = HIGH  (naikkan kecepatan)
PID < -0.5  → OPTO_DOWN = HIGH (turunkan kecepatan)
Else        → keduanya OFF
```

---

## 📈 Klasifikasi Kavitasi

### Indeks Kavitasi (Thoma Number σ)
```
σ = (P_static - P_vapor) / P_static
P_vapor air pada 20°C ≈ 0.023 Bar
```

### Skor Kavitasi (Multi-parameter)
```
Skor = 40% × (faktor tekanan) + 40% × (faktor vibrasi) + 20% × (faktor arus)
```

### Klasifikasi
| Kelas | Label          | Skor    | Kondisi |
|-------|----------------|---------|---------|
| 0     | NORMAL         | < 20%   | Aman, operasi normal |
| 1     | INCIPIENT      | 20–45%  | Awal kavitasi, pantau |
| 2     | DEVELOPED      | 45–75%  | Kavitasi aktif, kurangi kecepatan |
| 3     | SUPERCAVITATION | > 75%  | Kritis! Hentikan pompa |

---

## 📁 Format Data JSON
```json
{
  "updated_at": 1715000000000,
  "readings": [
    {
      "ts": 1000,
      "pressure": 4.200,
      "accel_x": 0.010,
      "accel_y": 0.020,
      "accel_z": 1.000,
      "vibration": 0.120,
      "current": 5.200,
      "cav_index": 0.995,
      "cav_class": 0,
      "cav_label": "NORMAL",
      "pid_output": 0.100,
      "opto_up": false,
      "opto_down": false
    }
  ]
}
```

## 📥 Format CSV (Download dari Dashboard)
```csv
Timestamp_ms,Tekanan_Bar,Vibrasi_g,Arus_A,Accel_X_g,Accel_Y_g,Accel_Z_g,CavIndex_sigma,Cav_Class,Cav_Label,PID_Output,OptoUP,OptoDOWN
1000,4.200,0.120,5.200,0.010,0.020,1.000,0.995,0,NORMAL,0.100,0,0
```

---

## 🔄 Alur Data

```
ESP32-S3
  ├── Baca WPT83G (ADC) → Tekanan (Bar)
  ├── Baca ACS712 (ADC) → Arus (A)
  ├── Baca MPU6050 (I2C) → Akselerasi + Vibrasi
  ├── Hitung Kavitasi Index (σ Thoma)
  ├── Klasifikasi Kavitasi (0-3)
  ├── Jalankan Fuzzy-PID
  ├── Kontrol OPTO_UP / OPTO_DOWN
  └── Upload ke GitHub (setiap 30 detik)
        ↓
  GitHub Repository
        ↓
  GitHub Actions (auto-deploy)
        ↓
  GitHub Pages → Web Dashboard
        ↓
  Browser → Grafik + Status + CSV
```
