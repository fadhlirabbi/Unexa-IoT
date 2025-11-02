# ⛽ Unexa-IoT: Sistem Monitoring Tangki Bahan Bakar Solar (Diesel) ESP32-S3

[![Platform](https://img.shields.io/badge/Platform-ESP32--S3-blue.svg)](https://www.espressif.com/en/products/socs/esp32-s3)
[![Connectivity](https://img.shields.io/badge/Connectivity-HTTPS%20%7C%20WiFi%20%7C%20SD%20Card-orange.svg)]()
[![License](https://img.shields.io/badge/License-MIT-green.svg)]()

**Unexa-IoT** adalah solusi *firmware* yang dirancang untuk pemantauan level, konsumsi, dan status keamanan pada **Tangki Penyimpanan Solar (Diesel)**. Sistem ini vital untuk manajemen inventaris bahan bakar di lokasi industri atau genset, memastikan ketersediaan pasokan dan mencatat waktu kerja perangkat listrik terkait (misalnya pompa atau genset).

Keandalan data dijamin melalui pencatatan lokal ke **SD Card** saat koneksi *cloud* terputus, dan komunikasi aman menggunakan **HTTPS/TLS**.

***

## ✨ Fitur Inti dan Reinterpretasi Sensor

Sistem ini mengubah data sensor fisik menjadi data manajemen bahan bakar yang krusial:

| Kategori | Fitur yang Dimonitor | Komponen Kunci dalam Kode |
| :--- | :--- | :--- |
| **Level & Inventaris** | **Level Bahan Bakar (*Fuel Level*)** | **Sensor Ultrasonik (UART)**: Mengukur jarak dari sensor ke permukaan solar (mm) secara akurat untuk menghitung sisa volume. |
| | **Alarm Level Tinggi (Overfill)** | **Saklar Batas (*Limit Switch*, Pin 15)**: Memicu notifikasi darurat saat solar mencapai level kritis (misalnya saat pengisian berlebih). |
| **Konsumsi & Aliran** | **Laju Konsumsi/Transfer** | **Sensor Aliran (*Flow Sensor*, Pin 36)**: Memantau laju solar yang ditransfer (L/menit), penting untuk melacak konsumsi aktual atau laju pengisian. |
| **Perangkat Pendukung** | **Waktu Kerja Genset/Pompa** | **Sensor Tegangan AC (ZMPT, Pin 1)**: Mencatat total *running hour* (jam kerja) perangkat AC (Genset atau Pompa Transfer) yang terhubung ke tangki, dicatat secara persisten di NVS. |
| **Integritas Data** | **Pencatatan Data Kritis** | **SD Card Backup**: Merekam semua data level, aliran, dan status AC ke file `/data.txt` ketika koneksi WiFi terputus, menjamin tidak ada *data loss*. |

***

## 🛠️ Persyaratan Hardware & Wiring

Sistem ini dibangun di atas mikrokontroler **ESP32-S3** karena kebutuhan konektivitas dan *processing power* untuk sensor yang beragam:

| Komponen | Pin (GPIO) | Keterangan |
| :--- | :--- | :--- |
| **Ultrasonic Sensor (UART)** | **RX: 18, TX: 17** | Harus sensor yang tahan terhadap lingkungan solar atau ditempatkan pada tabung pelindung. |
| **Limit Switch** | **15** | Untuk mendeteksi level maksimal solar. |
| **AC Voltage Sensor (ZMPT)** | **1** | Untuk monitoring status Genset/Pompa. |
| **Flow Sensor** | **36** | Harus menggunakan *flow sensor* yang kompatibel dengan bahan bakar diesel. |
| **SD Card Module** | **CS: 40, MISO: 13, MOSI: 11, SCK: 12** | Untuk penyimpanan data offline. |
| **RTC DS3231** | **SDA: 8, SCL: 9** | Menyediakan stempel waktu yang akurat (UTC) untuk semua log data. |
| **Modem Power Relay** | **48** | Mengontrol daya radio WiFi/modem untuk efisiensi atau *recovery*. |

***

## 🌐 Akses dan Konfigurasi Sistem

### 1. Dashboard Web Lokal

Setelah perangkat terhubung ke WiFi, atau saat dalam mode AP, Anda dapat mengakses *dashboard* lokal untuk monitoring dan kontrol:

* **Akses:** `http://[IP_Perangkat]`
* **Data Real-time:** Menampilkan level solar (mm), laju aliran (L/menit), status AC, dan Total *Running Hour* AC.
* **Kontrol Perangkat:**
    * **Konfigurasi Pengaturan (/config):** Mengubah kredensial WiFi dan mengatur IP Statis.
    * **Reset Running Hour AC:** Mengatur ulang total jam kerja AC/Genset yang tersimpan ke nol.
    * **Manajemen Log:** Melihat, mengunduh, atau melakukan **Penghapusan Log Kritis** (Factory Reset).

### 2. Mode Konfigurasi (AP Default)

Jika perangkat tidak dapat terhubung ke WiFi, perangkat akan secara otomatis masuk ke **Access Point Mode** untuk memfasilitasi konfigurasi:

* **SSID AP:** `IoTMaxWin`
* **Password AP:** `MAXwin@2025.,`

### 3. Jadwal Operasi dan Recovery

| Fitur | Waktu | Tujuan |
| :--- | :--- | :--- |
| **Modem Power Cycling** | **22:00 hingga 23:00 UTC** | Mematikan radio WiFi/modem untuk menghemat daya atau melakukan *soft-reset* jaringan. |
| **Restart Terjadwal** | **05:00 UTC dan 17:00 UTC** | Memastikan sistem *refresh* dan mengatasi kebocoran memori (mempertahankan stabilitas jangka panjang). |
| **Restart Reaktif** | **Otomatis** | Dipicu setelah **10 kegagalan HTTP berturut-turut** saat mengirim data ke *cloud*, memaksa *power cycle* modem untuk memulihkan koneksi. |
