# Panduan Pengujian Virtual (SITL) `CPP_Gabungan` dengan Python Dashboard

Dokumen ini menjelaskan cara melakukan pengujian **Software-in-the-Loop (SITL)** untuk modul kendali C++ NMPC (`CPP_Gabungan`) secara langsung di **1 PC/Laptop Anda**, tanpa perlu menghubungkan modul fisik ESP32 (*User-Side* maupun *Remote-Side*).

---

## 1. Jawaban Utama: Apakah Sudah Bisa Diuji?

> **YA, SANGAT BISA!**
> Seluruh modul C++ pada Tahap 1, 2, 3, dan 4 (`read_write_serial.exe`) sudah lengkap dan siap beroperasi. 
> Anda dapat mensimulasikan pergerakan kapal, melihat visualisasi rute peta di `Local Monitor Dashboard-beta1.5.py`, serta mengirim titik waypoint baru secara interaktif hanya menggunakan PC Anda saat ini.

---

## 2. Arsitektur Pengujian Virtual SITL

Di lapangan nyata, sistem bekerja menggunakan kabel serial dan transmisi nirkabel ESP-NOW:
```text
[Dashboard Python] <---USB---> [ESP32 User] <---ESP-NOW---> [ESP32 Remote] <---USB---> [Mini PC C++]
```

Pada simulasi mandiri di PC (SITL), kita menggantikan seluruh perangkat keras ESP32 dan badan kapal nyata dengan **Virtual Serial Bridge & Dynamic Plant Simulator (`sitl_simulator.py`)**:

```text
+-----------------------------------------------------------------------------------------+
|                                    1 KOMPUTER / LAPTOP                                  |
|                                                                                         |
|  +--------------------------------+                   +-------------------------------+ |
|  | Python Dashboard               |                   | C++ NMPC Controller           | |
|  | Local Monitor Dashboard-beta1.5|                   | read_write_serial.exe         | |
|  +--------------------------------+                   +-------------------------------+ |
|                 ^                                                     ^                 |
|                 | (Serial 24-Kolom @ 10 Hz)                           | (CSV 8-Kolom)   |
|                 v                                                     v                 |
|          [Virtual COM1]                                        [Virtual COM4]           |
|                 |                                                     |                 |
|                 | (Pair 1: com0com)                                   | (Pair 2: com0com|
|                 v                                                     v                 |
|          [Virtual COM2]                                        [Virtual COM3]           |
|                 ^                                                     ^                 |
|                 +-----------------------+   +-------------------------+                 |
|                                         |   |                                           |
|                               +-----------------------+                                 |
|                               | Python SITL Simulator |                                 |
|                               |   sitl_simulator.py   |                                 |
|                               | (Model WyNDA 11 Param)|                                 |
|                               +-----------------------+                                 |
+-----------------------------------------------------------------------------------------+
```

---

## 3. Persiapan & Kebutuhan Alat

### A. Virtual Serial Port Driver (Sepasang COM Port Virtual di Windows)
Untuk menghubungkan 2 program dalam 1 PC lewat protokol serial, gunakan software Virtual COM Port gratis:
1. **com0com (Sangat Direkomendasikan / Open Source):**
   - Download: [com0com di SourceForge](https://sourceforge.net/projects/com0com/)
   - Buat 2 pasang port virtual:
     - **Pasangan 1:** `COM1` $\longleftrightarrow$ `COM2` (untuk Dashboard $\leftrightarrow$ Simulator)
     - **Pasangan 2:** `COM3` $\longleftrightarrow$ `COM4` (untuk Simulator $\leftrightarrow$ NMPC C++)
2. *Alternatif lain:* **Virtual Serial Port Driver (VSPD)**, **HW VSP**, atau **Virtual Serial Ports Emulator (VSPE)**.

### B. Dependensi Python
Pastikan pustaka pendukung terinstall di Python Anda:
```powershell
pip install pyserial PySide6 numpy pyqtgraph
```

---

## 4. Langkah demi Langkah Menjalankan Uji Coba

Buka **3 jendela Terminal / PowerShell** di PC Anda:

### Jendela 1: Jalankan Program C++ NMPC
```powershell
cd D:\2026\Percobaan_Kapal_Autonomous\USV-NMPC\CPP_Gabungan

# Jalankan NMPC terhubung ke COM4 (Port pasangannya simulator)
.\build\read_write_serial.exe --port COM4 --rudder-mode nmpc --print nmpc
```

### Jendela 2: Jalankan SITL Simulator & Virtual Bridge
```powershell
cd "D:\2026\Percobaan_Kapal_Autonomous\USV-NMPC\pengujian CPP_Gabungan sudah sampai tahap 4"

# Menghubungkan Dashboard di COM2 dan NMPC Controller di COM3
python sitl_simulator.py --dashboard-port COM2 --controller-port COM3
```

### Jendela 3: Jalankan Python Dashboard
```powershell
cd "D:\2026\Percobaan_Kapal_Autonomous\github\Ship_Model_Control_ESP32-S3-v2026.01\Pythonfile\Way_Points_Tracking"

python "Local Monitor Dashboard-beta1.5.py"
```

---

## 5. Apa yang Terjadi di Dashboard?

1. **Koneksi Port:**
   - Pada dropdown port serial di Dashboard, pilih **`COM1`** lalu klik **Connect**.
2. **Status Mini PC:**
   - Indikator status `Mini PC Link` di panel Live akan langsung menyala hijau: **CONNECTED**.
3. **Visualisasi Peta & Gerakan Kapal:**
   - Kapal akan muncul pada peta (di koordinat kolam uji `-7.287150, 112.796000`).
   - Anda dapat melihat pergerakan kapal secara halus melintasi Waypoint 1 $\rightarrow$ Waypoint 2 $\rightarrow$ Waypoint 3 $\rightarrow$ Waypoint 4.
4. **Interaksi Waypoint Baru:**
   - Masuk ke tab **Map Points**, klik titik-titik koordinat baru di peta, lalu tekan tombol **Send Way Points**.
   - Simulator akan meneruskan rute tersebut ke program C++ NMPC secara otomatis, dan kapal akan langsung bermanuver mengejar rute baru tersebut!
5. **Uji Coba Fitur Shutdown:**
   - Klik tombol **Shutdown Mini PC** di panel Live.
   - Program C++ akan menerima sinyal `$SHUTDOWN` dan menghentikan proses kendali secara aman.

---

## 6. Opsi Pengujian Cepat Tanpa Perlu Virtual COM Port

Jika Anda belum menginstall software virtual COM port dan hanya ingin melihat simulasi numerik tertutup (*closed-loop trajectory tracking*), jalankan program simulasi mandiri bawaan C++:

```powershell
cd D:\2026\Percobaan_Kapal_Autonomous\USV-NMPC\CPP_Gabungan

# Jalankan simulasi mandiri 150 detik (Model WyNDA + NMPC)
.\build\test_nmpc_sim.exe
```

Output terminal akan menampilkan manuver pelacakan langkah demi langkah:
```text
[STEP    1 | t= 0.10s] Pos=(  0.00,  0.00)m | Psi= 90.00° | Target=WP#1 (d=44.48m, Los= 90.00°) | Rudder=  0.00° | Flag=1
[STEP  200 | t=20.00s] Pos=(  0.00,  9.54)m | Psi= 90.00° | Target=WP#1 (d=34.94m, Los= 90.00°) | Rudder=  0.00° | Flag=1
...
[STEP  900 | t=90.00s] Pos=(  0.00, 43.09)m | Psi= 90.00° | Target=WP#2 (d=51.42m, Los= 90.00°) | Rudder=  0.00° | Flag=1
...
[GOAL] Kapal BERHASIL mencapai Waypoint Terakhir (WP #4)!
```
