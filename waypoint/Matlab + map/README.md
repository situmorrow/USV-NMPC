# Simulasi NMPC Waypoint Tracking USV Autonomous - Danau 8 ITS

Repositori/folder ini berisi modul simulasi kendali **Nonlinear Model Predictive Control (NMPC)** untuk pelacakan rute titik acuan (*Waypoint Tracking*) kapal otonom (*Unmanned Surface Vehicle / USV*) berlokasi di perairan **Danau 8 ITS, Surabaya**.

---

## 📌 Daftar Isi
1. [Deskripsi Singkat](#-deskripsi-singkat)
2. [Struktur File Folder](#-struktur-file-folder)
3. [Spesifikasi Kapal USV RC](#-spesifikasi-kapal-usv-rc)
4. [Evaluasi Batasan Kendali (Constraints)](#-evaluasi-batasan-kendali-constraints)
5. [Koordinat Peta & Waypoint Danau 8 ITS](#-koordinat-peta--waypoint-danau-8-its)
6. [Cara Menjalankan Simulasi](#-cara-menjalankan-simulasi)
7. [Penjelasan Output Simulasi](#-penjelasan-output-simulasi)

---

## 🚀 Deskripsi Singkat

Simulasi ini dirancang untuk menguji performa kendali NMPC dalam mengarahkan kapal USV melalui sekelompok *waypoints* presisi di area perairan Danau 8 ITS. Model matematika yang digunakan adalah **Model Kapal 2-DOF (Sway & Yaw)** dengan koefisien hidrodinamika yang disesuaikan dari data fisik dan validasi model eksperimental USV RC.

Sistem kendali bekerja secara *closed-loop* dengan siklus optimasi `fmincon` (Sequential Quadratic Programming - SQP) pada frekuensi sampling **10 Hz ($T_{sim} = 0.1\text{ s}$)** dan diintegrasikan secara numerik menggunakan metode **Runge-Kutta Orde ke-4 (RK4)**.

---

## 📁 Struktur File Folder

```text
Matlab + map/
├── README.md                           # Dokumentasi resmi simulasi NMPC Danau 8 ITS
├── nmpc_kapal_waypoint_danau8its.m     # Entry-point controller NMPC khusus USV RC Danau 8 ITS
├── run_nmpc_danau8its.m                # Skrip utama simulasi, visualisasi map & ekspor video MP4
├── nmpc_kapal_waypoint.m               # Controller NMPC awal (skala kapal 101 m)
├── run_nmpc.m                          # Skrip simulasi awal (skala kapal 101 m)
├── simulasi_nmpc_danau8its_video.mp4   # Output video animasi simulasi MP4
├── hasil_simulasi_danau8its.png        # Grafik respons state & trajektori peta ENU
├── grafik_batasan_kendali.png          # Grafik evaluasi batasan r_limit, u_limit, dan u_rate_limit
└── trajektori_geografis_danau8its.png  # Grafik trajektori koordinat geografis (Lat/Lon)
```

---

## 🚢 Spesifikasi Kapal USV RC

Data parameter kapal diambil dari folder validasi eksperimen (`USV-NMPC/validasi_model_dan_nmpc`):

| Parameter | Simbol | Nilai | Satuan |
| :--- | :---: | :---: | :---: |
| Panjang Kapal (*Length*) | $L$ | **1.20** | m |
| Lebar Kapal (*Beam*) | $B$ | **0.35** | m |
| Draft Kapal (*Draft*) | $T$ | **0.10** | m |
| Massa Kapal (*Mass*) | $m$ | **8.50** | kg |
| Kecepatan Surge Rata-Rata | $u_0$ | **1.08** | m/s (~2.1 knot) |
| Koefisien Blok | $C_B$ | **0.65** | - |
| Pusat Massa Sumbu-X | $x_G$ | **0.05** | m |
| Massa Jenis Air Danau | $\rho$ | **1000** | kg/m$^3$ |

---

## 🛑 Evaluasi Batasan Kendali (Constraints)

Pengontrol NMPC secara ketat mematuhi batasan operasional berikut:

| Batasan Operasional | Simbol / Variabel | Batas Minimum | Batas Maksimum | Satuan |
| :--- | :---: | :---: | :---: | :---: |
| Kecepatan Putar (*Yaw Rate*) | $r$ | **-45.0** | **+45.0** | deg/s |
| Sudut Kemudi Rudder | $\delta$ | **-35.0** | **+35.0** | derajat |
| Rate Perubahan Kemudi (*Slew Rate*) | $\Delta\delta / \Delta t$ | **-150.0** | **+150.0** | deg/s |

Grafik evaluasi batasan diekspor secara otomatis ke berkas [`grafik_batasan_kendali.png`](file:///d:/2026/Percobaan_Kapal_Autonomous/USV-NMPC/waypoint/Matlab%20+%20map/grafik_batasan_kendali.png).

---

## 🗺️ Koordinat Peta & Waypoint Danau 8 ITS

### Titik Acuan Awal (*Home Point*)
- **Latitude**: `-7.28715^\circ`
- **Longitude**: `112.79600^\circ`
- **Heading Awal**: `0.0^\circ` (Menghadap Sumbu-X ENU)

### Batas Perairan Danau 8 ITS
Digunakan poligon 21 titik koordinat presisi yang melingkupi perairan Danau 8 ITS dari berkas `danau_8_its.geojson`.

### Sekuens Waypoint
Kapal USV akan menavigasi sekuens 4 waypoint berikut:

| Waypoint | Latitude | Longitude | Posisi ENU X (m) | Posisi ENU Y (m) | Keterangan Area |
| :---: | :---: | :---: | :---: | :---: | :--- |
| **Start / WP 4** | `-7.28715` | `112.79600` | `0.00` | `0.00` | Kolam Selatan Danau 8 ITS |
| **WP 1** | `-7.28675` | `112.79600` | `0.00` | `44.48` | Penyempitan Saluran Tengah |
| **WP 2** | `-7.28630` | `112.79600` | `0.00` | `94.52` | Kolam Utara Danau 8 ITS |
| **WP 3** | `-7.28675` | `112.79610` | `11.03` | `44.48` | Sisi Timur Tengah |

- **Switching Radius ($r_{tran}$)**: `3.50` meter (~3 kali panjang kapal).

---

## 💻 Cara Menjalankan Simulasi

1. Buka aplikasi **MATLAB** (versi R2021a atau yang lebih baru direkomendasikan).
2. Pindahkan *Current Folder* MATLAB ke lokasi skrip:
   ```matlab
   cd('d:\2026\Percobaan_Kapal_Autonomous\USV-NMPC\waypoint\Matlab + map')
   ```
3. Jalankan skrip utama simulasi:
   ```matlab
   run_nmpc_danau8its
   ```
4. Simulasi akan menampilkan progres per iterasi pada Command Window, melakukan integrasi RK4 10 Hz, serta secara otomatis mengekspor berkas gambar dan video animasi MP4.

---

## 📊 Penjelasan Output Simulasi

Setelah skrip selesai dijalankan, file-file hasil berikut akan dibuat di folder:

1. **`simulasi_nmpc_danau8its_video.mp4`**:
   - Berkas video MP4 yang merekam pergerakan real-time kapal USV mengelilingi Danau 8 ITS beserta respons grafik sinyal dan garis batas kendali.
2. **`grafik_batasan_kendali.png`**:
   - Grafik evaluasi 3 batasan kendali NMPC (Yaw Rate $r$, Sudut Kemudi $\delta$, dan Perubahan Sudut Kemudi $d\delta/dt$) lengkap dengan garis batas ambang merah (*red dashed limit lines*).
3. **`hasil_simulasi_danau8its.png`**:
   - Gambar grafik kombinasi yang menampilkan peta perairan Danau 8 ITS, lintasan NMPC kapal, lokasi waypoints, dan subplot dinamika state.
4. **`trajektori_geografis_danau8its.png`**:
   - Visualisasi trajektori kapal di atas peta geografis basemap MATLAB (*streets-light*).

---

## 🛠️ Kontak & Lisensi
Proyek Simulasi Kendali USV Autonomous NMPC - Departemen Teknik Perkapalan / Teknik Sistem Perkapalan, Institut Teknologi Sepuluh Nopember (ITS).
