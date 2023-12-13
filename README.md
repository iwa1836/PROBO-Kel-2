# Deskripsi
Repository sebagai lampiran program dalam laporan mata kuliah Proyek Robotika. Terdapat 2 folder utama, yaitu:
- `GCS/`: Folder tempat source code dan compiled program dari aplikasi desktop sebagai antarmuka robot.
- `Firmware/`: Folder tempat source code dari firmware/program yang digunakan pada robot.

## Kapabilitas Program:
- [x] Komunikasi wireless menggunakan protokol WebSocket dalam jaringan Wi-Fi lokal.
- [x] Read/write konfigurasi robot (Kp, Ki, Kd, base speed, threshold) melalui aplikasi GCS.
- [x] Kendali manual robot.
- [x] Program mekanisme mengambil bola.
- [x] Program mekanisme menyimpan bola.
- [x] Program mekanisme menjatuhkan dan melemparkan bola.
- [ ] Kemampuan menjalankan misi dalam mode otomatis hingga finish dengan sempurna.
- [ ] Kemampuan memasukkan bola ke dalam target secara akurat.
- [ ] Kemampuan berjalan mengikuti garis secara akurat.

## Catatan:
- Aplikasi GCS dikembangkan menggunakan Visual Studio Community 2022 dengan .NET Framework (WPF Application).
- Firmware dikembangkan menggunakan PlatformIO pada Visual Studio Code untuk mikrokontroler ESP32 DevkitC v4.

# Anggota Kelompok 2
- Muhammad Fahrizal - 4121600002
- Ziani Yaniar - 4121600007
- Ubaidillah Ramadhan N. S. - 4121600012
- Fatwa Aulia Al-Haq - 4121600020
- Diyon Aziz Saifulloh - 4121600025
