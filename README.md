# Prototipe Monitoring Lingkungan Berbasis Node Sensor Surya

Implementasi **baru dari nol** untuk prototipe proposal yang diperluas sesuai diagram arsitektur: panel surya 6 V/1 W, BH1750, NTC baterai, input anemometer analog, beberapa ESP32-C3 SuperMini dengan ESP-NOW relay, dan gateway MQTT. Keputusan penggunaan panel/charger/regulator serta sensor angin harus tetap diverifikasi terhadap komponen aktual dan pembimbing.

## Konteks proposal

- **Node sensor:** ESP32-C3 SuperMini, sensor cahaya digital BH1750, NTC 10 kΩ untuk suhu baterai, input ADC dari anemometer dinamo melalui rangkaian proteksi, dan pemantauan tegangan baterai opsional.
- **Catu daya yang diusulkan:** panel surya 6 V/1 W, dioda Schottky 1N5819, TP4056, baterai 18650 3,7 V, dan regulator HT7333 3,3 V. Susunan dan kompatibilitasnya perlu diverifikasi sebelum dirakit.
- **Jaringan lokal:** topologi broadcast ESP-NOW dengan image firmware `sensor-relay` (mengukur sekaligus me-relay) dan relay aplikasi terbatas oleh deduplikasi serta TTL/hop.
- **Gateway:** ESP32 menerima paket dan menerbitkan payload JSON ke broker MQTT melalui Wi-Fi.
- **Web:** dashboard statis menerima pesan melalui MQTT over WebSocket dan menampilkan lux, suhu baterai, tegangan baterai, tegangan input anemometer, serta daftar node.

Folder ini adalah **prototipe evaluatif**, bukan sistem operasional atau instalasi kampus. Kebutuhan proyek, komponen, rentang pengukuran, lokasi uji, frekuensi sampling, broker, dan kriteria penerimaan perlu disepakati dengan Direktorat UNESA Smart and Sustainability Campus (USSC). Tidak ada asumsi bahwa mitra telah menyetujui proyek atau menyediakan jaringan/fasilitas.

## Struktur

```text
mikrokontroler/
├── platformio.ini
├── partitions.csv
├── include/
│   ├── config.h
│   ├── config.local.example.h
│   └── telemetry.h
├── src/main.cpp
├── web/index.html
└── .github/workflows/pages.yml
```

## Firmware

Dibutuhkan PlatformIO Core. Pilih environment sesuai fungsi board yang diprogram:

Pilih board (`esp32c3` atau `esp32classic`) dan salah satu dari dua image (`gateway` atau `sensor-relay`):

```sh
pio run -e gateway-esp32c3          # gateway + OLED untuk ESP32-C3
pio run -e gateway-esp32classic     # gateway + OLED untuk ESP32 klasik
pio run -e sensor-relay-esp32c3     # sensor + relay untuk ESP32-C3
pio run -e sensor-relay-esp32classic # sensor + relay untuk ESP32 klasik
pio device monitor -b 115200
```

Default `pio run` membangun `gateway-esp32c3`. Gateway menampilkan status koneksi Wi-Fi/SSID, MQTT, jumlah node yang aktif, dan statistik paket pada OLED SSD1306 I2C 128×64 (alamat default `0x3C`). Jika ada lebih dari satu node, layar berganti tiap 5 detik antara status gateway dan halaman tiap node yang berisi lux, suhu, tegangan, RSSI ESP-NOW, hop, nomor urut, serta usia data. Node dianggap aktif bila paket diterima dalam tiga interval sampling. Estimasi jarak tidak ditampilkan karena RSSI saja tidak cukup akurat tanpa kalibrasi lingkungan dan antena. OLED opsional saat runtime: gateway tetap bekerja bila display tidak terdeteksi. Jalur I2C default C3 adalah SDA GPIO8 / SCL GPIO9, dan ESP32 klasik adalah SDA GPIO21 / SCL GPIO22. Sesuaikan pin/alamat pada build flags atau `include/config.local.h` bila wiring berbeda. Semua node yang berkomunikasi melalui ESP-NOW harus menggunakan kanal yang sama dengan gateway.

1. Salin `include/config.local.example.h` menjadi `include/config.local.h`.
2. Wi-Fi dapat dikonfigurasi melalui serial monitor gateway pada 115200 baud. Ketik `wifi help` untuk perintah: `wifi set <ssid> <password>` untuk WPA2-PSK, atau `wifi set eduroam <username> <password> [identity]` untuk WPA2-Enterprise PEAP/MSCHAPv2. Kemudian jalankan `wifi save` agar kredensial disimpan di NVS dan `wifi apply` untuk koneksi saat ini. `wifi ca <single-line PEM>` opsional untuk CA kampus; escape baris baru sebagai `\\n`. `wifi clear` hapus profil NVS. Password tidak dicetak ke log. Atau, konfigurasi `WIFI_SSID`, `WIFI_PASSWORD`, dan nilai eduroam di `include/config.local.h` sebelum build. Jangan membagikan serial log saat memasukkan kredensial atau mengunggah firmware personal ke publik.
3. Jika eduroam kampus mensyaratkan verifikasi sertifikat server, minta CA PEM yang benar dari dukungan TI kampus dan konfigurasi melalui `EDUROAM_CA_CERT` atau perintah serial `wifi ca`. Firmware tidak mencetak password ke serial. Mode PEAP dipilih sebagai konfigurasi umum; metode/identitas eduroam dapat berbeda menurut institusi, jadi cocokkan petunjuk IT kampus.
4. Broker gateway default adalah `test.mosquitto.org:1883`. Untuk mengubahnya dari serial, gunakan `mqtt set <host> <port> [username] [password]`, lalu `mqtt save`; pengaturan juga tersedia di bagian “Konfigurasi broker gateway” pada GUI web saat serial gateway tersambung. Nilai build dapat dioverride melalui `include/config.local.h`.
5. Set `ESPNOW_CHANNEL` sama dengan kanal access point gateway. Kanal AP harus diverifikasi di lokasi; jangan menganggap semua router tetap di kanal 1.
6. Build dan flash gateway terlebih dahulu, lalu build/flash node.
7. Buka serial monitor untuk melihat status Wi-Fi, kanal gateway, MQTT, sampel, serta statistik paket. Gateway mencoba sinkronisasi waktu NTP melalui Wi-Fi (UTC); ESP32 menyimpan waktu sistem selama deep sleep melalui RTC internal dan mengoreksinya lagi saat NTP tersedia. Jika daya benar-benar terputus, waktu perlu disinkronkan ulang. Setiap node/relay harus menggunakan kanal gateway yang tercetak.

`config.local.h` diabaikan Git agar kredensial tidak ikut terunggah. Default broker MQTT gateway adalah `test.mosquitto.org:1883` tanpa autentikasi, hanya untuk pengujian publik—jangan kirim data sensitif. Ubah dari serial (`mqtt set <host> <port> [username] [password]`, `mqtt save`) atau kolom broker web GUI. Perubahan serial tersimpan di NVS pada gateway; pengaturan web broker disimpan lokal per-browser. Port 1883 tidak terenkripsi; gateway MQTT TLS belum diimplementasikan.

### Catatan eduroam

- Firmware memilih mode enterprise hanya jika `WIFI_SSID` **persis** `eduroam`. SSID lain tetap memakai WPA2-PSK.
- ESP32-C3 hanya mendukung Wi-Fi 2,4 GHz. Pastikan lokasi uji menyediakan eduroam 2,4 GHz.
- Konfigurasi bawaan memakai PEAP/MSCHAPv2. Institusi yang hanya mengizinkan EAP-TLS (sertifikat klien), WPA3-Enterprise, atau onboarding lewat portal belum didukung oleh kode ini.
- Tanpa `EDUROAM_CA_CERT` atau CA yang disimpan melalui `wifi ca`, verifikasi sertifikat server tidak dikonfigurasi secara eksplisit dan sebagian jaringan dapat menolak atau tidak memberi jaminan identitas server. Isi CA resmi dari IT kampus bila diwajibkan. Jangan menganggap koneksi sudah aman hanya karena berhasil terhubung.
- Banyak jaringan kampus membatasi port masuk/keluar atau menerapkan isolasi klien. Pastikan broker MQTT yang dituju memang dapat dijangkau dari jaringan eduroam; broker lokal di laptop/PC lain sering tidak bisa diakses.
- Gateway mengikuti kanal AP tempat ia terasosiasi. Saat gateway berpindah AP eduroam, kanal dapat berubah dan node ESP-NOW yang memakai `ESPNOW_CHANNEL` tetap tidak akan ikut pindah. Pastikan gateway tetap pada AP/kanal yang sama, atau gunakan AP khusus untuk tautan ESP-NOW. Perubahan kanal hanya dicatat di log, tidak diperbaiki otomatis.
- Kredensial eduroam bersifat personal; konfigurasi ini bukan pengganti kebijakan keamanan atau pendaftaran perangkat yang ditetapkan pihak kampus.

## Sambungan sensor (rancangan awal, cocokkan dengan board dan modul aktual)

Pin default ESP32-C3 SuperMini: GPIO8 SDA / GPIO9 SCL untuk I2C; GPIO0 NTC; GPIO1 anemometer analog; GPIO3 pemantauan baterai opsional. Untuk ESP32 klasik default-nya GPIO21 SDA / GPIO22 SCL; GPIO32 NTC; GPIO33 anemometer; GPIO34 pemantauan baterai. ESP32 klasik hanya memakai pin **ADC1** (GPIO32–39) karena ADC2 tidak dapat dibaca saat Wi-Fi/ESP-NOW aktif; GPIO1/GPIO3 pada board klasik adalah UART0, bukan ADC. Pastikan pin tersebut tidak dipakai fungsi board lain. BH1750 umumnya memakai alamat 0x23 (ADDR ke GND) atau 0x5C (ADDR ke VCC). LED status ESP32-C3 ditetapkan pada GPIO7 agar tidak bentrok dengan I2C default GPIO8/9 atau ADC default GPIO0/1/3; pada banyak ESP32-C3 SuperMini GPIO7 terhubung ke LED onboard, tetapi verifikasi revisi board. Jika board tidak memiliki LED di pin itu, hubungkan LED eksternal dengan resistor seri. ESP32 klasik memakai GPIO2 sebagai status LED secara default. LED berkedip saat paket ESP-NOW diterima dan dua kali saat node mengirim/meneruskan paket.

### BH1750

Firmware membaca BH1750 melalui I2C pada mode continuous high-resolution dan menghitung lux sebagai raw / 1.2. Ini implementasi dasar mengikuti skala sensor, bukan kalibrasi sistem. Catat kondisi pemasangan, orientasi, rentang, dan bandingkan dengan alat rujukan bila tersedia.

### Anemometer dinamo kecil

Input analog GPIO1 hanya merekam tegangan yang sudah melalui rangkaian divider/proteksi. Firmware **belum menghitung kecepatan angin** karena hubungan putaran/tegangan terhadap m/s memerlukan karakterisasi dan kalibrasi anemometer aktual. Diagram menyebut zener 3.3 V; pilih zener dan resistor seri sesuai arus, impedansi sumber, batas ADC, polaritas/transien, serta disipasi. Jangan menghubungkan dinamo langsung ke pin ESP32. Pastikan tegangan pada ADC selalu dalam rentang aman.

### NTC 10 kΩ

Asumsi pembagi: 3V3 → resistor tetap 10 kΩ → titik ADC → NTC 10 kΩ → GND. Firmware memakai beta 3950 K dan nominal 25 °C; keduanya harus diganti bila datasheet NTC berbeda. Gunakan resistor pull-up/pembagi yang sesuai dan pemasangan termal yang aman. NTC ini hanya untuk pemantauan, bukan pengganti sensor/proteksi charger atau BMS.

### Tegangan baterai (opsional)

GPIO3 membaca melalui pembagi resistif dengan faktor default 2.0. Pastikan rasio aktual, toleransi resistor, batas ADC, dan tegangan maksimum sesuai. Jika tidak memasang divider, set `FEATURE_BATTERY_SENSE 0`. Jangan hubungkan baterai langsung ke pin ADC.

**Keselamatan:** Jangan merakit/mengukur PHB atau sumber tegangan berbahaya dengan prototipe ini. Gunakan sel 18650 terlindungi, charger Li-ion sesuai kimia dan konfigurasi sel, proteksi arus lebih/overcharge/overdischarge, pembatasan arus, isolasi, penahan sel, dan pengawasan yang sesuai. Verifikasi rancangan catu oleh pembimbing sebelum menghubungkan panel atau baterai. Jangan tinggalkan pengisian tanpa pengawasan.

## Sistem daya surya dan peringatan rancangan

Diagram yang diberikan mengusulkan **panel 6 V/1 W → 1N5819 → TP4056 → sel 18650 → HT7333 3.3 V → ESP32-C3**. Firmware tidak mengendalikan charger/regulator tersebut dan diagram ini masih perlu tinjauan pembimbing sebelum dirakit. TP4056 standar umumnya dirancang untuk input sumber tegangan yang relatif stabil sekitar 5 V, bukan panel surya 6 V dengan karakteristik berubah-ubah. Dioda seri mengurangi tegangan tetapi tidak mengatur/menstabilkan daya; panel 6 V langsung melalui dioda dapat tetap berada di atas batas input saat irradiance tinggi. Gunakan charger solar yang sesuai (dengan pembatasan tegangan/arus dan power-path yang tepat), cocokkan dengan spesifikasi sel, serta verifikasi rangkaian oleh pembimbing. Jangan menganggap kombinasi panel–dioda–TP4056 aman atau optimal hanya dari diagram.

HT7333 adalah LDO 3.3 V; headroom, drop-out, arus puncak radio ESP32-C3, arus diam, arus output, dan disipasi perlu diverifikasi. Kapasitas 1 W panel juga belum membuktikan energi cukup; ukur profil beban dan hasil pengisian dalam kondisi nyata. Gunakan sel 18650 terlindungi dan jangan meninggalkan pengisian tanpa pengawasan.

## Alur data / topologi

```text
BH1750 + NTC + dinamo anemometer (ADC terlindungi)
                  │
                  ▼
  Node sensor ESP32-C3 + ESP-NOW relay broadcast
         ↕ node/relay perantara (ESP32-C3)
                  │
                  ▼
    ESP32 gateway ── Wi-Fi/IP ── MQTT broker
                                      │ MQTT over WebSocket
                                      ▼
                              web/index.html
```

Image `sensor-relay` mengukur sekaligus meneruskan paket untuk mendukung node sensor dan relay perantara; image `gateway` menerima paket dan menerbitkan MQTT. Relay dilakukan dengan rebroadcast, jitter, pengurangan TTL, penambahan hop, dan cache ID+sequence untuk menekan duplikasi/broadcast storm. Ini **protokol relay demonstrasi sederhana, bukan mesh routing yang resilien**; uji topologi, kehilangan paket, interferensi, dan jangkauan diperlukan.

Paket radio berversi membawa ID node, nomor urut, uptime, hop, TTL, flags validitas, lux (centilux), suhu (centi-°C), dan tegangan (mV). Gateway melakukan validasi ukuran/header dan deduplikasi sederhana, lalu menerbitkan:

- `<prefix>/<node_id>/state` — JSON telemetri
- `<prefix>/gateway/status` — retained `online`/`offline`

Prefix default `unesa/ussc/sensor`. Contoh JSON:

```json
{"schema":1,"node_id":"node-a1b2c3","sequence":3,"lux_lx":245.5,"battery_temp_c":31.2,"battery_v":3.91,"hop":0,"rssi_dbm":-52,"gateway_rx_epoch":1795171200}
```

**Relay/multi-hop dasar sudah diprogram**, tetapi ini bukan mesh routing otomatis atau protokol yang teruji ketahanan. Firmware menambahkan deduplikasi, jitter acak, hop, dan TTL. Pengiriman broadcast tetap best-effort; topologi, jumlah node, kehilangan paket, duplikasi, interferensi, dan jangkauan harus diuji sebelum menarik kesimpulan.

ESP-NOW broadcast ini tidak dienkripsi. Jangan mengirim data sensitif; penambahan peer terenkripsi dan manajemen kunci dibutuhkan sebelum penggunaan lebih luas. Gateway yang terhubung ke AP menggunakan kanal Wi-Fi AP, jadi semua node dan relay harus diprogram dengan kanal tersebut (lihat log gateway), bukan mengandalkan `ESPNOW_CHANNEL` jika nilainya berbeda. Dashboard web memerlukan broker yang menyediakan MQTT over WebSocket (`ws://` atau, disarankan, `wss://`) dan mengizinkan koneksi browser. Kredensial broker di halaman statis tidak rahasia.

## Upload firmware dan serial monitor di web

Dashboard menyediakan uploader firmware, serial monitor berbasis Web Serial, pengaturan Wi-Fi gateway, dan pengaturan MQTT TCP gateway melalui serial USB. Untuk GUI MQTT dashboard gunakan URL WebSocket tersendiri; default publik `wss://test.mosquitto.org:8081` mungkin memerlukan prefix topik yang cocok (`unesa/ussc/sensor`). Buka situs melalui HTTPS (GitHub Pages) di Chrome/Edge desktop, pilih firmware sesuai board, sambungkan board dengan kabel USB data, lalu pilih port dan setujui izin browser. Uploader memeriksa jenis chip sebelum menghapus flash; proses flash menghapus firmware dan data flash lama. Jangan cabut USB saat proses berjalan.

Serial monitor memakai xterm.js pada 115200 baud secara default. Auto-reconnect mencoba membuka kembali port yang sudah diberi izin jika board sempat lepas; browser tetap dapat meminta izin/aksi pengguna setelah perangkat dicabut. Port serial hanya bisa dipegang satu fitur/tab pada satu waktu, jadi tutup sesi serial monitor sebelum upload firmware. MQTT juga memakai auto-reconnect.

Web Serial memerlukan browser yang mendukung API tersebut dan secure context (HTTPS atau localhost); Firefox/Safari belum didukung secara universal. Library esptool-js dan xterm.js dimuat dari jsDelivr.

## Web dashboard lokal

Untuk mencoba tampilan statis, jalankan server dari folder proyek:

```sh
python3 -m http.server 8000 --directory web
```

Buka `http://localhost:8000`. Isi URL WebSocket broker dan prefix topik, lalu hubungkan. Preferensinya disimpan lokal pada browser. Web dashboard tidak menyediakan backend, autentikasi, penyimpanan historis, atau broker; semua itu memerlukan infrastruktur terpisah. MQTT.js dimuat dari jsDelivr versi tetap.

## Publikasi GitHub Pages

Workflow `.github/workflows/pages.yml` membangun firmware sensor/relay dan gateway, membuat paket unduhan firmware dan manifest, lalu menerbitkan dashboard statis melalui GitHub Pages. Setelah proyek berada di repositori GitHub, aktifkan Pages dengan sumber **GitHub Actions**. Situs statis tetap perlu broker MQTT WebSocket yang dapat dijangkau browser.

## Batasan dan rencana uji

- Dukungan pembacaan yang diprogram: BH1750 I2C, NTC, tegangan ADC anemometer, dan divider tegangan baterai. Nilai tegangan anemometer belum menjadi kecepatan angin.
- NTC diasumsikan beta 3950 K; cocokkan dengan datasheet aktual.
- Tegangan dinamo anemometer wajib dikondisikan/proteksi dan dibatasi sebelum pin ADC; uji tegangan maksimum dan transien secara aman.
- Baterai dan charger solar memerlukan tinjauan listrik/termal terpisah; jangan gunakan susunan TP4056 pada panel 6 V tanpa memastikan batas input dan kebutuhan charger solar.
- Estimasi ketahanan baterai belum dapat dinyatakan sebelum arus sistem, siklus sampling, panel, kapasitas sel, dan kondisi pencahayaan diuji.
- Deep sleep dimatikan secara default; relay perlu tetap aktif mendengarkan. Mengaktifkan sleep pada image sensor mematikan relay dan perlu konfigurasi peran terpisah sebelum uji hemat energi.
- Gateway queue bersifat RAM/best-effort; pesan dapat hilang jika broker putus atau queue penuh. Mekanisme ini belum memiliki store-and-forward.
- Relay sederhana tidak menjamin pengiriman, routing optimal, atau pemulihan rute; TTL dan deduplikasi hanya batas dasar.
- Target kinerja numerik ditetapkan setelah komponen, lingkungan, jumlah node, dan metode uji disepakati.
- Belum ada klaim uji perangkat keras atau penerapan lapangan.

Rencana uji: pembacaan BH1750 dibanding alat rujukan, karakterisasi ADC anemometer dan kalibrasi tegangan-ke-kecepatan, verifikasi pembagi NTC, uji node langsung dan relay dengan hitung paket hilang/duplikat serta latensi, keberhasilan publish MQTT, pengukuran arus dan energi catu, tinjauan keselamatan, dokumentasi, dan validasi pembimbing/mitra.
