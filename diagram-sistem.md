# Diagram Sistem Node Sensor Surya

```mermaid
graph TD
    subgraph Power_Management [Sistem Daya Tenaga Surya - rancangan yang perlu direview]
        SP[Panel surya 6 V 1 W] -->|Input yang perlu regulasi/charger solar sesuai| CH[Charger Li-ion solar yang sesuai]
        CH <-->|Charge / discharge| BAT[Baterai 18650 3.7 V dengan proteksi]
        BAT -->|Tegangan sesuai kebutuhan dan arus puncak radio| REG[Regulator 3.3 V]
    end

    subgraph Node_ESP32C3 [Node Sensor ESP32-C3]
        REG -->|3.3 V stabil| MCU[ESP32-C3 SuperMini]
        LUX[BH1750] -->|I2C SDA GPIO8 / SCL GPIO9| MCU
        TEMP[NTC 10 kΩ suhu baterai] -->|Pembagi tegangan / ADC GPIO0| MCU
        DYN[Dinamo kecil anemometer] --> VDIV[Pembagi tegangan + proteksi/clamp hasil review]
        VDIV -->|Tegangan analog ADC GPIO1| MCU
        BAT --> BMON[Pembagi tegangan baterai opsional]
        BMON -->|ADC GPIO3| MCU
    end

    subgraph ESP_NOW_Mesh [Prototipe relay ESP-NOW - bukan mesh routing otomatis]
        MCU <-->|ESP-NOW channel sama| NODE2[Node / relay ESP32-C3 #2]
        MCU <-->|ESP-NOW channel sama| NODE3[Node / relay ESP32-C3 #3]
        NODE2 <-->|Rebroadcast terbatas TTL + dedup| NODE3
        NODE3 -->|Paket dengan hop/TTL| GW[Gateway ESP32]
    end

    GW -->|Wi-Fi| MQTT[Broker MQTT]
    MQTT -->|MQTT over WebSocket| WEB[Dashboard web statis]
```

## Peringatan rancangan daya

Diagram awal mengusulkan panel 6 V → dioda 1N5819 → TP4056 → sel Li-ion → HT7333. Jangan rakit susunan tersebut tanpa tinjauan: TP4056 standar ditujukan untuk sumber input yang sesuai batas tegangan dan relatif stabil; panel surya 6 V plus dioda bukan pengatur tegangan atau pengendali titik daya. Tegangan panel dapat melampaui rating input yang aman dan karakteristik panel berubah terhadap cahaya. Gunakan charger solar yang memang sesuai, verifikasi batas input/current limit/kimia sel dan proteksi, lalu tinjau regulator untuk arus puncak ESP32. Semua perakitan baterai dilakukan di bawah supervisi yang sesuai.

## Sinyal anemometer

Firmware menyediakan masukan ADC dan menerbitkan `wind_input_v` saja. Nilai tersebut bukan kecepatan angin. Dibutuhkan datasheet/karakterisasi dinamo, validasi divider-zener/penjepit, dan kalibrasi terhadap alat rujukan untuk menghasilkan satuan m/s.
