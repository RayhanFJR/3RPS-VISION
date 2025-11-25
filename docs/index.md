# Dokumentasi Sistem Kontrol Rehabilitasi 3RPS Modular

## Selamat Datang

Dokumentasi ini menjelaskan sistem kontrol rehabilitasi 3RPS versi modular. Sistem ini dirancang untuk mengontrol platform rehabilitasi 3RPS dengan komunikasi Modbus TCP untuk HMI dan komunikasi serial untuk Arduino.

## Daftar Isi

1. [Instalasi](instalasi.md) - Panduan instalasi dan setup sistem
2. [Penggunaan](penggunaan.md) - Panduan penggunaan sistem
3. [Struktur Modul](STRUCTURE.md) - Dokumentasi struktur modul dan arsitektur

## Overview Sistem

Sistem Kontrol Rehabilitasi 3RPS Modular adalah aplikasi C++ yang mengontrol platform rehabilitasi dengan fitur-fitur berikut:

### Fitur Utama

- **Multi-Trajectory Support**: Mendukung 3 trajectory berbeda yang dapat dipilih secara dinamis
- **Modbus TCP Communication**: Komunikasi dengan HMI melalui Modbus TCP
- **Serial Communication**: Komunikasi dengan Arduino untuk kontrol motor
- **State Machine**: Sistem state machine untuk mengelola alur kontrol
- **Real-time Graph**: Update grafik real-time di HMI
- **Cycle Counter**: Penghitung siklus rehabilitasi
- **Emergency Stop**: Fitur emergency stop untuk keamanan

### Komponen Sistem

1. **TrajectoryManager**: Mengelola data trajectory (posisi, kecepatan, force)
2. **SerialHandler**: Menangani komunikasi serial dengan Arduino
3. **ModbusHandler**: Menangani komunikasi Modbus dengan HMI
4. **GraphManager**: Mengelola data grafik untuk HMI
5. **ControlHandler**: Logika kontrol sistem
6. **StateHandlers**: Handler untuk state machine

### Arsitektur

Sistem menggunakan arsitektur modular dengan pemisahan tanggung jawab yang jelas:

```
Main Loop
    ↓
ModbusHandler (receive)
    ↓
StateHandlers (process state)
    ↓
ControlHandler (control logic)
    ↓
SerialHandler (send to Arduino)
    ↓
GraphManager (update HMI)
    ↓
TrajectoryManager (get data)
```

## Persyaratan Sistem

- **OS**: Linux (disarankan Ubuntu/Debian)
- **Compiler**: GCC dengan dukungan C++11
- **Dependencies**: 
  - libmodbus
  - Boost (asio, system)
  - pthread

## Quick Start

1. Install dependencies (lihat [Instalasi](instalasi.md))
2. Compile program dengan `make`
3. Pastikan data trajectory ada di folder `../data/`
4. Jalankan program dengan `./mainhmi`

## Struktur Data

Sistem memerlukan data trajectory yang disimpan dalam struktur berikut:

```
data/
├── data_1/
│   ├── pos1.txt, pos2.txt, pos3.txt
│   ├── velo1.txt, velo2.txt, velo3.txt
│   ├── fc1.txt, fc2.txt, fc3.txt
│   └── grafik.txt
├── data_2/
│   └── (struktur sama)
└── data_3/
    └── (struktur sama)
```

## Kontak dan Dukungan

Untuk pertanyaan atau masalah, silakan lihat dokumentasi lengkap di:
- [Instalasi](instalasi.md) - Setup dan konfigurasi
- [Penggunaan](penggunaan.md) - Cara menggunakan sistem
- [Struktur Modul](STRUCTURE.md) - Detail arsitektur

