# 3RPS Modular System

Versi modular dari Sistem Kontrol Rehabilitasi 3RPS dengan struktur kode yang lebih terorganisir dan mudah di-maintain.

## Struktur Proyek

```
3RPS_Modular/
├── include/              # Header files
│   ├── Config.h         # Konstanta dan konfigurasi
│   ├── StateMachine.h   # State machine definitions
│   ├── TrajectoryManager.h
│   ├── SerialHandler.h
│   ├── ModbusHandler.h
│   ├── GraphManager.h
│   ├── ControlHandler.h
│   └── StateHandlers.h
├── src/                  # Source files
│   ├── TrajectoryManager.cpp
│   ├── SerialHandler.cpp
│   ├── ModbusHandler.cpp
│   ├── GraphManager.cpp
│   ├── ControlHandler.cpp
│   └── StateHandlers.cpp
├── main.cpp              # Entry point
├── Makefile              # Build configuration
└── README.md
```

## Modul-modul

### 1. Config
Mengelola semua konstanta dan konfigurasi sistem.

### 2. TrajectoryManager
Mengelola loading dan switching trajectory data.

### 3. SerialHandler
Menangani komunikasi serial dengan Arduino.

### 4. ModbusHandler
Menangani komunikasi Modbus dengan HMI.

### 5. GraphManager
Mengelola data grafik untuk HMI.

### 6. ControlHandler
Menangani logika kontrol sistem.

### 7. StateHandlers
Menangani state machine transitions.

## Kompilasi

```bash
cd 3RPS_Modular
make
```

## Menjalankan

```bash
./main_tra_modular
```

## Keuntungan Versi Modular

1. **Kode lebih terorganisir** - Setiap modul memiliki tanggung jawab yang jelas
2. **Mudah di-maintain** - Perubahan di satu modul tidak mempengaruhi modul lain
3. **Mudah di-test** - Setiap modul dapat di-test secara independen
4. **Reusable** - Modul dapat digunakan kembali di proyek lain
5. **Scalable** - Mudah menambah fitur baru tanpa mengubah kode yang ada

## Catatan

Pastikan path ke data trajectory benar. Default: `../data/`

