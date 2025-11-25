# Struktur Modul 3RPS Modular

## Overview

Program telah dipecah menjadi modul-modul terpisah untuk memudahkan maintenance dan pengembangan.

## Modul-modul

### 1. Config (`include/Config.h`)
**Tanggung Jawab**: Menyimpan semua konstanta dan konfigurasi sistem
- Konstanta trajectory (jumlah titik, index)
- Timing configuration
- Modbus register addresses
- Serial port configuration

### 2. StateMachine (`include/StateMachine.h`)
**Tanggung Jawab**: Definisi state machine
- Enum SystemState
- State definitions

### 3. TrajectoryManager (`include/TrajectoryManager.h`, `src/TrajectoryManager.cpp`)
**Tanggung Jawab**: Management trajectory data
- Loading trajectory dari file
- Switching antar trajectory
- Akses data trajectory aktif
- Konfigurasi trajectory (index, range, dll)

**Fitur**:
- Load semua trajectory (1, 2, 3)
- Switch trajectory aktif
- Akses data posisi, kecepatan, force
- Akses konfigurasi graph dan gait

### 4. SerialHandler (`include/SerialHandler.h`, `src/SerialHandler.cpp`)
**Tanggung Jawab**: Komunikasi serial dengan Arduino
- Initialize serial port
- Send commands
- Read data
- Parse feedback

**Fitur**:
- Initialize dengan port dan baud rate
- Send command ke Arduino
- Read data dari Arduino
- Parse value dari feedback string

### 5. ModbusHandler (`include/ModbusHandler.h`, `src/ModbusHandler.cpp`)
**Tanggung Jawab**: Komunikasi Modbus dengan HMI
- Initialize Modbus server
- Handle connections
- Read/write registers
- Float operations

**Fitur**:
- Initialize Modbus TCP server
- Accept connections
- Read/write registers
- Read/write float values
- Handle connection errors

### 6. GraphManager (`include/GraphManager.h`, `src/GraphManager.cpp`)
**Tanggung Jawab**: Management grafik untuk HMI
- Load trajectory data ke Modbus
- Update animation
- Reset graph data
- Clear channel data

**Fitur**:
- Load trajectory graph ke CH0
- Update real-time animation di CH1
- Reset graph data
- Clear animation channel

### 7. ControlHandler (`include/ControlHandler.h`, `src/ControlHandler.cpp`)
**Tanggung Jawab**: Logika kontrol sistem
- Manual control
- Calibration
- Start rehab cycle
- Threshold management
- Trajectory selection
- Arduino feedback processing
- Retreat control
- Auto rehab processing

**Fitur**:
- Handle manual commands
- Handle calibration
- Start rehabilitation cycle
- Update thresholds
- Handle trajectory selection
- Process Arduino feedback
- Manage retreat sequence
- Process auto rehabilitation

### 8. StateHandlers (`include/StateHandlers.h`, `src/StateHandlers.cpp`)
**Tanggung Jawab**: State machine handlers
- Handle IDLE state
- Handle RESETTING state
- Handle AUTO_RETREAT state
- Handle POST_REHAB_DELAY state

**Fitur**:
- State transition logic
- State-specific operations
- Cycle management

### 9. Main (`main.cpp`)
**Tanggung Jawab**: Entry point dan main loop
- Initialize semua modul
- Main loop
- State machine coordination
- Cleanup

## Alur Data

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

## Dependencies

```
Main
├── Config
├── StateMachine
├── TrajectoryManager
├── SerialHandler
├── ModbusHandler
├── GraphManager (depends on ModbusHandler, TrajectoryManager)
├── ControlHandler (depends on all above)
└── StateHandlers (depends on ControlHandler, ModbusHandler, SerialHandler)
```

## Keuntungan Struktur Modular

1. **Separation of Concerns**: Setiap modul punya tanggung jawab jelas
2. **Maintainability**: Mudah menemukan dan memperbaiki bug
3. **Testability**: Setiap modul bisa di-test secara independen
4. **Reusability**: Modul bisa digunakan di proyek lain
5. **Scalability**: Mudah menambah fitur baru
6. **Readability**: Kode lebih mudah dibaca dan dipahami

## Catatan Implementasi

- Beberapa fungsi masih perlu disesuaikan untuk fully modular
- StateHandlers perlu akses ke ControlHandler untuk cycle management
- GraphManager perlu akses ke TrajectoryManager untuk data
- Semua modul menggunakan references untuk menghindari copy overhead

