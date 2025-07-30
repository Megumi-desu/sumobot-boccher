# 🥊 SumoBot-Boccher  
**Arduino IDE Code for a 2-Wheel ESP32 Sumo Robot – Full RC Control**

> Ready-to-win firmware for TechnoCorner 2025!  
> Built around **ESP32 + L298N Dual H-Bridge + 2.4 GHz RC Rx/Tx**

---

## 📁 Repository Structure
```
sumobot-boccher/
├── README.md               ← You are here
├── boccher_v3_RC/          ← Final release v3 (RC)
│   └── boccher_v3_RC.ino   
└── extras/                 ← CAD, tuning logs, etc
```

---

## 🧰 Hardware Stack
| Part | Model / Pinout |
|------|----------------|
| Microcontroller | **ESP32 DOIT DevKit v1** |
| Motor Driver | **L298N Dual H-Bridge** |
| Motors | **JGA25-370** 12 V 500 rpm (3 kg robot) |
| RC System | **4-CH 2.4 GHz (FlySky TX4 clone)** |
| Battery | 3 S Li-ion 11.1 V 2200 mAh |
| Chassis | 3 mm aluminium plate, 3D-printed wheels |

---

## 🔌 Quick Wiring
| L298N | ESP32 Pin | Function |
|-------|-----------|----------|
| IN1   | 32        | Right motor direction 1 |
| IN2   | 33        | Right motor direction 2 |
| ENA   | 25        | Right PWM (Channel 0) |
| IN3   | 27        | Left motor direction 1 |
| IN4   | 14        | Left motor direction 2 |
| ENB   | 12        | Left PWM (Channel 1) |

| RC Rx | ESP32 Pin | Channel |
|-------|-----------|---------|
| CH1   | 5         | **Steering** |
| CH2   | 17        | **Throttle** |
| CH3   | 16        | **Rotate mode switch** |
| CH4   | 4         | **Emergency stop** |

> ⚡ Power the L298N logic from **5 V** rail, **NOT** 3.3 V.

---

## 🚀 Flash & Run
1. Install **Arduino IDE 2.x** with ESP32 board package (`esp32 by Espressif`).
2. Clone or download this repo.
3. Open `boccher_v2_RC/boccher_v3_RC.ino`.
4. Select:
   - Board: `ESP32 Dev Module`
   - Upload Speed: `921600`
   - Port: your COMx / ttyUSBx
5. Upload.
6. Power on the robot **then** the RC transmitter.
7. Open Serial Monitor @ 115 200 baud to see live telemetry.

---

## 🎮 RC Control Logic
| Stick / Switch | Behavior |
|----------------|----------|
| **Left stick (CH2)** | Throttle forward / backward |
| **Right stick (CH1)** | Steering (tank mixing) |
| **3-pos switch (CH3)** | Up = **Rotate mode**<br>Middle = Normal drive<br>Down = N/A |
| **Momentary switch (CH4)** | Press = **Emergency stop** |

---

## 🛠 Tuning & Calibration
Calibrate your own sticks once:
```cpp
calibrateRC();   // Uncomment line 367, upload, follow Serial prompts
```
The sketch will print new `RC_CHx_CENTER` values – paste them back into the defines.

---

## 🧪 Built-in Test Routines
```cpp
testMotors();    // Line 350 → Full L298N motor test forward / backward
```
Great first-run sanity check.

---

## 🖥 Serial Debug Output (500 ms)
```
========== DEBUG INFO ==========
RC Connected: YES
Current Mode: NORMAL_DRIVE
--- RC Channels (Raw) ---
CH1 (Steering): 1670 | Mapped: 80
CH2 (Throttle): 1350 | Mapped: -110
CH3 (Rotate): 1500
CH4 (E-Stop): 1900
--- Motor Speeds ---
Right Motor: -190
Left Motor: -30
...
```

---

## 🏆 Competition Tips
- Set `speedMultiplier` (line 50) to 70–80 % during practice, 100 % on fight day.  
- Use **CH4 emergency stop** for instant failsafe.  
- Enable **debugMode = false** before matches to reduce latency.

---

## 🤝 Contributions & Forks
Pull requests welcome!  
🪲 Issues → GitHub Issues tab  
📸 Tag us on Instagram `#bocchersumo`

---

## 📄 License
MIT – do whatever you want, just mention the team 😉

---

**Built with ❤️ by Boccher Team – TechnoCorner 2025**  
> “*The ring is ours, the code is open.*”
