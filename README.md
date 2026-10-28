# Obstacle Avoider – NexSync Design Challenge (UG2–UG4 Elimination Round)

A round, centre-axle obstacle-avoiding robot designed end to end: Fusion 360 chassis, electronics schematic, BOM, conceptual 3D PCB, firmware logic, simulation and integration.

**Team:** Igris  ·  **Institute:** IIIT Sri City  ·  **Date:** 28 Oct 2026

---

## 1. What it does

The robot drives forward, slows when an obstacle comes within 50 cm, stops at 25 cm, turns toward the clearer side using a gyro, and resumes only when the path ahead is clear beyond 40 cm. A floating bumper and two cliff sensors act as safety backups for objects and edges the ultrasonic sensors cannot see.

| Required scenario (brief) | How the robot handles it |
| --- | --- |
| 1. No obstacle → move forward | FORWARD state at cruise speed |
| 2. Obstacle ahead → stop / change movement | SLOW below 50 cm, STOP at 25 cm |
| 3. Avoidance manoeuvre | Compares left vs right, spins in place toward the clearer side (gyro-measured 45° steps, 180° at a dead end) |
| 4. Path clear → resume | Resumes FORWARD once the front reads ≥ 40 cm |
| Extra: bumper hit / cliff | BACK OFF ~10 cm, then turn away |

## 2. Key design features

- **Round Ø190 mm chassis with the axle at the centre**: spins in place inside its own outline (turning sweep 105 mm vs 137 mm for a rectangular chassis).
- **3 × HC-SR04 at 0° / ±40°, tilted 8° up**: continuous left-front-right view; the tilt keeps floor echoes beyond the stop distance.
- **Floating front bumper + 2 microswitches**: catches soft or thin objects ultrasonic misses.
- **2 × TCRT5000 cliff sensors**: stop at table edges and steps.
- **MPU-6050 gyro on the turning axis + encoder motors**: accurate turns and straight driving.
- **ESP32-S3 + TB6612FNG + MP1584 buck**: enough I/O, efficient motor drive and power, full protection (reverse-polarity FET, 3 A PTC fuse, TVS, BMS).

## 3. Folder structure

```
Obstacle_Avoider/
├── README.md
├── 01_Design_Overview/      Architecture, components & algorithm (PDF)
├── 02_Chassis/
│   ├── Obstacle Avoider.f3d                 Fusion 360 design (export from Fusion)
│   ├── ObstacleAvoiderFull/                 Fusion script: builds chassis + 3D PCB
│   ├── ObstacleChassisV4/                   Fusion script: chassis only
│   ├── Chassis_V4_STEP_backup.zip           STEP files (chassis, deck, mounts, bumper)
│   └── renders/                             chassis_v4_renders.png, turning_comparison_v4.png, floor_reflection.png
├── 03_Schematic/
│   ├── Obstacle_Avoider_Schematic.pdf       4 sheets: Power, MCU+USB, Sensors, Motors+UI
│   └── Obstacle_Avoider_Connection_List.xlsx   every net, pin to pin
├── 04_BOM/
│   └── Obstacle_Avoider_BOM_Advanced_v4.xlsx   BOM, pin map, power budget, PCB spec, PCB placement, sim plan
├── 05_PCB_3D/
│   ├── ObstaclePCB3D/                       Fusion script: 3D PCB on the chassis standoffs
│   ├── PCB3D_STEP_backup.zip
│   └── pcb_layout_map.png, pcb_3d_render.png
├── 06_Simulation/
│   ├── Wokwi link: [add link]
│   ├── ObstacleAvoidDemo/                   Fusion script: 3D obstacle-avoidance animation
│   └── obstacle_avoidance_sim.mp4           top-down simulation video
├── 07_Code/                 [firmware source .ino / .cpp]
├── 08_Validation/           screenshots for test scenarios 1–8
├── 09_Integration/
│   ├── Full_Assembly_STEP.zip               chassis + 3D PCB assembly
│   └── pcb_integration_render.png
└── 10_Report/               Technical report (PDF)
```

## 4. How to open / run each part

### Fusion 360 model (chassis + 3D PCB)
1. Install Autodesk Fusion (free education licence). Set **Default design type = Hybrid**.
2. In Fusion press **Shift + S** → **+** → **Script or add-in from device** → select the `ObstacleAvoiderFull` folder.
3. If the list looks empty, click **Clear Filters** (the window opens on *Favorites*).
4. Select **ObstacleAvoiderFull** → **Run** (▶). The full robot is built with a parametric timeline. Save it.
5. Edit dimensions with **Modify → Change Parameters**.

No script access? Import `Full_Assembly_STEP.zip` instead: **Data Panel → Upload → open the .step file**.

### 3D obstacle-avoidance animation in Fusion
1. Open the saved robot design.
2. **Shift + S** → **+** → select the `ObstacleAvoidDemo` folder → **Run**.
3. Choose **No** for a live demo (or **Yes** to save PNG frames to `Desktop/ObstacleAvoidDemo`).
4. A room with walls, 3 boxes and a sound-absorbing cushion appears; the robot drives through it with its 3 sensor beams drawn live (green clear, amber slow, red stop).
5. When it finishes, the robot returns to its start. Hide or delete the "Demo Arena" component afterwards.

> This is a kinematic animation of our control logic (simulated ultrasonic ray-casting + the same state machine), not a physics simulation.

### Circuit + code simulation (Wokwi)
- Board: ESP32-S3 DevKitC-1 (same chip as the U1 module).
- 3 × HC-SR04 (drag distance sliders), MPU6050, pushbuttons on GPIO 35/36 (bumper), potentiometers on GPIO 2/4 (cliff) and GPIO 1 (battery), LEDs + logic analyzer on the motor pins.
- Link: [add Wokwi link]

### Firmware
- Arduino framework on ESP32-S3. Source: `07_Code/[file name]`.
- Upload over the board's USB-C (hold BOOT, tap RESET if needed). Serial log at 115200 baud.

## 5. Pin map (ESP32-S3)

| Function | GPIO |
| --- | --- |
| Ultrasonic TRIG / ECHO – left, front, right | 5/6, 7/15, 16/17 |
| Cliff sensors left / right (ADC) | 2 / 4 |
| Bumper left / right | 35 / 36 |
| IMU SDA / SCL / INT | 8 / 9 / 10 |
| Motor A: PWMA / AIN1 / AIN2 | 11 / 12 / 13 |
| Motor B: PWMB / BIN1 / BIN2 | 14 / 21 / 38 |
| Motor driver STBY | 18 |
| Encoders L-A / L-B / R-A / R-B | 39 / 40 / 41 / 42 |
| Battery sense (ADC) | 1 |
| Buzzer / RGB LED | 47 / 48 |
| BOOT-START button | 0 |
| USB D− / D+ | 19 / 20 |

## 6. Key numbers

| Item | Value |
| --- | --- |
| Chassis | Ø190 mm, 3 mm plate, round, axle at centre |
| Wheels | Ø65 mm, track 150 mm |
| Sensor height | 60 mm above floor, tilted 8° up |
| Turning sweep | 105 mm incl. bumper (rectangular design: 137 mm) |
| Thresholds | slow < 50 cm, stop ≤ 25 cm, resume ≥ 40 cm, side steer < 20 cm |
| PCB | 70 × 50 × 1.6 mm, 4 × M3 at 63 × 43 mm, centred on the turning axis |
| Power | 2S 18650 (7.4 V) → 5 V buck → 3.3 V LDO; est. ~3 h runtime |
| Simulation result | 45 s run, 0 collisions, all 5 states used, bumper caught the cushion |

## 7. What is verified and what is not

| Verified | Calculated / not verified |
| --- | --- |
| Avoidance logic and thresholds (simulation) | Floor-echo distance (datasheet beam angle) |
| CAD clearances, sensor line of sight, bumper travel, PCB fit | Power budget, runtime, regulator heat |
| Schematic connectivity; BOM ↔ schematic ↔ 3D PCB match | Real motor/encoder motion, print strength |

## 8. Attribution

- Chassis designed from scratch (no open-source chassis used).
- Third-party 3D models: [list any GrabCAD / SnapEDA models, or "none"].
- Tools: Autodesk Fusion 360 + Python API, Wokwi, EasyEDA, Python (CadQuery, schemdraw).
- Datasheets: ESP32-S3-WROOM-1, TB6612FNG, MP1584EN, AMS1117, MPU-6050, HC-SR04, TCRT5000, WS2812B.
- AI assistance: Claude (Anthropic) helped with design checks, scripts and documentation drafts; design decisions were reviewed by the team.
