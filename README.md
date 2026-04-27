# 🤖 Self-Balancing Robot

A two-wheeled robot that stands upright on its own using an STM32F3 microcontroller, an IMU (gyroscope + accelerometer), and a PID control loop running 200 times every second.

---

## 📖 What Is This?

Imagine balancing a broomstick on the palm of your hand. You can't just hold your hand still — the broom would fall over instantly. You have to constantly move your hand to keep it upright. If the broom leans left, you move your hand left to catch it.

This robot does the same thing:
- The **broomstick** is the robot's body
- The **hand** is two wheels driven by motors
- The **eyes that see the lean** are motion sensors
- The **brain** is an STM32F3 microcontroller making decisions every 5 milliseconds

When you turn it on, the robot wobbles for a moment, then settles into balance — making tiny corrections so fast and so often that it appears to be standing perfectly still.

---

## 🎯 Why This Project Matters

Self-balancing is the foundational concept behind Segways, SpaceX rockets landing themselves vertically, humanoid robots like Boston Dynamics' Atlas, and drones holding position in mid-air.

If you understand how this little robot stays upright, you understand the same control theory used in spacecraft and surgical robots. The principles are identical — only the scale changes.

---

## 🧠 How It Actually Works

The control loop runs at **200 Hz** (every 5 ms) and follows three steps:

### Step 1: Sense — "Am I leaning?"

Two motion sensors are mounted on the robot:
- **L3GD20 gyroscope** — measures rotational velocity (deg/s) over **SPI** at 200 Hz
- **LSM303DLHC accelerometer** — measures gravity vector over **I²C** at 100 Hz

Neither sensor is perfect alone:
- The gyroscope drifts over time because we **integrate** its rate to get angle, and small errors accumulate
- The accelerometer is noisy because every vibration registers as acceleration

**The fix is sensor fusion via a complementary filter:**

```
angle = 0.98 × (angle + gyro_rate × dt) + 0.02 × accel_angle
```

The gyroscope handles fast, short-term motion (98% weight) while the accelerometer corrects long-term drift (2% weight) by anchoring to gravity. The result is a smooth, accurate tilt angle — better than either sensor alone.

This is the same idea your inner ear uses. Your fluid-filled canals act gyroscope-like, and tiny stones called otoliths respond to gravity (accelerometer-like). Your brain fuses them without you ever thinking about it.

### Step 2: Decide — "How hard should I push back?"

This is where a **PID controller** comes in — a 100-year-old algorithm still considered the workhorse of modern control systems.

The error is calculated as:
```
error = current_angle − setpoint
```

Then three terms are computed:

- **P (Proportional) — "How far off am I right now?"**
  `P = Kp × error`
  The further the robot leans, the harder it pushes back. Like a stiff spring.

- **I (Integral) — "Have I been off for a while?"**
  `I = Ki × ∫error dt`
  Accumulates persistent errors over time. If a slight imbalance keeps the robot tilted by 1° forever, the I term grows until the motors generate enough force to fix it. Crucial for handling things like uneven weight distribution.

- **D (Derivative) — "How fast is the error changing?"**
  `D = Kd × (d_error / dt)`
  Predicts where things are going. Acts like a brake — if the robot is tipping over fast, D adds a strong counter-force *before* the angle even gets large. Without D, the robot oscillates wildly.

The final command is:
```
output = P + I + D
```

This output gets sent to the motors as a PWM signal between -999 and +999. Positive values drive forward, negative values reverse.

Our tuned values are **Kp = 30, Ki = 0.2, Kd = 1.5**. These weren't guesses — they came from hours of tuning while watching the robot wobble, fall, oscillate, and eventually balance.

### Step 3: Act — "Spin the wheels"

The PID output controls two DC motors through an **L298N H-bridge** on the motor shield. Each motor has:
- A **PWM pin** for speed (0–999, mapped to duty cycle on `TIM3` channels)
- Two **direction pins** (IN1, IN2) determining forward/reverse via the H-bridge logic

If the robot leans forward, both wheels spin forward to "drive under" the falling body. If it leans back, both reverse. The faster it's falling, the harder the wheels spin.

This loop repeats 200 times per second — faster than any human could possibly react. That's why the robot looks like it's standing still even though it's making thousands of micro-corrections every minute.

---

## 🔧 Hardware

| Component | Role |
|-----------|------|
| **STM32F3 Discovery** | 72 MHz Cortex-M4 microcontroller, runs the control code |
| **Keyestudio Motor Shield** | L298N H-bridge driver for the motors |
| **2× GM37-520 DC motors** | Geared motors for the wheels (~250 RPM at 12V) |
| **L3GD20 Gyroscope** | 245 dps full-scale, ±8.75 mdps/digit, on the Discovery board (SPI) |
| **LSM303DLHC Accelerometer** | ±2g range, on the Discovery board (I²C) |
| **3D-printed chassis** | Holds everything together |

### Pin Mapping

| Signal | STM32 Pin | Purpose |
|--------|-----------|---------|
| Left motor PWM | PC6 (TIM3_CH1) | Speed control |
| Left motor IN1/IN2 | PB12, PB13 | Direction |
| Right motor PWM | PA4 (TIM3_CH2) | Speed control |
| Right motor IN1/IN2 | PB14, PB15 | Direction |
| Gyro CS | PE3 | SPI chip select |
| UART TX/RX | PC4, PC5 | Serial logging at 115200 baud |

---

## 💻 Code Architecture

The program runs **two parallel contexts** that share data through `volatile` variables:

### The Real-Time Interrupt (`HAL_TIM_PeriodElapsedCallback`)
Triggered by `TIM2` every 5 ms with hard real-time priority. It:
1. Reads gyro (SPI) and accel (I²C)
2. Runs the complementary filter
3. Computes the PID terms
4. Updates the PWM compare registers via `__HAL_TIM_SET_COMPARE`
5. Sets flags for the main loop to process

This **must finish in under 5 ms** or the next tick gets missed. Currently it runs in roughly 1.2 ms, leaving plenty of headroom.

### The Main Loop (`main()`)
Runs whenever the ISR isn't executing. Handles non-critical work:
- UART status logging every 2 seconds
- Continuous CSV data stream at 10 Hz for the Python plotter
- Warning messages when tilt exceeds 30°

The interrupt **always wins** — even if the main loop is mid-`sprintf`, the ISR pauses it, runs the control loop, and resumes the main loop. This guarantees balance is never compromised by logging overhead.

### File Structure

```
project/
├── Core/
│   ├── Inc/                 # Header files
│   └── Src/
│       ├── main.c           # Entry point, ISR, control logic
│       ├── stm32f3xx_it.c   # Hardware interrupt handlers
│       └── ...
├── plot_sensors.py          # Real-time plotting on the host PC
├── README.md                # This file
└── Lab11_Task1.ioc          # STM32CubeMX hardware config
```

---

## 📊 Real-Time Plotting

The robot streams its internal state over UART at **115200 baud**. A Python script (`plot_sensors.py`) reads this stream and plots four panels live:

1. **Tilt angle** — filtered (smooth) overlaid on raw accel angle (noisy). You can literally see the sensor fusion working.
2. **Gyroscope rate** — angular velocity in deg/s
3. **PID terms separately** — see how P, I, and D each contribute
4. **PID total output and error**

When you push the robot, all four plots spike, then quickly return to normal as the controller fights back. It's a beautiful visual of feedback control in action.

```bash
pip install pyserial matplotlib numpy
python3 plot_sensors.py
```

The script auto-detects log lines (anything starting with `[`) and prints them to the terminal while plotting only the CSV data.

---

## 🚀 Getting It Running

### Prerequisites
- **STM32CubeIDE** (any recent version)
- A **mini-USB cable** to flash the code (ST-LINK port)
- Optional: a second USB cable for live plotting

### Steps

1. Open the project in STM32CubeIDE
2. Connect the Discovery board via the ST-LINK USB port
3. Click **Build** (hammer icon) to compile
4. Click **Run** (green play button) to flash
5. Place the robot upright on a flat surface, holding it gently
6. Power on the battery
7. Let go!

### Tuning Tips
| Symptom | Fix |
|---------|-----|
| Falls without resisting | Increase **Kp** |
| Wobbles violently / oscillates | Increase **Kd** or decrease **Kp** |
| Drifts slowly to one side | Increase **Ki** |
| Tips one direction more | Adjust **SETPOINT** (currently -3.5°) |

You don't need to recompile to tune — modify the `#define` values at the top of `main.c`, then re-flash.

---

## 🐛 Things We Learned The Hard Way

- **Sensors lie.** The accelerometer reads ±0.5° of noise even when sitting still. Filtering isn't optional.
- **Motors aren't symmetric.** Even from the same batch, DC motors vary by 10–20% in strength. The robot will drift unless you compensate with per-motor PWM trim.
- **Loop timing matters more than algorithm complexity.** A simple PID running at 200 Hz beats a sophisticated controller running at 50 Hz. Always.
- **The setpoint isn't always zero.** The "balanced" angle depends on the exact center of gravity, sensor mounting orientation, and battery position. Ours tuned to **-3.5°** to actually stand straight.
- **Debug printing can break real-time systems.** Sending too much data over UART eats CPU time the control loop needs. We throttle output to 10 Hz to keep the 200 Hz loop healthy.
- **Integral windup is real.** Without anti-windup clamping (`INTEGRAL_MAX = 500`), the I term saturates during a fall and the robot lurches violently when picked back up.
