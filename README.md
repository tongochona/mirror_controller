# View Mirror Console Controlling Using CAN Bus (Closed Loop Control System)

This project simulates a **closed-loop control system** for **view mirror adjustment** using the **CAN Bus** protocol, based on the **STM32F103CBT6** microcontroller. The system is designed to receive control commands over CAN and provide feedback on the current position of the view mirror using encoder data.

---

## 📌 Features

- 🚗 **View Mirror Position Control** using servo/DC motor
- 🔁 **Closed-loop control** via encoder feedback
- 🛠️ **PWM control** for precise movement
- 📡 **CAN Bus communication** between nodes (Master ↔ Slave)
- ⚙️ **Real-time response** to mirror adjustment commands
- 🧠 Based on **STM32F103CBT6** using STM32CubeIDE + HAL drivers

---

## 🧰 Hardware Requirements

- STM32F103CBT6 development board (e.g. Blue Pill)
- L298N Motor Driver (for DC motor) or PWM control circuit (for Servo)
- Rotary encoder (for feedback)
- Potentiometer (for adjust angle)
- CAN transceiver module (e.g. MCP2551 + SN65HVD230)
- OLED (for display)
- Power supply (5V or 12V depending on motor type)
- Jumper wires, breadboard or PCB

---

## 🔌 System Architecture


- Master node sends angle setpoints via CAN.
- Slave node controls mirror using motor + encoder.
- Encoder provides position feedback to close the loop.

---

## 🧠 Control Logic

1. **Receive desired angle** via CAN from master node.
2. **Compare** desired vs. actual angle (from encoder).
3. **Generate PWM** to motor/servo to adjust position.
4. **Loop until** error ≤ threshold.
5. **Display on screen status** via OLED.

---

## 🧪 Testing

- ✅ Test CAN communication between two STM32 boards
- ✅ Test motor movement with PWM signal
- ✅ Validate encoder feedback logic
- ✅ Implement closed-loop control with PID (optional)
- ✅ Observe stable final mirror position

---

## 📁 Folder Structure


---

## 📌 Future Improvements

- PID control for smoother adjustment
- Support for multiple mirrors (left/right)
- GUI interface for mirror simulation
- Fault detection over CAN (e.g. broken encoder)

---