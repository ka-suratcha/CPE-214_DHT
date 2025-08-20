# CPE-214_DHT11

## 📖 Overview
This project was created as part of **CPE-214: Microprocessor System Laboratory**.  

The goal of this assignment is to use at least two components that were taught in the lab and **one new component** that was not taught in class.

I decided to build a **temperature and humidity monitoring system** using the **DHT11 sensor** (new component). The system displays the readings on the **on-board LCD** and activates **LEDs and a buzzer** when the temperature goes above a defined level.

---

## ✨ Features
- Reads **temperature** and **humidity** from the **DHT11 sensor**.
- Displays values in real time on the **STM32L152 glass LCD**:
  - using the **user button** (EXTI interrupt) to switch between temperature and humidity to be displayed on LCD.
- **Alarm system**:
  - When the temperature is higher than 20 °C:
    - Turns on **two LEDs** (PA11 and PB6).
    - Activates a **buzzer** using the DAC output.
  - Otherwise, the LEDs and buzzer are turned off.
- Uses **STM32 Low-Layer (LL) drivers** to control hardware.
- Demonstrates **interrupt handling (EXTI)** and **direct register configuration**.

---

## 🧩 Flowchart
(To be added later)


## ▶️ Example Workflow
1. **System Initialization**:
   - Configure system clock to 32 MHz.
   - Initialize LCD, GPIO, EXTI, and DAC.
2. **DHT11 Data Acquisition**:
   - Continuously read 40-bit data frame from DHT11.
   - Extract humidity and temperature values.
3. **Condition Check**:
   - If the temperature > 20 °C → LEDs + buzzer ON.  
   - Else → LEDs + buzzer OFF.
4. **Display on LCD**:
   - Default display: Temperature.  
   - Press **user button (PA0)** → switch to humidity view.  
   - Press again → switch back to temperature.

---

## 🚀 Future Improvements
- Store historical data in **EEPROM/Flash** for later analysis.
- Add **serial output (UART)** for data logging on PC.
- Support additional sensors (e.g., DHT22 for better accuracy).
- Add **humidity warnings** to the list of buzzer alerts.

---

## 🛠️ Hardware Used
- STM32L152 Discovery Board (with on-board glass LCD).
- DHT11 Sensor.
- external LED (PA11).
- DAC-controlled buzzer.

### 📍 Pin Allocation
<img width="2162" height="667" alt="PIN_CPE-214_DHT11" src="https://github.com/user-attachments/assets/332c57e5-f430-42d3-969f-9643c182b8ed" />
