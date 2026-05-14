<img width="904" height="1280" alt="WhatsApp Image 2026-05-14 at 12 11 40 PM" src="https://github.com/user-attachments/assets/ae71c4b3-d7bf-4b6a-a6a8-49daaaa25881" />

# 🤖 ASSISTIVE-GESTURE-CONTROLLED-ROBOTIC-ARM

> A ROS2-based 6-DOF robotic arm that performs real-time gesture-controlled movements via UDP communication.

---

## 📌 Overview

This project enables **gesture-based robotic control** using:
- **ROS2 modular nodes**
- **UDP communication**
- **PCA9685 servo driver**
- **MG996R & MG90S servo motors**

---

## ✨ Features

- ✅ **Real-time gesture control via UDP**
- ✅ **Modular ROS2 architecture**
- ✅ **Servo smoothing algorithm**
- ✅ **Safe angle constraints**
- ✅ **6 DOF joint control**

---

## 🧠 ROS2 Nodes

### 🔹 `udp_receiver`
**Purpose:** Receives gesture commands over UDP.

- Listens on UDP port
- Parses incoming data
- Publishes to ROS2 topic

---

### 🔹 `servo_controller`
**Purpose:** Controls servo motors.

- Subscribes to gesture topic
- Applies smoothing
- Enforces angle limits
- Sends PWM signals via PCA9685

---

## 🏗 System Architecture

```text
Gesture Input
      ↓ (UDP)
udp_receiver
      ↓ (ROS2 Topic)
servo_controller
      ↓ (I2C)
PCA9685 Driver
      ↓
Servo Motors
