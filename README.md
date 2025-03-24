# 🚀 Autonomous Inspection Robot

### 🤖 Overview
This project is built using **ESP32 Dev Modules, ESP32-CAM, and LoRa RA-02 modules**. It features:  
✅ **Wireless communication** via LoRa between GUI and Robot.  
✅ **ESP32-CAM for real-time video streaming.**  
✅ **Python-based GUI for manual control and object detection using OpenCV.**  

---

## 🛠 Hardware Used  
- 🟢 **ESP32 Dev Module** (×2)  
- 📷 **ESP32-CAM**  
- 📡 **LoRa RA-02 Module** (×2)  
- 🔋 **Motor Driver & Motors**  

---

## 🖥 Software & Dependencies  
- 🐍 **Python** (GUI & OpenCV for object detection)  
- 🏎 **Arduino IDE** (For ESP32 coding)  
- 🎯 **YOLOv3** for Object Recognition  

---

## 📂 Code Upload Instructions  

| Module | Code to Upload | Functionality |
|--------|--------------|--------------|
| **ESP32 Dev Module (Sender)** | `espCarSend.ino` | Sends control signals via LoRa |
| **ESP32 Dev Module (Receiver - Main Robot)** | `carResiev02.ino` | Controls the robot based on received signals |
| **ESP32-CAM** | `CameraWebServer.ino` | Streams live video to GUI |

---

## 🎮 GUI Control & Object Detection  
- The **GUI** is coded in **Python** and allows:  
  ✔️ **Manual car control**  
  ✔️ **Live camera feedback**  
  ✔️ **Object detection & recognition** using **OpenCV + YOLOv3**  

### 🔗 Download YOLOv3 Files  
👉 **[Get YOLOv3-320 Weights & Config](https://pjreddie.com/darknet/yolo/)**  
📌 Place them in the **same directory** as `GUI08.py`  

---

## 🛠 How to Run the GUI  
```bash
python GUI08.py
