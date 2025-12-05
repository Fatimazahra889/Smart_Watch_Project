# 📍 Smart Health Watch (Bracelet Connecté)

> **Master's Project - IoT & Embedded Systems**  
> *Université Ibn Zohr - Faculty of Sciences, Agadir*

![Project Status](https://img.shields.io/badge/Status-Completed-success)
![Python](https://img.shields.io/badge/Backend-FastAPI-009688?style=flat&logo=fastapi)
![React Native](https://img.shields.io/badge/Mobile-React%20Native-61DAFB?style=flat&logo=react)
![Azure](https://img.shields.io/badge/Cloud-Microsoft%20Azure-0078D4?style=flat&logo=microsoftazure)
![IoT](https://img.shields.io/badge/Hardware-ESP32-red?style=flat&logo=espressif)

## 📖 Overview

The **Smart Health Watch** project is a complete end-to-end IoT solution designed to monitor physiological data in real-time. It consists of a wearable device (ESP32-based bracelet), a mobile application for visualization, and a secure Cloud backend for data storage and analysis.

**Key capabilities:**
*   ❤️ **Heart Rate & SpO2 Monitoring:** Real-time tracking using MAX30102.
*   thermometer **Body Temperature:** Non-contact measurement via MLX90614.
*   🚨 **Intelligent Alerts:** Detects anomalies (Tachycardia, Hypoxia, Fever) and triggers alerts.
*   📱 **BLE Communication:** Low-energy transmission from the watch to the mobile app.
*   ☁️ **Cloud Integration:** Secure data storage on Azure via a REST API.

---

## 🏗️ System Architecture

The project follows a **3-tier architecture**:

1.  **Hardware Layer (IoT):** Collects vital signs and displays them on an OLED screen.
2.  **Mobile Layer (Gateway):** Receives data via Bluetooth Low Energy (BLE) and forwards it to the cloud.
3.  **Cloud Layer (Backend):** A containerized FastAPI application hosted on Azure App Service with a PostgreSQL database.

*(See `documentation/` folder for the detailed architecture diagrams).*

---

## 🛠️ Technology Stack

### 1. IoT & Hardware (Firmware)
*   **Microcontroller:** ESP32-C3 Super Mini (Chosen for native BLE 5.0).
*   **Sensors:** 
    *   `MAX30102` (Heart Rate & SpO2)
    *   `MLX90614` (IR Temperature)
    *   `MPU6050` (Accelerometer/Gyroscope)
*   **Display:** 0.96" OLED Screen (I2C).
*   **Protocol:** Bluetooth Low Energy (BLE).

### 2. Mobile Application
*   **Framework:** React Native.
*   **Libraries:** Axios (API requests), React Native BLE PLX (Bluetooth).
*   **Features:** Real-time dashboard, User authentication (JWT), Historical charts.

### 3. Backend & Cloud
*   **Language:** Python 3.10.
*   **Framework:** FastAPI (Async/Await).
*   **Database:** PostgreSQL (Azure Database for PostgreSQL).
*   **ORM:** SQLAlchemy.
*   **Security:** OAuth2 with JWT (Users) & API Key (IoT Device).
*   **DevOps:** Docker, Azure Container Registry (ACR), Azure App Service.

---

## 🚀 Features

### ✅ Real-time Monitoring
Visualization of BPM, Oxygen saturation (%), and Skin Temperature on both the OLED screen and the Mobile App.

### ✅ Alert System
Automatic detection of critical health states:
*   **Bradycardia / Tachycardia:** BPM < 60 or > 100.
*   **Hypoxia:** SpO2 < 95%.
*   **Fever:** Temp > 37.5°C.

### ✅ Security
*   **User Data:** Protected via bcrypt password hashing and JWT tokens.
*   **Device Data:** Authenticated via `X-API-KEY` headers.
*   **Environment:** Sensitive keys managed via Azure Environment Variables (Twelve-Factor App methodology).

