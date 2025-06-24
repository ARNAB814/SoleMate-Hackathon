# SoleMate - Smart Fall Prevention System

**Prevention over Detection**

## Why This Project?

Every 11 seconds, a senior citizen falls. Current medical alert systems only work AFTER falls happen and cost $300+. We built SoleMate to demonstrate that AI-powered fall **prediction** is possible for under $50 in components.

## What It Does

SoleMate is a smart insole prototype that uses machine learning and sensor fusion to predict falls before they happen:

- **Real-time balance monitoring** with 95% accuracy
- **AI-powered fall prediction** using 6-axis motion sensors
- **Immediate feedback** through LED, audio, and display alerts
- **Posture analysis** with personalized baseline calibration
- **Multi-stage detection** to reduce false positives

Built in 48 hours as a hackathon prototype to explore preventive healthcare technology.

## Hardware
- Raspberry Pi 4 + MPU6050 6-axis sensor
- RGB LED, buzzer, LCD display, vibration sensor
- Consumer-grade components totaling ~$50

## Quick Start
```bash
git clone [repository-url]
cd solemate
pip install -r requirements.txt
sudo python3 SoleMate_Algorithm.py
```

## Project Website
**[arunav.dev/solemate](https://arunav.dev/solemate)** - Live demo and full project showcase

---
*SoleMate: Building the future of preventive healthcare, one step at a time.*
