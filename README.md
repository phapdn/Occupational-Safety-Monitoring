# Occupational-Safety-Monitoring
Occupational safety in dynamic construction environment.

## Project Overview

This project provides a low-cost, real-time occupational safety monitoring system for dynamic construction environments.

- **Edge-AI**: A lightweight 1D-CNN model (14 KB) runs on-device for fall detection with 96.2% accuracy, removing the need for constant cloud connectivity.
- **UWB**: Delivers high-precision (sub-30 cm) proximity warnings for worker safety.
- **LoRa**: Enables long-range, low-power data transmission for scalable site coverage.
- **Tiered Orchestration**: Decouples critical safety decisions from the cloud, ensuring deterministic latency (sub-30 ms) and privacy.

## Repository Structure

- **Dataset/**: Contains IMU sensor data for fall, walk, run, and static activities, used for model training and evaluation.
- **DL Model/**: Includes the trained 1D-CNN model (`.h5`, `.tflite`, `.h`), and documentation for AI deployment.
- **Implementation/Helmet_Esp32-s3/**: Source code and hardware designs for the smart helmet, including:
	- `Test_All_Modules/`: Test code for GPS, temperature, heart rate, battery, and UWB modules.
	- `Full_System_DualCore/`: Main firmware for dual-core ESP32-S3 operation.
	- `LoRa_Anchor_dual_RX_TX/`, `LoRa_Anchor_Rx/`, `LoRa_Tag_Tx/`: LoRa communication modules.
	- `Model_AI/`: AI model integration for on-device inference.
	- `GPS_Module/`, `Do_nhiet_do/`, `Do_nhip_tim/`, `Do_khoang_cach/`, `Do_dung_luong_pin/`: Individual sensor modules.
- **Implementation/LoRa_Anchor_dual_RX_TX/**: Additional LoRa gateway scripts.
- **Implementation/RTLS/**: Real-Time Location System (RTLS) code for UWB-based localization.

## Key Features

- Real-time fall detection and health monitoring on the helmet.
- High-precision UWB proximity alerts for hazardous zones.
- Long-range LoRa communication for site-wide safety data.
- Modular, open-source hardware and software for easy adoption.
- Significant cost savings and privacy protection compared to commercial systems.

## Getting Started

1. **Hardware Setup**: Follow the hardware designs in the `Implementation/Helmet_Esp32-s3/` folder.
2. **Firmware Upload**: Use the provided `.ino` and Python files to program the ESP32-S3 and gateways.
3. **Model Deployment**: Deploy the AI model from `DL Model/` to the device as described in `DATA.md`.
4. **Testing**: Use `Test_All_Modules/` to verify sensor and communication modules.
5. **Dataset**: Use the data in `Dataset/` for further model training or evaluation.

## Citation

If you use this project, please cite our paper and reference this repository.

## License

This project is licensed under the terms of the LICENSE file.