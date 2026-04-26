### 🚗 AutoCar-Kit V3: AutoCar-Kit V3: An AI-Based System for Multiple Autonomous Vehicles with Speed Tracking and Collision Avoidance in a Laboratory Environment at FPT University
![Python](https://img.shields.io/badge/Python-3.9%2B-blue)
![PyTorch](https://img.shields.io/badge/PyTorch-CUDA-orange)
![Hardware](https://img.shields.io/badge/Hardware-ESP32--S3-green)

> **Team Members:**
> * Doan Ngoc Hien
> * Tran Hoang Duy Anh
> * Nguyen Tat Cuong
>
> **Mentor:**
> * AnhKD3 (Khuất Đức Anh)
>
> **Program:** Bachelor of Artificial Intelligence - FPT University

---
📖 1. Project Overview
This project implements a real-time autonomous navigation and Advanced Driver Assistance System (ADAS) for a miniature vehicle platform (AutoCar-Kit V3). Operating over an asynchronous UDP network, the system processes live camera streams from an ESP32-S3 to perform deep learning-based perception, geometric spatial mapping, and closed-loop kinematic control.

Developed as a graduation thesis at FPT University, this repository emphasizes:

* Decoupled Architecture: Parallel execution of Lateral Control (Lane Keeping) and Longitudinal Control (ADAS/Collision Avoidance).

* Multi-Vehicle Coordination: A safety-oriented Leader-Follower framework preventing chain collisions.

* Edge-to-AI Integration: Bridging lightweight embedded hardware with heavy neural networks (YOLOv8n) via zero-lag UDP streaming.

* Real-World Deployment: A fully functional physical testbed, bypassing the limitations of idealized software simulations.

🤝 2. Project Inheritance & Evolution
This project is built upon the AutoCar-Kit hardware platform and the foundational Lane Keeping system developed by a previous capstone project. We respectfully acknowledge the original authors for their open-source contribution.

* **Original Project:** ACE_v2.3 (AutoCar-Kit)
* **Original Repository:** [🔗 GitHub - nohope-n3/ACE_v2.3](https://github.com/nohope-n3/ACE_v2.3.git)
* **V2 Project:** AutoCar-LaneKeeping
* **Original Repository:** 🔗 GitHub duongnd12102003/AutoCar-LaneKeeping(https://github.com/duongnd12102003/AutoCar-LaneKeeping)

While inheriting the mechanical chassis and lateral steering algorithms, our work fundamentally transforms the system by integrating a comprehensive, vision-based ADAS subsystem. We focus on bridging the gap between theoretical AI and physical robotics by:

* Deploying YOLOv8n & ByteTrack for real-time dynamic/static obstacle detection and temporal tracking.

* Implementing Inverse Perspective Mapping (IPM) to translate 2D pixel bounding boxes into actionable 3D metric distances.

* Designing a Kinematic-Coupled Dynamic ROI that bends the collision detection zone according to the vehicle's real-time steering angle.
