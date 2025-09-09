# ROS2 Autonomous Racing Project

This project explores a fundamental challenge in robotics: how can a vehicle autonomously follow a complex path at speed? It provides a complete, racing controller in ROS2, demonstrating key concepts in robotics and control theory in a realistic simulator.

![Demo GIF of the robot racing in Gazebo](Demo/demo.gif)

---

### The Control Algorithm: Pure Pursuit

The robot's navigation logic is based on the **Pure Pursuit algorithm**, a classic and elegant path tracking method.

Instead of just looking at its error to the closest point on the path, Pure Pursuit looks ahead to a "target point" further along the trajectory. It then calculates the perfect steering arc required to smoothly intercept that target point. By constantly choosing a new target point as it moves, the robot can follow complex curves with remarkable stability and accuracy.



The entire system is a self-contained ROS2 node that subscribes to the robot's position, performs the Pure Pursuit calculations, and publishes velocity commands to the motors.

---

### Key Features & Outcomes

* **Autonomous High-Speed Navigation:** The robot successfully navigates a pre-defined racetrack, demonstrating a practical application of control theory.
* **Endurance & Reliability:** The system is built for continuous operation, capable of running infinite laps while logging performance data.
* **Project Learnings:** This project provided hands-on experience in:
    * ROS2 node development and communication.
    * Implementing and tuning a classic robotics control algorithm.
    * Debugging complex issues involving coordinate frames and version control (Git).

---

### Tech Stack

ROS2 Humble | Python | Gazebo | NumPy
