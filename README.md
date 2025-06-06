# 

# General Description

This project showcases a fully self-built, self-balancing robot designed and manufactured entirely by our team. Every mechanical component has been custom-designed and 3D-printed, enabling full control over the physical architecture and rapid iteration during development.

The robot balances on two wheels using a PID (Proportional-Integral-Derivative) controller and real-time feedback from an inertial measurement unit (IMU). Based on the tilt angle, the controller adjusts the speed and direction of two DC motors to keep the robot upright. The entire system is powered by a Seeed Studio XIAO ESP32S3 microcontroller, chosen for its compact form factor, Wi-Fi/Bluetooth capabilities, and processing performance.

Inspired by the dynamics of an inverted pendulum, this project combines hands-on mechatronics, embedded systems, and control theory. It serves as a concrete example of how classical feedback algorithms like PID can be used to stabilize inherently unstable systems using custom hardware and software.

Our goal is to explore the full stack of robotic system design — from 3D mechanical fabrication and motor control to sensor integration and algorithmic tuning — in a way that is modular, reproducible, and educational.
