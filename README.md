# PID-Controlled Robotic Hand (Arduino)

This project implements a robotic hand using servo motors and flex sensors, controlled by PID algorithms for precise finger movement. It uses an Arduino microcontroller to read sensor data and adjust finger positions in real time.

## Features

- **PID-Controlled Fingers:** Each finger (Thumb, Index, Middle, Pinky) responds to flex sensor input using PID control.
- **Simple Grip Servo:** A separate servo controls the grip with predefined positions.
- **Pushbutton Trigger:** Movement is only active while a button is pressed, simulating grasp action.
- **Serial Output:** Useful for debugging sensor input and monitoring finger activity.

## Hardware Requirements

- Arduino Uno or compatible board
- 4x Flex sensors (connected to analog pins A0–A3)
- 5x Servo motors (connected to digital pins 3, 5, 6, 9, 10)
- Pushbutton (connected to pin 2)
- Resistors as needed for button pull-up/pull-down configuration
- Power supply (depending on servo requirements)

## Libraries Used

- [`Servo`](https://www.arduino.cc/en/Reference/Servo) – for controlling servo motors
- [`PID_v1`](https://playground.arduino.cc/Code/PIDLibrary/) – for implementing the PID algorithm

## Pin Configuration

| Component       | Arduino Pin |
|----------------|-------------|
| Thumb Sensor   | A0          |
| Index Sensor   | A3          |
| Middle Sensor  | A1          |
| Pinky Sensor   | A2          |
| Thumb Servo    | 6           |
| Index Servo    | 5           |
| Middle Servo   | 10          |
| Pinky Servo    | 9           |
| Grip Servo     | 3           |
| Button         | 2           |

## How It Works

- On boot, the hand initializes with a relaxed grip.
- When the pushbutton is pressed, each finger updates its position based on real-time sensor readings using PID control.
- Releasing the button returns the hand to a default resting position.

## Setup Instructions

1. Wire the flex sensors and servos according to the pin configuration.
2. Upload the Arduino sketch to your board.
3. Press and hold the button to activate the hand.
4. Adjust PID parameters in code for better responsiveness if needed.

## Notes

- Make sure to power your servos properly. High torque servos may require an external power supply.
- PID constants (`Kp`, `Ki`, `Kd`) can be tuned to achieve optimal motion behavior.
- You can expand this code by adding more control gestures, serial commands, or integrating Bluetooth/WiFi modules.
