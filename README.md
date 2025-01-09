

# **Obstacle Avoidance Robot with Servo and Ultrasonic Sensor** 🤖

This project demonstrates a basic obstacle avoidance robot. The robot uses an ultrasonic sensor to detect obstacles within a specified distance and automatically adjusts its direction. A servo motor is used to scan the left and right sides of the robot to choose the best direction for avoiding obstacles.

---

## **📂 Project Files**

- `ObstacleAvoidanceRobot.ino`  
  - Arduino code for the obstacle avoidance robot.

---

## **🔧 Hardware Required**

- **Arduino Uno / Nano** (or any compatible board)
- **1x Servo Motor**
- **2x DC Motors**
- **2x Motor Drivers (e.g., L298N)**
- **1x Ultrasonic Distance Sensor (HC-SR04)**
- **4x Motor Driver Pins**
- **Wires and Breadboard**
- **External Power Supply for Motors**

---

## **🔨 Circuit Diagram**

- **Servo Motor** connected to **Pin 3** (PWM pin).
- **Ultrasonic Sensor**:
  - **TRIG** connected to **Pin 11**.
  - **ECHO** connected to **Pin 12**.
- **Motors** connected through motor driver pins:
  - **Right Motor**: Pins **7, 8** for direction and **Pin 5** for speed control.
  - **Left Motor**: Pins **9, 10** for direction and **Pin 6** for speed control.

---

## **💻 Code Overview**

### **Setup**

- **Servo motor** is attached to Pin 3 and set to the initial position (90 degrees).
- **Ultrasonic sensor** is set up to measure the distance in front of the robot.
- **Motors** are set up for driving the robot, and their directions and speeds are controlled by pins **5, 6, 7, 8, 9, 10**.

### **Loop**

1. **Obstacle Detection**:
   - The robot constantly checks the distance using the ultrasonic sensor.
   - If an obstacle is detected within **30 cm**, the robot stops and adjusts its direction using the motors and servo.

2. **Servo Movement**:
   - The servo scans left and right to check for available paths.
   - Depending on the left and right distances, the robot will decide to turn in the direction where no obstacle is present.

3. **Motor Control**:
   - The `rotateMotor()` function adjusts the direction and speed of both motors.
   - If an obstacle is detected, the robot reverses and chooses the best path based on the distance readings.

4. **Regular Movement**:
   - If no obstacles are detected, the robot moves forward at a constant speed.

---

## **⚙️ Code Explanation**

- **Distance Measurement**: The `mySensor.ping_cm()` function is used to measure the distance to any object in front of the robot.
- **Motor Control**: The `rotateMotor()` function controls the direction and speed of both motors. It uses `digitalWrite()` to set the motor direction and `analogWrite()` to control the motor speed.
- **Servo Movement**: The servo is used to scan left and right for obstacles. The robot adjusts its movement based on the readings from both sides.
- **Obstacle Avoidance Logic**: The robot moves forward if no obstacles are detected. If an obstacle is detected, it reverses for a brief moment, checks both left and right for available space, and adjusts the motors accordingly.

---

## **⚠️ Notes**

- Adjust the **distance threshold** (`DISTANCE_TO_CHECK`) to suit your robot's requirements.
- The **servo movement** is set to scan left and right for 500 milliseconds each, but this delay can be adjusted for different scanning behaviors.

---

## **💡 Future Improvements**

- **Multiple Sensors**: Add more ultrasonic sensors to cover more directions for better obstacle detection.
- **Path Planning Algorithms**: Implement more advanced path planning algorithms like A* or Dijkstra for smarter navigation.
- **Mobile App Control**: Add Bluetooth or Wi-Fi connectivity to control the robot via a mobile app.
- **Speed Control**: Implement adaptive speed control based on obstacle proximity for smoother movement.

---

## **👨‍💻 Author**

- **Technical Tamizha**  
  - [Website](https://www.procreativehub.com)

