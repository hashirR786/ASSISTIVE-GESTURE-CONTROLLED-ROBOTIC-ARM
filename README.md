A ROS2-based robotic arm project that allows gesture-controlled movements via UDP commands. The arm uses servo motors (MG996R and MG90S) driven by a PCA9685 controller and can perform precise movements in real-time.

**📌Features**

->Real-time gesture control via UDP.
->ROS2 nodes for modular design:

  **udp_receiver**: Receives gesture commands over UDP.
  
  **servo_controller**: Controls the robotic arm servos.
  
->Servo smoothing to avoid abrupt movements.

->Safe angle limits to prevent mechanical damage.

->Works with 6 DOF robotic arms (base, shoulder, elbow, forearm, wrist, gripper).

**Hardware Requirements**

->Robotic arm with 6 DOF

->Servos:

  ->MG996R: Base, shoulder, elbow
  
  ->MG90S: Forearm, wrist, gripper
  
->PCA9685 16-channel servo driver

->Raspberry Pi / Jetson / Microcontroller with ROS2 support

->Power supply suitable for servos

**Software Requirements**

->ROS2 (tested on Humble / Foxy)

->Python 3.8+

->rclpy ROS2 Python client library
->adafruit-circuitpython-servokit
->std_msgs ROS2 package
