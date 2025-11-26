import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from adafruit_servokit import ServoKit

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')

        # ROS Subscription
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/arm/angles',
            self.callback,
            10
        )

        # PCA9685 servo driver
        self.kit = ServoKit(channels=16)

        # ---- Servo Calibration ----
        MG996R = [0,1,2]   # base, shoulder, elbow
        MG90S  = [3,4,5]   # forearm, wrist, gripper

        for ch in MG996R:
            self.kit.servo[ch].actuation_range = 180
            self.kit.servo[ch].set_pulse_width_range(600, 2400)

        for ch in MG90S:
            self.kit.servo[ch].actuation_range = 180
            self.kit.servo[ch].set_pulse_width_range(500, 2400)

        # ---- SAFE ANGLE LIMITS ----
        self.safe = {
            0: (60, 120),   # base rotation
            1: (20, 70),    # shoulder
            2: (20, 70),    # elbow
            3: (0, 180),    # forearm rotation
            4: (40, 140),   # wrist
            5: (30, 110),   # gripper
        }

        # Previous angles for smoothing
        self.old = [90] * 6

        # Must change angle at least this much to move
        self.delta_threshold = 3

        self.get_logger().info("🔥 Servo controller started")

    # -----------------------------
    def limit_angle(self, ch, angle):
        lo, hi = self.safe[ch]
        return max(lo, min(hi, angle))

    # -----------------------------
    def callback(self, msg):
        angles = msg.data

        if len(angles) != 6:
            self.get_logger().warn("Invalid angle array")
            return

        # Incoming values from Mac
        shoulder = angles[0]
        elbow    = angles[1]
        forearm  = angles[2]
        wrist    = angles[3]
        grip_raw = angles[4]
        base     = angles[5]

        # Grip mapping
        grip_angle = 45 if grip_raw == 1 else 135

        new_vals = [
            base,
            shoulder,
            elbow,
            forearm,
            wrist,
            grip_angle
        ]

        final_angles = []
        alpha = 0.25  # smoothing factor

        for ch in range(6):
            new_val = self.limit_angle(ch, new_vals[ch])

            # Ignore tiny changes
            if abs(new_val - self.old[ch]) < self.delta_threshold:
                new_val = self.old[ch]

            # Smoothing
            smoothed = (1 - alpha) * self.old[ch] + alpha * new_val

            self.old[ch] = smoothed
            final_angles.append(smoothed)

        # Send to servos
        try:
            for i in range(6):
                self.kit.servo[i].angle = final_angles[i]
        except Exception as e:
            self.get_logger().error(f"Servo error: {e}")

# ------------------------------------------------------------

def main():
    rclpy.init()
    node = ServoController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    print("\n⚠️ Killing all servos...")
    for ch in range(6):
        try:
            node.kit.servo[ch].angle = None
        except:
            pass
    print("Servos disabled safely.")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()