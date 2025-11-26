import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import socket

class UDPPublisher(Node):
    def __init__(self):
        super().__init__('udp_angle_publisher')

        self.pub = self.create_publisher(Float32MultiArray, '/arm/angles', 10)

        # UDP server
        UDP_IP = "0.0.0.0"
        UDP_PORT = 5055

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))

        self.get_logger().info("📡 Listening for UDP packets on port 5055")

        # Timer = check for UDP every 1ms
        self.create_timer(0.001, self.read_udp)

    def read_udp(self):
        try:
            data, addr = self.sock.recvfrom(1024)
        except BlockingIOError:
            return

        msg = data.decode().strip()
        parts = msg.split(",")

        if len(parts) != 6:
            self.get_logger().warn("Invalid packet")
            return

        angles = Float32MultiArray()
        angles.data = [
            float(parts[0]),
            float(parts[1]),
            float(parts[2]),
            float(parts[3]),
            float(parts[4]),
            float(parts[5])
        ]

        self.pub.publish(angles)
        self.get_logger().info(f"TX: {angles.data}")

def main():
    rclpy.init()
    node = UDPPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()