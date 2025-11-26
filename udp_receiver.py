import rclpy
from rclpy.node import Node
import socket

class UDPReceiver(Node):
    def __init__(self):
        super().__init__('udp_receiver')

        self.UDP_IP = "0.0.0.0"
        self.UDP_PORT = 5055

        # create UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.UDP_IP, self.UDP_PORT))
        self.sock.setblocking(False)

        self.get_logger().info(f"UDP Receiver running on {self.UDP_PORT}")

        # timer callback to poll UDP
        self.timer = self.create_timer(0.001, self.read_udp)

    def read_udp(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            msg = data.decode().strip()
            self.get_logger().info(f"Received: {msg}")
        except BlockingIOError:
            pass

def main():
    rclpy.init()
    node = UDPReceiver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()