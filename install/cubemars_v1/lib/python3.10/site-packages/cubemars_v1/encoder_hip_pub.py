import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nmotion_transport import USBInterface, NCoder

class NCoderNode(Node):
    def __init__(self):
        super().__init__('ncoder_node')
        self._iface = USBInterface("/dev/ttyACM0")
        self._encoder = NCoder(interface=self._iface)

        self.angle_publisher_ = self.create_publisher(Float32, 'current_angle', 10)
        self.timer = self.create_timer(0.01, self.callback)
    
    def callback(self):
        msg = Float32()
        (status, msg.data) = self._encoder.getAbsoluteAngle()
        self.angle_publisher_.publish(msg)
        self.get_logger().info(f"Publishing Angle: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = NCoderNode()
    rclpy.spin(node)
    rclpy.shutdown()
