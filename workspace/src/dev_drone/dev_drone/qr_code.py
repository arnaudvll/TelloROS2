import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist, Vector3


class QR_Code(Node):
    def __init__(self):
        super().__init__('QR_Code_Node')
        self.subscription = self.create_subscription(
            String,
            'barcode',
            self.onQrCodeReceived(),
            10)
        self.subscription  # prevent unused variable warning

        self.START = "start"
        self.STOP = "stop"

    def onQrCodeReceived(self, msg):
            if (msg.data == self.START):
                self.publisher_ = self.create_publisher(Twist, 'control', 10)
                self.publisher_.publish(Twist(linear = Vector3(x = -10.0, y = 0.0, z = 0.0), angular = Vector3(x = 0.0, y = 0.0, z = 0.0)))
            elif (msg.data == self.STOP):
                self.publisher_ = self.create_publisher(Twist, 'control', 10)
                self.publisher_.publish(Twist(linear = Vector3(x = 0.0, y = 0.0, z = 0.0), angular = Vector3(x = 0.0, y = 0.0, z = 0.0)))  
                time.sleep(1)
                self.publisher_ = self.create_publisher(Empty, 'land', 10)
                self.publisher_.publish(Empty())
            


def main(args=None):
    rclpy.init(args=args)

    QR_Code_Node = QR_Code()

    rclpy.spin(QR_Code_Node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    QR_Code_Node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
