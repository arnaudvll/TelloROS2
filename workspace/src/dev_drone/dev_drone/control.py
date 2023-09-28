#!/usr/bin/env python

import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist, Vector3

class Control(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.inAir = False

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.axes)
        if msg.buttons[3] == 1 and not self.inAir:
            self.publisher_ = self.create_publisher(Empty, 'takeoff', 10)
            self.publisher_.publish(Empty())
            self.inAir = not self.inAir
        elif msg.buttons[1] == 1 and self.inAir:
            self.publisher_ = self.create_publisher(Empty, 'emergency', 10)
            self.publisher_.publish(Empty())
            self.inAir = not self.inAir
        elif msg.buttons[0] == 1 and self.inAir:
            self.publisher_ = self.create_publisher(Empty, 'land', 10)
            self.publisher_.publish(Empty())
            self.inAir = not self.inAir

        elif msg.axes[7] == 1 and self.inAir:
            self.publisher_ = self.create_publisher(String, 'flip', 10)
            self.publisher_.publish(String(data = 'f'))
            time.sleep(1)
        elif msg.axes[7] == -1 and self.inAir:
            self.publisher_ = self.create_publisher(String, 'flip', 10)
            self.publisher_.publish(String(data = 'b'))
            time.sleep(1)
        elif msg.axes[6] == 1 and self.inAir:
            self.publisher_ = self.create_publisher(String, 'flip', 10)
            self.publisher_.publish(String(data = 'l'))
            time.sleep(1)
        elif msg.axes[6] == -1 and self.inAir:
            self.publisher_ = self.create_publisher(String, 'flip', 10)
            self.publisher_.publish(String(data = 'r'))
            time.sleep(1)

        elif self.inAir:
            ### Controle des mouvements lineaires verticaux (up and down) ###

            coefLeftRight= - msg.axes[0]
            coefFrontBack = msg.axes[1]
            zAxisMovement = - (- msg.axes[2] + 1) / 2 if msg.axes[2] != 1 else (- msg.axes[5] + 1) / 2
            coefZLeftRight= - msg.axes[3]

            self.publisher_ = self.create_publisher(Twist, 'control', 10)
            self.publisher_.publish(Twist(linear = Vector3(x = 50.0 * coefLeftRight, y = 50.0 * coefFrontBack, z = 50.0 * zAxisMovement), angular = Vector3(x = 0.0, y = 0.0, z = 100.0 * coefZLeftRight)))
            time.sleep(0.001)




def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Control()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
