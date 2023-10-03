#!/usr/bin/env python

import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist, Vector3

class Control(Node):
    def __init__(self):
        super().__init__('control')

        # Souscription aux controles de la manette
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription

        self.inAir = False

    # Fonction appelée lorsqu'un controle de la manette est modifié
    def listener_callback(self, msg):
        # Bouton Y
        if msg.buttons[3] == 1 and not self.inAir:
            self.publisher_ = self.create_publisher(Empty, 'takeoff', 10)
            self.publisher_.publish(Empty())
            self.inAir = not self.inAir
        # Bouton B
        elif msg.buttons[1] == 1 and self.inAir:
            self.publisher_ = self.create_publisher(Empty, 'emergency', 10)
            self.publisher_.publish(Empty())
            self.inAir = not self.inAir
        # Bouton A
        elif msg.buttons[0] == 1 and self.inAir:
            self.publisher_ = self.create_publisher(Empty, 'land', 10)
            self.publisher_.publish(Empty())
            self.inAir = not self.inAir

        # Flèche du haut
        elif msg.axes[7] == 1 and self.inAir:
            self.publisher_ = self.create_publisher(String, 'flip', 10)
            self.publisher_.publish(String(data = 'f'))
            time.sleep(1)
        # Flèche du bas
        elif msg.axes[7] == -1 and self.inAir:
            self.publisher_ = self.create_publisher(String, 'flip', 10)
            self.publisher_.publish(String(data = 'b'))
            time.sleep(1)
        # Flèche de gauche
        elif msg.axes[6] == 1 and self.inAir:
            self.publisher_ = self.create_publisher(String, 'flip', 10)
            self.publisher_.publish(String(data = 'l'))
            time.sleep(1)
        # Flèche de droite
        elif msg.axes[6] == -1 and self.inAir:
            self.publisher_ = self.create_publisher(String, 'flip', 10)
            self.publisher_.publish(String(data = 'r'))
            time.sleep(1)

        # Controle des mouvements lineaires verticaux et horizontaux avec les joystickss
        elif self.inAir:
            coefLeftRight= - msg.axes[0]
            coefFrontBack = msg.axes[1]
            zAxisMovement = - (- msg.axes[2] + 1) / 2 if msg.axes[2] != 1 else (- msg.axes[5] + 1) / 2
            coefZLeftRight= - msg.axes[3]

            self.publisher_ = self.create_publisher(Twist, 'control', 10)
            self.publisher_.publish(Twist(linear = Vector3(x = 50.0 * coefLeftRight, y = 50.0 * coefFrontBack, z = 50.0 * zAxisMovement), angular = Vector3(x = 0.0, y = 0.0, z = 100.0 * coefZLeftRight)))
            time.sleep(0.001)




def main(args=None):
    rclpy.init(args=args)

    control = Control()

    rclpy.spin(control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
