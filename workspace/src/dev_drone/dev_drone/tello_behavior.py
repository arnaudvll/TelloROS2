import time
import threading
import rclpy
from rclpy.node import Node
from interfaces.srv import ChangeDroneMode
from std_msgs.msg import String, Empty
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3

import sys
import cv2
import imutils
import numpy as np
import sklearn

from cv_bridge import CvBridge




class TelloBehavior(Node):

    def __init__(self):
        super().__init__('drone_mode_service')
        self.srv = self.create_service(ChangeDroneMode, 'drone_mode', self.drone_mode_callback)

        self.controlSubscription = self.create_subscription(
            Twist,
            'secure_cmd',
            self.onSecureCmdReceived,
            10)
        self.controlSubscription  # prevent unused variable warning

        self.flipSubscription = self.create_subscription(
            String,
            'secure_flip',
            self.onSecureFlipReceived,
            10)
        self.flipSubscription  # prevent unused variable warning

        self.qrCodeSubscription = self.create_subscription(
            String,
            'barcode',
            self.onQrCodeReceived,
            10)
        self.qrCodeSubscription  # prevent unused variable warning

        self.imageSubscription = self.create_subscription(
            Image,
            'image',
            self.onImageReceived,
            10)
        self.imageSubscription  # prevent unused variable warning

        self.surveillanceMode = False
        self.manualMode = False
        self.spielbergMode = False
        self.qrCodeMode = False
        self.followerMode = False

        self.START = "start"
        self.STOP = "stop"

        self.cvBridge = CvBridge()
        self.xDif = 0
        self.zDif = 0

        # Set to QR Code Mode at the start
        self.drone_mode = 2

    def drone_mode_callback(self, request, response):
        self.drone_mode = request.drone_mode

        match self.drone_mode:
            # Mode surveillance
            case 0:
                self.surveillanceMode = True
                self.manualMode = False
                self.qrCodeMode = False
                self.followerMode = False
                thread = threading.Thread(target=self.surveillanceModeCallback)
                thread.start()

                response.status = True

            # Mode manuel
            case 1:
                self.manualMode = True
                self.surveillanceMode = False
                self.qrCodeMode = False
                self.followerMode = False

                response.status = True

            # Mode QR Code
            case 2:
                self.qrCodeMode = True
                self.manualMode = False
                self.surveillanceMode = False
                self.followerMode = False

                response.status = True

            # Mode Follower
            case 3: 
                self.followerMode = True
                self.manualMode = False
                self.surveillanceMode = False
                self.qrCodeMode = False

                response.status = True
            
            case _:
                self.surveillanceMode = False
                self.manualMode = False
                self.qrCodeMode = False

                response.status = False
        
        return response
    
    def surveillanceModeCallback(self):
        while (self.surveillanceMode):
            self.publisher_ = self.create_publisher(Twist, 'control', 10)
            self.publisher_.publish(Twist(linear = Vector3(x = 0.0, y = 0.0, z = 0.0), angular = Vector3(x = 0.0, y = 0.0, z = 40.0)))
            time.sleep(1)
    
    def onSecureCmdReceived(self, msg):
        if (self.manualMode):
            self.publisher_ = self.create_publisher(Twist, 'control', 10)
            self.publisher_.publish(msg)

    def onSecureFlipReceived(self, msg):
        if (self.manualMode or self.spielbergMode):
            self.publisher_ = self.create_publisher(String, 'flip', 10)
            self.publisher_.publish(msg)

    def onQrCodeReceived(self, msg):
        if (self.qrCodeMode):
            if (msg.data == self.STOP):
                self.publisher_ = self.create_publisher(Twist, 'control', 10)
                self.publisher_.publish(Twist(linear = Vector3(x = 0.0, y = 0.0, z = 0.0), angular = Vector3(x = 0.0, y = 0.0, z = 0.0)))  
                time.sleep(1)
                self.publisher_ = self.create_publisher(Empty, 'land', 10)
                self.publisher_.publish(Empty())
            elif (msg.data == self.START):
                self.publisher_ = self.create_publisher(Twist, 'control', 10)
                self.publisher_.publish(Twist(linear = Vector3(x = -10.0, y = 0.0, z = 0.0), angular = Vector3(x = 0.0, y = 0.0, z = 0.0)))
    
    def onImageReceived(self, msg):
        if (self.followerMode):
            frame = self.cvBridge.imgmsg_to_cv2(msg)

            cv2.imshow("frame", frame)
            key = cv2.waitKey(1) & 0xFF
            
            
            #example code from google TODO: link
            qr_decoder = cv2.QRCodeDetector()
        
            # Detect and decode the qrcode
            data, bbox, rectified_image = qr_decoder.detectAndDecode(frame)
            if len(data)>0:
                pt1 = int(bbox[0][0][0]), int(bbox[0][0][1])    # angle en haut à gauche
                pt2 = int(bbox[0][2][0]), int(bbox[0][2][1])    # angle en bas à droite

                xMid = (pt1[0] + pt2[0]) / 2
                zMid = (pt1[1] + pt2[1]) / 2

                self.zDif = 480 - zMid
                self.xDif = 360 - xMid

                self.followQrCode()
            else:
                self.publisher_ = self.create_publisher(Twist, 'control', 10)
                self.publisher_.publish(Twist(linear = Vector3(x = 0.0, y = 0.0, z = 0.0), angular = Vector3(x = 0.0, y = 0.0, z = 0.0)))
                time.sleep(0.01)
                print("QR Code not detected")
                cv2.imshow("Results", frame)

           

    def followQrCode(self):
        self.get_logger().info('actuel xdif "%d"' % self.xDif)
        if (abs(self.xDif) > 20):
            if (abs(self.zDif) > 20):
                self.publisher_ = self.create_publisher(Twist, 'control', 10)
                self.publisher_.publish(Twist(linear = Vector3(x = -10.0 if self.xDif > 0 else 10.0, y = 0.0, z = -10.0 if self.zDif > 0 else 10.0), angular = Vector3(x = 0.0, y = 0.0, z = 0.0)))
                time.sleep(0.01)
            else:
                self.publisher_ = self.create_publisher(Twist, 'control', 10)
                self.publisher_.publish(Twist(linear = Vector3(x = -10.0 if self.xDif > 0 else 10.0, y = 0.0, z = 0.0), angular = Vector3(x = 0.0, y = 0.0, z = 0.0)))
                time.sleep(0.01)
        else:
            if (abs(self.zDif) > 20):
                self.publisher_ = self.create_publisher(Twist, 'control', 10)
                self.publisher_.publish(Twist(linear = Vector3(x = 0.0, y = 0.0, z = -10.0 if self.zDif > 0 else 10.0), angular = Vector3(x = 0.0, y = 0.0, z = 0.0)))
                time.sleep(0.01)

            


def main(args=None):
    rclpy.init(args=args)

    tello_behavior = TelloBehavior()

    rclpy.spin(tello_behavior)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tello_behavior.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()