import time
import threading
import rclpy
from rclpy.node import Node
from interfaces.srv import ChangeDroneMode
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3



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

        self.surveillanceMode = False
        self.manualMode = False
        self.spielbergMode = False

    def drone_mode_callback(self, request, response):
        drone_mode = request.drone_mode

        match drone_mode:
            # Mode surveillance
            case 0:
                self.surveillanceMode = True
                self.manualMode = False
                your_thread = threading.Thread(target=self.surveillanceModeCallback)
                your_thread.start()
                response.status = True

            # Mode manuel
            case 1:
                self.manualMode = True
                self.surveillanceMode = False
                response.status = True
            
            case _:
                response.status = False
                self.surveillanceMode = False
                self.manualMode = False
        
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