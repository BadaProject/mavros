import rclpy
from rclpy.node import Node
import socket
from py_plc.plc import PLCPacket  

from mavros_msgs.msg import ActuatorOutputs

class ActuatorSubscriber(Node):

    def __init__(self):
        super().__init__('actuator_subscriber')
        self.px4_listen_port = 2006
        self.plc_ip = '127.0.0.1' #self.plc_ip = '192.168.2.88'
        self.plc_port = 2005
        self.px4_ip = '127.0.0.1' # self.px4_ip = '192.168.2.200'
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # self.sock.bind((self.px4_ip, self.px4_listen_port))
        self.plcPacket = PLCPacket()
        self.subscription = self.create_subscription(
            ActuatorOutputs,
            'actuatoroutput',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # 수신하면 메시지를 받아서 plc로 전송
        # socket 생성해서
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        throttle = int(msg.actuator[3])
        steering = int(msg.actuator[1])
        sock.sendto(self.plcPacket.makeWritePacket(), (self.plc_ip, self.plc_port))
        print("sending: ", throttle, steering)

def main(args=None):
    rclpy.init(args=args)

    actuator_subscriber = ActuatorSubscriber()

    rclpy.spin(actuator_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    actuator_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()