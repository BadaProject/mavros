import rclpy
from rclpy.node import Node

from mavros_msgs.msg import ActuatorOutputs

class ActuatorSubscriber(Node):

    def __init__(self):
        super().__init__('actuator_subscriber')
        self.subscription = self.create_subscription(
            ActuatorOutputs,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # 수신하면 메시지를 받아서 plc로 전송
        # socket 생성해서 
        pass 


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