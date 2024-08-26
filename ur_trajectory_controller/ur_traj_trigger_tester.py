import rclpy 
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Empty
from keyboard_msgs.msg import Key


class UrController(Node):

    def __init__(self) -> None:
        super().__init__('ur_trigger_tester')

        self.declare_parameter("controller_name", "position_trajectory_controller")
        controller_name = self.get_parameter("controller_name").value

        # Setup the trigger publisher
        publisher_topic = f"/{controller_name}/trigger_move"
        self.publisher_trigger  = self.create_publisher(Empty, publisher_topic, 1)

        self.keyboard_listener  = self.create_subscription(Key, 'keydown', self.key_callback, 1)
        # self.timer              = self.create_timer(6, self.timer_callback)
        
    def key_callback(self, msg):
        # Trigger on Enter keypress
        if msg.code == Key.KEY_RETURN or msg.code == Key.KEY_SPACE:
            self.publisher_trigger.publish(Empty())

def main(args=None):
    rclpy.init(args=args)

    ur_trigger = UrController()
    executor = MultiThreadedExecutor()

    rclpy.spin(ur_trigger, executor=executor)
    ur_trigger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    