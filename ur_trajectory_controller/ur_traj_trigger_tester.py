import rclpy 
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Empty
from keyboard_msgs.msg import Key
from std_srvs.srv import SetBool


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
        
        # The default values in the LLS
        self.inv_x = True
        self.inv_z = True
        self.x_flipper = self.create_client(SetBool, '/scancontrol_driver/invert_x')
        self.z_flipper = self.create_client(SetBool, '/scancontrol_driver/invert_z')

    def key_callback(self, msg):
        # Trigger on Enter keypress
        if msg.code == Key.KEY_RETURN or msg.code == Key.KEY_SPACE:
            self.publisher_trigger.publish(Empty())


        if msg.code == Key.KEY_X:
            self.inv_x = not self.inv_x
            val = SetBool.Request(data=self.inv_x)
            self.get_logger().info(f"Fipping the x-axis of the LLS to {val.data}")
            call = self.x_flipper.call_async(val)
            call.add_done_callback(self.axis_flipped_callback)
        
        if msg.code == Key.KEY_Z:
            self.inv_z = not self.inv_z
            val = SetBool.Request(data=self.inv_z)
            self.get_logger().info(f"Fipping the z-axis of the LLS to {val.data}")
            call = self.z_flipper.call_async(val)
            call.add_done_callback(self.axis_flipped_callback)

    def axis_flipped_callback(self, future):
        result = future.result()
        self.get_logger().info(f"Result of axis flip: {result.succes} \n{result.message}")
        

def main(args=None):
    rclpy.init(args=args)

    ur_trigger = UrController()
    executor = MultiThreadedExecutor()

    rclpy.spin(ur_trigger, executor=executor)
    ur_trigger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    