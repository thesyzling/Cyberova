import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        # Publisher: 50 Hz frekansında cmd_vel gönderiyor
        self.publisher = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel', 10)
        timer_period = 0.02  # 50 Hz
        self.timer = self.create_timer(timer_period, self.publish_cmd_vel)

    def publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = 1.0  # İleri hız (m/s)
        msg.linear.y = 0.0  # Yan hız
        msg.linear.z = 0.0  # Dikey hız
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0  # Dönüş hızı (rad/s)

        self.publisher.publish(msg)
        self.get_logger().info('Published cmd_vel: linear.x=%.1f m/s, angular.z=%.1f rad/s' % (msg.linear.x, msg.angular.z))

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
