import rclpy
from rclpy.node import Node
from rclpy.clock import ROSClock

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
# from geometry_msgs.msg import TwistStamped

# Joystick indices
JOY1_X = 0
JOY1_Y = 1
LT = 2
JOY2_X = 3
JOY2_Y = 4
RT = 5

# Button indices
A = 0
B = 1
X = 2
Y = 3
LB = 4
RB = 5

class JoyToTwist(Node):

    def __init__(self):
        super().__init__('joy_to_twist')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        # Publish to /twist_teleop
        self.publisher_ = self.create_publisher(Twist, '/twist_teleop', 10)

        # Publish to /servo_node/delta_twist_cmds
        # self.publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

        
        # Apply threshold
        self.threshold = 0.1

        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):

        # Get joystick values
        joy1_x_value = msg.axes[JOY1_X]
        joy1_y_value = msg.axes[JOY1_Y]
        lt = msg.axes[LT]
        joy2_x_value = msg.axes[JOY2_X]
        joy2_y_value = msg.axes[JOY2_Y]
        rt = msg.axes[RT]

        # Get button values
        lb = msg.buttons[LB]
        rb = msg.buttons[RB]
        a = msg.buttons[A]
        b = msg.buttons[B]

        # Speed function
        sp = 1.0
        if a == 1:
            sp = 2

        # Slow function
        sl = 1.0
        if b ==1:
            sl = 0.5

        # Joy1 & Joy2 Joy to Twist conversions
        twist = Twist()
        twist.linear.x = -joy1_x_value*sp*sl
        twist.linear.z = -joy1_y_value*sp*sl
        twist.angular.y = 2*joy2_x_value*sp*sl
        twist.angular.x = joy2_y_value*sp*sl

        # Joy1 & Joy2 Joy to TwistStamped conversions
        # twist_s = TwistStamped()
        # twist_s.header.stamp = ROSClock().now().to_msg()
        # twist_s.header.frame_id = 'tool0'
        # twist_s.twist.linear.x = -joy1_x_value*sp*sl
        # twist_s.twist.linear.z = -joy1_y_value*sp*sl
        # twist_s.twist.angular.y = 2*joy2_x_value*sp*sl
        # twist_s.twist.angular.x = joy2_y_value*sp*sl

        # Button logic
        if rb == 1:
            angular_z = 1.0
        elif lb == 1:
            angular_z = -1.0
        else:
            angular_z = 0.0

        # LT & RT logic
        linear_y = 0.0
        if lt != 1:
            linear_y = (1-lt)/5
        elif rt != 1:
            linear_y = -(1-rt)/5

        # Apply threshold
        joy_values = [joy1_x_value, joy1_y_value, joy2_x_value, joy2_y_value, linear_y, angular_z]
        filtered_values = [joy_value if abs(joy_value) > self.threshold else 0 for joy_value in joy_values]
        if all(value == 0 for value in filtered_values):
            return
        
        # LT & RT Joy to TwistStamped conversion
        # twist_s.twist.linear.y = linear_y*sp*sl

        # LB & RB Joy to TwistStamped conversion
        # twist_s.twist.angular.z = angular_z*sp*sl

        # Publish twist_s
        #self.publisher_.publish(twist_s)

        # LT & RT Joy to Twist conversion
        twist.linear.y = linear_y*sp*sl

        # LB & RB Joy to Twist conversion
        twist.angular.z = angular_z*sp*sl

        # Publish twist
        self.publisher_.publish(twist)




def main(args=None):
    rclpy.init(args=args)

    joy_to_twist = JoyToTwist()

    rclpy.spin(joy_to_twist)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joy_to_twist.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()