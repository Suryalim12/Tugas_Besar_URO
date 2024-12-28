import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class LaserFollower(Node):
    def __init__(self):
        super().__init__('laser_follower')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.get_logger().info('Laser Follower Node has started.')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower_red = (0, 100, 100)
            upper_red = (10, 255, 255)
            mask = cv2.inRange(hsv, lower_red, upper_red)
            moments = cv2.moments(mask)
            if moments['m00'] > 0:
                cx = int(moments['m10'] / moments['m00'])
                cy = int(moments['m01'] / moments['m00'])
                self.get_logger().info(f'Laser detected at: {cx}, {cy}')
                twist = Twist()
                if cx < 320:
                    twist.angular.z = 0.3
                elif cx > 320:
                    twist.angular.z = -0.3
                else:
                    twist.angular.z = 0.0

                twist.linear.x = 0.2
                self.publisher.publish(twist)
            else:
                self.get_logger().info('No laser detected.')
                self.stop_robot()

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    laser_follower = LaserFollower()
    try:
        rclpy.spin(laser_follower)
    except KeyboardInterrupt:
        laser_follower.get_logger().info('Shutting down Laser Follower Node.')
    finally:
        laser_follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
