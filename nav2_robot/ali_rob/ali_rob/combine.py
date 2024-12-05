import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist


class LineDetectionAndFollowing(Node):
    def __init__(self):
        super().__init__('Lane_follower')
        self.subscriber = self.create_subscription(Image, '/camera/image_raw', self.process_image_data, 10)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 40)
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.send_cmd_vel)
        self.velocity = Twist()
        self.error = 0
        self.action = ""

    def send_cmd_vel(self):
        self.velocity.linear.x = 0.5
        if self.error > 0:
            self.velocity.angular.z = 0.15
            self.action = "Go Left"
        else:
            self.velocity.angular.z = -0.15
            self.action = "Go Right"
        self.publisher.publish(self.velocity)

    def process_image_data(self, data):
        frame = self.bridge.imgmsg_to_cv2(data)
        light_line = np.array([100, 100, 100])
        dark_line = np.array([200, 200, 200])
        mask = cv2.inRange(frame, light_line, dark_line)
        canny = cv2.Canny(mask, 40, 10)
        r1, c1 = 150, 0
        img = canny[r1:r1 + 240, c1:c1 + 640]

        edge = np.where(img[160] == 255)[0]
        print(edge)

        if len(edge) == 4:
            edge[0], edge[1] = edge[0], edge[2]
        elif len(edge) == 3 and edge[1] - edge[0] > 5:
            edge[0], edge[1] = edge[0], edge[1]
        elif len(edge) == 3:
            edge[0], edge[1] = edge[0], edge[2]

        if len(edge) < 2:
            edge = [240, 440]

        mid_area = edge[1] - edge[0]
        mid_point = edge[0] + (mid_area / 2)
        img[160, int(mid_point)] = 255

        frame_mid = 639 / 2
        self.error = frame_mid - mid_point

        img[160, int(frame_mid)] = 255
        img[159, int(frame_mid)] = 255
        img[161, int(frame_mid)] = 255
        f_image = cv2.putText(img, self.action, (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0), 2, cv2.LINE_AA)

        cv2.imshow('output image', f_image)
        cv2.waitKey(1)


class ObstacleAvoidanceBot(Node):
    def __init__(self):
        super().__init__('Go_to_position_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 40)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.process_scan_data, 40)
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.send_cmd_vel)
        self.linear_vel = 0.22
        self.regions = {'right': [], 'mid': [], 'left': []}
        self.velocity = Twist()

    def get_scan_values(self, scan_data):
        self.regions = {
            'right': int(min(min(scan_data.ranges[0:60]), 100)),
            'mid': int(min(min(scan_data.ranges[60:120]), 100)),
            'left': int(min(min(scan_data.ranges[120:180]), 100)),
        }
        print(self.regions['left'], " / ", self.regions['mid'], " / ", self.regions['right'])

    def send_cmd_vel(self):
        self.velocity.linear.x = self.linear_vel

        if self.regions['left'] > 2 and self.regions['mid'] > 2 and self.regions['right'] > 2:
            self.velocity.angular.z = 0.0
            print("forward")
        elif self.regions['left'] > 2 and self.regions['mid'] > 2 and self.regions['right'] < 2:
            self.velocity.angular.z = 1.57
            print("right")
        elif self.regions['left'] < 2 and self.regions['mid'] > 2 and self.regions['right'] > 2:
            self.velocity.angular.z = -1.57
            print("left")
        elif self.regions['left'] < 2 and self.regions['mid'] < 2 and self.regions['right'] < 2:
            self.velocity.angular.z = 3.14
            print("reverse")
        else:
            print("some other conditions are required to be programmed")

        self.publisher.publish(self.velocity)

    def process_scan_data(self, scan_data):
        self.get_scan_values(scan_data)
        self.send_cmd_vel()


class ObstacleAvoidanceLaneFollowing(Node):
    def __init__(self):
        super().__init__('Obstacle_Avoidance_Lane_Following')
        self.lane_follower = LineDetectionAndFollowing()
        self.obstacle_avoidance = ObstacleAvoidanceBot()

    def process_image_data(self, image_data):
        self.lane_follower.process_image_data(image_data)

    def process_scan_data(self, scan_data):
        self.obstacle_avoidance.process_scan_data(scan_data)


def main(args=None):
    rclpy.init(args=args)
    oa_lf_node = ObstacleAvoidanceLaneFollowing()

    image_sub = oa_lf_node.create_subscription(Image, '/camera/image_raw', oa_lf_node.process_image_data, 10)
    scan_sub = oa_lf_node.create_subscription(LaserScan, '/scan', oa_lf_node.process_scan_data, 10)

    rclpy.spin(oa_lf_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
