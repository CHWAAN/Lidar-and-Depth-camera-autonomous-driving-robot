import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control')
        self.serial_port = '/dev/ttyUSB0'  # 시리얼 포트 설정
        self.serial_baudrate = 9600 # 모터 기본 baudrate 9600 사용

        self.serial = serial.Serial(self.serial_port, self.serial_baudrate)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def cmd_vel_callback(self, msg):
        left_rpm, right_rpm = self.twist_to_rpm(msg)
        self.send_rpm(left_rpm, right_rpm)
        self.get_logger().info('Sent: Left RPM=%d, Right RPM=%d' % (left_rpm, right_rpm))

    def twist_to_rpm(self, twist):
        wheel_separation = 0.5  # 바퀴 간 거리 설정

        linear_x = twist.linear.x
        angular_z = twist.angular.z

        left_rpm = int((linear_x - angular_z * wheel_separation) * 10)  # Scale to RPM values (0.1 RPM)
        right_rpm = int((linear_x + angular_z * wheel_separation) * 10)  # Scale to RPM values (0.1 RPM)

        return left_rpm, right_rpm

    def send_rpm(self, left_rpm, right_rpm):
        # 통신 프로트콜 노드
        header = bytearray([0xFF, 0xFE])
        id_byte = bytearray([0x00])
        data_size = bytearray([0x06])
        mode = bytearray([0x03])
        speed_direction = bytearray([0x00 if left_rpm >= 0 else 0x01])  # Speed direction (0x00 for forward, 0x01 for reverse)
        speed = bytearray([(abs(left_rpm) & 0xFF), ((abs(left_rpm) >> 8) & 0xFF), (abs(right_rpm) & 0xFF), ((abs(right_rpm) >> 8) & 0xFF)])  # Speed values (LSB to MSB)
        speed_arrival_time = bytearray([0x0A])

        # Calculate the checksum
        checksum = bytearray([(~sum(id_byte + data_size + mode + speed_direction + speed + speed_arrival_time) + 1) & 0xFF])

        # Construct the complete command packet
        command_packet = header + id_byte + data_size + checksum + mode + speed_direction + speed + speed_arrival_time
        self.get_logger().info('Send packet: %s' % (command_packet))
        self.serial.write(command_packet)

def main(args=None):
    rclpy.init(args=args)
    robot_control_node = RobotControlNode()
    rclpy.spin(robot_control_node)
    robot_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
