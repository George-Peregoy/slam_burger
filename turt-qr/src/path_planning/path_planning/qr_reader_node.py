import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pyzbar.pyzbar import decode
from PIL import Image
import os

class QRPublisher(Node):

    def __init__(self):
        super().__init__('qr_publisher_node')

        # get qr number from launch file 
        self.declare_parameter('qr_num', -1)
        self.qr_num = self.get_parameter('qr_num').value

        # verify qr_num was provided
        if self.qr_num < 0:
            self.get_logger().error('QR num is required')
            raise ValueError('qr_num param must be provided in launch file')
        
        self.get_logger().info(f'Reading QR for world {self.qr_num}')

        # get qr dir
        base_dir = os.path.abspath(os.path.dirname(__file__)) # reads share dir 
        root_dir = os.path.join(base_dir, '..', '..', '..', '..', '..', '..')
        qr_path = os.path.join(root_dir, 'src', 'path_planning', 'qrcodes', f'qr_code_{self.qr_num}.png')

        if not os.path.exists(qr_path):
            self.get_logger().error(f'QR code not found for world {self.qr_num}')
            raise FileNotFoundError(f"QR code not found for world {self.qr_num}")
        
        # read qr
        img = Image.open(qr_path)
        qr_data = decode(img)

        self.qr_string = qr_data[0].data.decode('utf-8')
        self.get_logger().info(f"QR data {self.qr_string}")

        # publish
        self.publisher_ = self.create_publisher(String, 'qr_data', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = self.qr_string
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    path_publisher = QRPublisher()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()