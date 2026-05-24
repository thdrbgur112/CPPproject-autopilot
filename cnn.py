import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import torch

class CNNInferenceNode(Node):
    def __init__(self):
        super().__init__('cnn_inference_node')
        self.subscription = self.create_subscription(Image, '/camera/image_processed', self.image_callback, 10)
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        try:
            self.model = torch.jit.load('best_resnet_model.pt', map_location=self.device)
            self.model.eval()
            self.get_logger().info('🚀 모델 연결 완료!')
        except Exception as e:
            self.get_logger().error(f'❌ 모델 에러: {e}')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            input_tensor = torch.from_numpy(cv_image).float() / 255.0
            input_tensor = input_tensor.unsqueeze(0).unsqueeze(0).to(self.device)
            if input_tensor.shape[1] == 1: input_tensor = input_tensor.repeat(1, 3, 1, 1)
            with torch.no_grad():
                output = self.model(input_tensor)
                output_arr = output.cpu().numpy()[0]
                lateral_err, h_err = float(output_arr[0]), float(output_arr[1])
            steer_angle = float(-lateral_err * 1.5 - h_err * 0.5)
            cmd_msg = Twist()
            cmd_msg.linear.x = 1.5
            cmd_msg.angular.z = steer_angle
            self.cmd_publisher.publish(cmd_msg)
        except Exception as e: pass

def main(args=None):
    rclpy.init(args=args)
    node = CNNInferenceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__': main()
