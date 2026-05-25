#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import torch
from torchvision import transforms
from PIL import Image as PILImage
import numpy as np
import os  # [수정] 경로 설정을 위해 os 모듈 추가!

class CNNInferenceNode(Node):
    def __init__(self):
        super().__init__('cnn_inference_node')
        self.subscription = self.create_subscription(Image, '/camera/image_processed', self.image_callback, 1)
        self.waypoint_pub = self.create_publisher(Float32MultiArray, '/waypoints', 10)
        self.bridge = CvBridge()
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

        # [수정] 모델 파일의 우분투 절대 경로 설정
        model_path = os.path.expanduser('~/ros2_ws/src/auto_pkg/scripts/best_resnet_model.pt')

        try:
            # [수정] 파일 이름 대신 절대 경로(model_path) 변수를 넣습니다.
            self.model = torch.jit.load(model_path, map_location=self.device)
            self.model.eval()
            self.get_logger().info("✅ 모델 로드 성공!")
        except Exception as e:
            self.get_logger().error(f"❌ 모델 로드 실패: {e}\n경로({model_path})에 파일이 있는지 꼭 확인하세요!")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            pil_image = PILImage.fromarray(cv_image)
            input_tensor = self.transform(pil_image).unsqueeze(0).to(self.device)

            with torch.no_grad():
                output = self.model(input_tensor)
                output_arr = output.cpu().numpy()[0] 
            
            wp_msg = Float32MultiArray()
            wp_msg.data = output_arr.tolist()
            self.waypoint_pub.publish(wp_msg)
            
        except Exception as e:
            self.get_logger().error(f"추론 에러: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CNNInferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()