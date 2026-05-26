// ROS2 C++ 클라이언트 핵심 라이브러리
#include <rclcpp/rclcpp.hpp>
// ROS2 이미지 메시지 타입 처리를 위한 헤더
#include <sensor_msgs/msg/image.hpp>
// ROS2 이미지 메시지와 OpenCV 이미지 데이터(Mat) 간의 변환을 위한 패키지
#include <cv_bridge/cv_bridge.h>
// OpenCV 영상 처리 라이브러리
#include <opencv2/opencv.hpp>

// 이미지 전처리를 수행하는 ROS2 노드 클래스 정의
class ImagePreprocessorNode : public rclcpp::Node {
public:
    // 생성자: 노드 이름을 "image_preprocessor_node"로 초기화
    ImagePreprocessorNode() : Node("image_preprocessor_node") {
        // [구독자(Subscriber) 생성]
        // "/camera/image_raw" 토픽에서 원본 이미지를 받아옴 (큐 사이즈: 10)
        // 메시지가 들어올 때마다 image_callback 함수가 실행되도록 바인딩
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&ImagePreprocessorNode::image_callback, this, std::placeholders::_1));
        
        // [발행자(Publisher) 생성]
        // 전처리가 완료된 이미지를 "/camera/image_processed" 토픽으로 발행 (큐 사이즈: 10)
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_processed", 10);
    }

private:
    // 이미지가 수신될 때마다 호출되는 콜백 함수
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const {
        try {
            // [ROS -> OpenCV 변환]
            // 수신된 ROS 이미지를 OpenCV에서 처리할 수 있도록 변환 (BGRA 8비트 포맷 사용)
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgra8");
            cv::Mat src = cv_ptr->image; // 변환된 원본 이미지 데이터
            
            // 이미지가 비어있는 경우 에러 방지를 위해 함수 종료
            if (src.empty()) return;

            // 1. ROI(Region of Interest, 관심 영역) 설정
            // 화면의 아래쪽 절반만 잘라내어 도로/차선 영역에 집중 (연산량 감소 효과)
            // 인자: x시작점, y시작점, 너비, 높이
            cv::Rect roi(0, src.rows / 2, src.cols, src.rows / 2);
            cv::Mat cropped = src(roi); // 잘라낸 이미지 저장

            // 2. 그레이스케일(흑백) 변환
            // 컬러(BGRA) 이미지를 흑백(GRAY)으로 변환하여 연산 단순화
            cv::Mat gray;
            cv::cvtColor(cropped, gray, cv::COLOR_BGRA2GRAY);

            // 🎯 [수정됨] 0.5배 강제 축소 코드 삭제! 원본 픽셀 데이터를 최대한 보존합니다.

            // 3. 이진화 (Thresholding)
            // 픽셀 밝기 값이 200 이상인 부분(차선 등 밝은 객체)은 255(흰색)로,
            // 그 미만인 부분은 0(검은색)으로 변환하여 흑백 대비를 극대화
            cv::Mat lane_mask;
            cv::threshold(gray, lane_mask, 200, 255, cv::THRESH_BINARY);

            // 4. CNN(딥러닝 모델) 입력 크기로 이미지 크기 조절
            // 모델의 입력 규격에 맞게 224x224 사이즈로 변환
            // cv::INTER_LINEAR (쌍선형 보간법)을 사용하여 확대/축소 시 픽셀 깨짐을 완화
            cv::Mat resized_mask;
            cv::resize(lane_mask, resized_mask, cv::Size(224, 224), 0, 0, cv::INTER_LINEAR);

            // 5. 결과물을 ROS2 토픽으로 발행
            // OpenCV의 처리 결과(resized_mask)를 다시 ROS2 메시지 포맷("mono8" 흑백)으로 변환
            auto processed_msg = cv_bridge::CvImage(msg->header, "mono8", resized_mask).toImageMsg();
            publisher_->publish(*processed_msg); // 처리된 이미지 발행

        }
        catch (cv_bridge::Exception& e) {
            // 이미지 변환 중 에러 발생 시, 프로그램이 다운되지 않고 로그를 남기도록 예외 처리
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 에러: %s", e.what());
        }
    }

    // ROS2 통신을 위한 구독자 및 발행자 포인터 선언
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

// 프로그램의 진입점(Main 함수)
int main(int argc, char* argv[]) {
    // ROS2 통신 초기화
    rclcpp::init(argc, argv);
    
    // ImagePreprocessorNode를 생성하고 끄지 않는 이상 계속 실행되도록(spin) 유지
    rclcpp::spin(std::make_shared<ImagePreprocessorNode>());
    
    // 노드 종료 시 ROS2 통신을 깔끔하게 정리하고 종료
    rclcpp::shutdown();
    
    return 0;
}
