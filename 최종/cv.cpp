#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImagePreprocessorNode : public rclcpp::Node {
public:
    ImagePreprocessorNode() : Node("image_preprocessor_node") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&ImagePreprocessorNode::image_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_processed", 10);
    }
private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgra8");
            cv::Mat src = cv_ptr->image;
            if (src.empty()) return;

            // 1. ROI 설정 (도로 영역: 화면 아래 절반)
            cv::Rect roi(0, src.rows / 2, src.cols, src.rows / 2);
            cv::Mat cropped = src(roi);

            // 2. 그레이스케일 변환
            cv::Mat gray;
            cv::cvtColor(cropped, gray, cv::COLOR_BGRA2GRAY);

            // 🎯 [수정됨] 0.5배 강제 축소 코드 삭제! 원본 픽셀 데이터를 최대한 보존합니다.

            // 3. 단순 이진화 (200 이상만 흰색으로)
            cv::Mat lane_mask;
            cv::threshold(gray, lane_mask, 200, 255, cv::THRESH_BINARY);

            // 4. CNN 입력 사이즈로 최종 조절 (224x224) 
            // 확대 시 픽셀이 덜 깨지도록 보간법(INTER_LINEAR)을 명시적으로 추가했습니다.
            cv::Mat resized_mask;
            cv::resize(lane_mask, resized_mask, cv::Size(224, 224), 0, 0, cv::INTER_LINEAR);

            // 5. 결과물 ROS2 토픽으로 발행
            auto processed_msg = cv_bridge::CvImage(msg->header, "mono8", resized_mask).toImageMsg();
            publisher_->publish(*processed_msg);

        }
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 에러: %s", e.what());
        }
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePreprocessorNode>());
    rclcpp::shutdown();
    return 0;
}