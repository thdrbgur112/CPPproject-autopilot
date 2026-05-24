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

            // 1. ROI 설정 (도로 영역)
            cv::Rect roi(0, src.rows / 2, src.cols, src.rows / 2);
            cv::Mat gray;
            cv::cvtColor(src(roi), gray, cv::COLOR_BGRA2GRAY);

            // 2. 가우시안 블러 (노이즈 완화)
            cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

            // 3. 적응형 임계값 (차선 강조)
            cv::Mat lane_mask;
            cv::adaptiveThreshold(gray, lane_mask, 255,
                cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                cv::THRESH_BINARY, 11, 2);

            // 4. [핵심] 모폴로지 연산 (노이즈 제거 및 선 굵기 조절)
            // MORPH_OPEN을 사용하면 작은 노이즈 점들이 사라지고, 
            // 원하는 큰 선(차선)들만 남습니다.
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
            cv::morphologyEx(lane_mask, lane_mask, cv::MORPH_OPEN, kernel);

            // 5. 결과물 리사이즈 및 발행
            cv::Mat resized_mask;
            cv::resize(lane_mask, resized_mask, cv::Size(224, 224), 0, 0, cv::INTER_AREA);

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