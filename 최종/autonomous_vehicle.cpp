//#include <webots/Robot.hpp>
//#include <webots/Motor.hpp>
//#include <webots/Camera.hpp>
//#include <rclcpp/rclcpp.hpp>
//#include <geometry_msgs/msg/twist.hpp>
//#include <sensor_msgs/msg/image.hpp>
//#include <cv_bridge/cv_bridge.h>
//
//using namespace webots;
//
//class WebotsRobotNode : public rclcpp::Node {
//public:
//    WebotsRobotNode() : Node("webots_robot_node"), robot_(new Robot()) {
//        timeStep_ = (int)robot_->getBasicTimeStep();
//
//        // 카메라 설정
//        cam_ = robot_->getCamera("camera");
//        if (cam_) cam_->enable(timeStep_);
//
//        // 모터 설정 및 멤버 변수에 저장
//        ls_ = robot_->getMotor("left_steer");
//        rs_ = robot_->getMotor("right_steer");
//        lr_ = robot_->getMotor("left_rear_wheel");
//        rr_ = robot_->getMotor("right_rear_wheel");
//
//        if (lr_) { lr_->setPosition(INFINITY); lr_->setVelocity(0.0); }
//        if (rr_) { rr_->setPosition(INFINITY); rr_->setVelocity(0.0); }
//
//        // 발행자 및 구독자
//        camera_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
//        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
//            std::bind(&WebotsRobotNode::cmd_vel_callback, this, std::placeholders::_1));
//
//        timer_ = this->create_wall_timer(std::chrono::milliseconds(timeStep_), std::bind(&WebotsRobotNode::step, this));
//    }
//
//private:
//    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
//        target_speed_ = msg->linear.x * 20.0;
//        target_steer_ = msg->angular.z;
//        if (target_steer_ > 0.3) target_steer_ = 0.3;
//        if (target_steer_ < -0.3) target_steer_ = -0.3;
//    }
//
//    void step() {
//        if (robot_->step(timeStep_) == -1) rclcpp::shutdown();
//
//        // 1. 카메라 데이터 발행
//        if (cam_) {
//            const unsigned char* image = cam_->getImage();
//            int width = cam_->getWidth();
//            int height = cam_->getHeight();
//            cv::Mat mat(height, width, CV_8UC4, (void*)image);
//
//            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgra8", mat).toImageMsg();
//            camera_pub_->publish(*msg);
//        }
//
//        // 2. 모터 제어
//        if (ls_) ls_->setPosition(target_steer_);
//        if (rs_) rs_->setPosition(target_steer_);
//        if (lr_) lr_->setVelocity(target_speed_);
//        if (rr_) rr_->setVelocity(target_speed_);
//    }
//
//    Robot* robot_;
//    int timeStep_;
//    Camera* cam_;
//    Motor* ls_, * rs_, * lr_, * rr_; // [수정됨] 멤버 변수 선언 추가
//    double target_speed_ = 0.0, target_steer_ = 0.0;
//    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_pub_;
//    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
//    rclcpp::TimerBase::SharedPtr timer_;
//};
//
//int main(int argc, char** argv) {
//    rclcpp::init(argc, argv);
//    rclcpp::spin(std::make_shared<WebotsRobotNode>());
//    rclcpp::shutdown();
//    return 0;
//}
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace webots;

class WebotsRobotNode : public rclcpp::Node {
public:
    WebotsRobotNode() : Node("webots_robot_node"), robot_(new Robot()) {
        timeStep_ = (int)robot_->getBasicTimeStep();

        // 카메라 설정
        cam_ = robot_->getCamera("camera");
        if (cam_) cam_->enable(timeStep_);

        // 모터 설정 및 멤버 변수에 저장
        ls_ = robot_->getMotor("left_steer");
        rs_ = robot_->getMotor("right_steer");
        lr_ = robot_->getMotor("left_rear_wheel");
        rr_ = robot_->getMotor("right_rear_wheel");

        if (lr_) { lr_->setPosition(INFINITY); lr_->setVelocity(0.0); }
        if (rr_) { rr_->setPosition(INFINITY); rr_->setVelocity(0.0); }

        // 발행자 및 구독자
        camera_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
            std::bind(&WebotsRobotNode::cmd_vel_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(timeStep_), std::bind(&WebotsRobotNode::step, this));
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        target_speed_ = msg->linear.x * 20.0;
        target_steer_ = msg->angular.z;
        if (target_steer_ > 0.42) target_steer_ = 0.42;
        if (target_steer_ < -0.42) target_steer_ = -0.42;
    }

    void step() {
        if (robot_->step(timeStep_) == -1) rclcpp::shutdown();

        // 1. 카메라 데이터 발행
        if (cam_) {
            const unsigned char* image = cam_->getImage();
            int width = cam_->getWidth();
            int height = cam_->getHeight();
            cv::Mat mat(height, width, CV_8UC4, (void*)image);

            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgra8", mat).toImageMsg();
            camera_pub_->publish(*msg);
        }

        // 2. 모터 제어
        if (ls_) ls_->setPosition(target_steer_);
        if (rs_) rs_->setPosition(target_steer_);
        if (lr_) lr_->setVelocity(target_speed_);
        if (rr_) rr_->setVelocity(target_speed_);
    }

    Robot* robot_;
    int timeStep_;
    Camera* cam_;
    Motor* ls_, * rs_, * lr_, * rr_; // [수정됨] 멤버 변수 선언 추가
    double target_speed_ = 0.0, target_steer_ = 0.0;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WebotsRobotNode>());
    rclcpp::shutdown();
    return 0;
}