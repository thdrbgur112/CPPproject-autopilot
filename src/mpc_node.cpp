#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "mpc_test.cpp"

class MPCNode : public rclcpp::Node {
public:
    MPCNode() : Node("mpc_node") {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        // [중요] cv_node의 결과물 구독
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_processed", 10, std::bind(&MPCNode::image_callback, this, std::placeholders::_1));
        mpc_solver_ = std::make_unique<KinematicMPC>();
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        (void)msg;
        std::vector<double> cnn_inputs(20, 0.0); // 임시 데이터
        auto controls = mpc_solver_->solve_CNN_inputs(cnn_inputs, 1.0, 0.0);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = (!controls.empty()) ? 2.0 : 0.0; // 움직이게 설정
        cmd.angular.z = (!controls.empty()) ? controls[0](1) : 0.0;
        cmd_pub_->publish(cmd);
    }
    std::unique_ptr<KinematicMPC> mpc_solver_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPCNode>());
    rclcpp::shutdown();
    return 0;
}