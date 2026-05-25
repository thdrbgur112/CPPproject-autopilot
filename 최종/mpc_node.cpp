#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "mpc_test.cpp"

class MPCNode : public rclcpp::Node
{
public:
    MPCNode() : Node("mpc_node")
    {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        wp_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/waypoints", 10, std::bind(&MPCNode::waypoint_callback, this, std::placeholders::_1));

        mpc_solver_ = std::make_unique<KinematicMPC>();

        // 차량 초기 상태
        current_velocity_ = 2.0;
        current_steer_ = 0.0;
    }

private:
    void waypoint_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 20) return;

        std::vector<double> cnn_inputs(msg->data.begin(), msg->data.end());

        // MPC에 현재 속도와 "진짜" 조향각 전달
        auto controls = mpc_solver_->solve_CNN_inputs(cnn_inputs, current_velocity_, current_steer_);

        geometry_msgs::msg::Twist cmd;
        if (!controls.empty())
        {
            double dt = 0.1; // 제어 주기 10Hz
            double accel_mpc = controls[0](0);
            double steer_rate_mpc = controls[0](1);

            // 🎯 [최종 수정 완료] 마이너스 부호 제거! (MPC와 Webots 방향 완벽 일치)
            double steer_rate_webots = steer_rate_mpc;

            // 변화량을 누적(적분)하여 현재 상태 업데이트
            current_velocity_ += accel_mpc * dt;
            current_steer_ += steer_rate_webots * dt;

            // 물리적 한계치 제한 (Clamping)
            if (current_velocity_ > 5.0) current_velocity_ = 5.0;
            if (current_velocity_ < 0.0) current_velocity_ = 0.0;

            if (current_steer_ > 0.42) current_steer_ = 0.42;
            if (current_steer_ < -0.42) current_steer_ = -0.42;

            // Webots로 최종 값 전송
            cmd.linear.x = current_velocity_;
            cmd.angular.z = current_steer_;
        }
        else
        {
            // 제어 실패 시 안전하게 정지 (또는 직진)
            cmd.linear.x = 0.0;
            cmd.angular.z = current_steer_;
        }

        cmd_pub_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wp_sub_;
    std::unique_ptr<KinematicMPC> mpc_solver_;

    double current_velocity_;
    double current_steer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPCNode>());
    rclcpp::shutdown();
    return 0;
}