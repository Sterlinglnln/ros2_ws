#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/spawn.hpp"

class TwoTurtlesCircle : public rclcpp::Node
{
public:
    TwoTurtlesCircle() : Node("two_turtles_circle")
    {
        // 创建第一个小海龟的速度发布者（默认已存在的turtle1）
        turtle1_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel", 10);
        
        // 创建服务客户端用于生成第二个小海龟
        spawn_client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
        
        // 等待spawn服务可用
        while (!spawn_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "等待/spawn服务中...");
        }
        
        // 请求生成第二个小海龟（位置在(5, 5)，角度0）
        auto spawn_request = std::make_shared<turtlesim::srv::Spawn::Request>();
        spawn_request->x = 5.0;
        spawn_request->y = 5.0;
        spawn_request->theta = 0.0;
        spawn_request->name = "turtle2";  // 第二个小海龟的名字
        
        // 发送请求并处理响应
        auto result_future = spawn_client_->async_send_request(
            spawn_request,
            std::bind(&TwoTurtlesCircle::on_spawn_response, this, std::placeholders::_1)
        );
        
        // 创建定时器，控制发布频率
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TwoTurtlesCircle::publish_velocities, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "双海龟画圆节点已启动");
    }

private:
    // 处理spawn服务的响应
    void on_spawn_response(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future)
    {
        try {
            auto response = future.get();
            if (!response->name.empty()) {
                RCLCPP_INFO(this->get_logger(), "成功创建第二个小海龟: %s", response->name.c_str());
                // 创建第二个小海龟的速度发布者
                turtle2_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
                    "/" + response->name + "/cmd_vel", 10);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "调用/spawn服务失败: %s", e.what());
        }
    }

    // 发布速度指令
    void publish_velocities()
    {
        // 如果第二个小海龟的发布者未初始化，则不发布
        if (!turtle2_pub_) return;
        
        // 第一个小海龟：逆时针画小圆
        auto msg1 = geometry_msgs::msg::Twist();
        msg1.linear.x = 1.0;    // 线速度
        msg1.angular.z = 2.0;   // 角速度（较大，圆较小）
        
        // 第二个小海龟：顺时针画大圆（参数不同）
        auto msg2 = geometry_msgs::msg::Twist();
        msg2.linear.x = 1.5;    // 线速度更大
        msg2.angular.z = -1.0;  // 负的角速度（顺时针）
        
        turtle1_pub_->publish(msg1);
        turtle2_pub_->publish(msg2);
    }

    // 发布者
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle1_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle2_pub_;
    
    // 服务客户端
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwoTurtlesCircle>());
    rclcpp::shutdown();
    return 0;
}
