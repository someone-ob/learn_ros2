/*
  需求: 发布相对于laser坐标系的坐标点数据
  实现流程:
    1.包含头文
    2.初始化 ROS2 客户端
    3.自定义节点类
      3-1.创建发布方
      3-2.创建定时器
      3-3.组织并发布消息
    4.调用spin函数，传入自定义类对象指针
    5.释放资源
*/
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

using namespace std::chrono_literals;

class PointBroadcaster:public rclcpp::Node{
  private:
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double x;
    // 3-3.组织并发布消息
    void on_timer(){
        //1.组织消息
        geometry_msgs::msg::PointStamped ps;
        ps.header.frame_id = "laser";
        ps.header.stamp = this->now();
        x+=0.05;
        ps.point.x = x;
        ps.point.y = 0;
        ps.point.z = -0.1;
        //2.发布消息
        pub_ -> publish(ps);
    }
  public:
    PointBroadcaster():Node("Node_name"),x(0){
        RCLCPP_INFO(this->get_logger(),"创建发布节点！");
        // 3-1.创建发布方
        pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("point",10);
        // 3-2.创建定时器
        timer_ = this->create_wall_timer(1s,std::bind(&PointBroadcaster::on_timer,this));
        
    }
  };
int main(int argc, char ** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<PointBroadcaster>());
  rclcpp::shutdown();
  return 0;
}