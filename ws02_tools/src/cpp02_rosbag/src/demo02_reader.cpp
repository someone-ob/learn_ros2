/*
  需求:读取bag文件数据，并将数据输出在终端
  实现流程:
    1.包含头文件
    2.初始化 ROS2 客户端
    3.自定义节点类
      3-1.创建一个回放对象
      3-2.设置被读取的文件
      3-3.读消息
      3-4.关闭文件
    4.调用spin函数，传入自定义类对象指针
    5.释放资源
*/
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rosbag2_cpp/reader.hpp"

class SimpleBagPlay:public rclcpp::Node{
  private:
    std::unique_ptr<rosbag2_cpp::Reader> reader_;
  public:
    SimpleBagPlay():Node("simple_bag_play"){
      RCLCPP_INFO(this->get_logger(),"消息回放对象创建！");
      // 3-1.创建一个回放对象
      reader_ = std::make_unique<rosbag2_cpp::Reader>();
      // 3-2.设置被读取的文件
      reader_->open("my_bag");
      // 3-3.读消息
      while (reader_->has_next()){
        auto twist = reader_->read_next<geometry_msgs::msg::Twist>();
        RCLCPP_INFO(this->get_logger(),"线速度：%.2f,角速度：%.2f",twist.linear.x,twist.angular.z);
      }
      // 3-4.关闭文件
      reader_->close();
    }
  };
int main(int argc, char ** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<SimpleBagPlay>());
  rclcpp::shutdown();
  return 0;
}