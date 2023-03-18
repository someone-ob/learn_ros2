#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MyNode:public rclcpp::Node{
  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  public:
    MyNode():Node("Node_name","t1_ns"){
      RCLCPP_INFO(this->get_logger(),"创建发布节点！");
      //全局话题：和命名空间，节点名称无关
      //publisher_=this->create_publisher<std_msgs::msg::String>("/topic",10);
      //相对话题：挂载到命名空间下
      //publisher_=this->create_publisher<std_msgs::msg::String>("topic",10);
      //私有话题：挂载到命名空间/节点名称下
      publisher_=this->create_publisher<std_msgs::msg::String>("~/topic",10);
    }
  };
int main(int argc, char ** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<MyNode>());
  rclcpp::shutdown();
  return 0;
}