/*
  需求: 以某个固定频率发送“Hello World”，文本后缀编号，每发送一个，编号+1
  实现流程:
    1.包含头文件 
    2.初始化 ROS2 客户端
    3.自定义节点类
      3-1.创建消息发布方
      3-2.创建定时器
      3-3.组织并发布消息
    4.调用spin函数，传入自定义类对象指针
    5.释放资源
*/
#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher:public rclcpp::Node{
  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count;
    void on_timer(){
      auto message=std_msgs::msg::String();
      message.data="hello world!"+std::to_string(count++);
      RCLCPP_INFO(this->get_logger(),"发布方发布的消息%s",message.data.c_str());
      publisher_->publish(message);
    }  
  public:
    MinimalPublisher():Node("minimal_publisher"),count(0){
      RCLCPP_INFO(this->get_logger(),"创建发布节点！");
      //定义发布者
      publisher_=this->create_publisher<std_msgs::msg::String>("topic",10);//参数为话题名称和Qss(消息队列长度)
      //创建定时器
      timer_=this->create_wall_timer(1s,std::bind(&MinimalPublisher::on_timer,this));
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
