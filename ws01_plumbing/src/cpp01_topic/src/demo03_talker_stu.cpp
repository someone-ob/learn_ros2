/*
    需求: 以某个固定频率发送学生信息
    实现流程:
        1.包含头文件 
        2.初始化 ROS2 客户端
        3.自定义节点类
          3-1.创建消息发布方
          3-2.创建定时器
          3-3.组织并发布学生信息
        4.调用spin函数，传入自定义类对象指针
        5.释放资源

*/
#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"

using namespace base_interfaces_demo::msg;
using namespace std::chrono_literals;

class TalkerStu:public rclcpp::Node{
  private:
    rclcpp::Publisher<Student>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    void on_timer(){
        auto student=Student();
        student.age=8;
        student.height=2.20;
        student.name="zhangsan";
        publisher_->publish(student);
        RCLCPP_INFO(this->get_logger(),"发布方发布的消息:(%d,%.2f,%s)",student.age,student.height,student.name.c_str());
    }
  public:
    TalkerStu():Node("minimal_publisher"){
      RCLCPP_INFO(this->get_logger(),"创建发布节点！");
        //3-1.创建消息发布方
        publisher_=this->create_publisher<Student>("chatter_stu",10);
        //3-2.创建定时器
        timer_=this->create_wall_timer(500ms,std::bind(&TalkerStu::on_timer,this));
        //3-3.组织并发布学生信息
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<TalkerStu>());
  rclcpp::shutdown();
  return 0;
}
