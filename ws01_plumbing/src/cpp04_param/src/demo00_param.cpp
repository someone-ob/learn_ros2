/*
  需求:演示api的使用。
  实现流程:
    1.包含头文件
    2.初始化 ROS2 客户端
    3.自定义节点类
      3-1.参数对象创建
      3-2.参数对象解析(获取键、值，讲值转换成字符串......)
    4.调用spin函数，传入自定义类对象指针
    5.释放资源
*/
#include <iostream>
#include "rclcpp/rclcpp.hpp"

class MyParam:public rclcpp::Node{
  private:
  public:
    MyParam():Node("my_param_node_cpp"){
      RCLCPP_INFO(this->get_logger(),"参数API使用");
      //3-1.参数对象创建
      rclcpp::Parameter p1("car_name","Tiger");
      rclcpp::Parameter p2("height",1.68);
      rclcpp::Parameter p3("wheels",4);
      //3-2.参数对象解析(获取键、值，讲值转换成字符串......)
      //解析值
      RCLCPP_INFO(this->get_logger(),"car_name = %s",p1.as_string().c_str());
      RCLCPP_INFO(this->get_logger(),"height = %.2f",p2.as_double());
      RCLCPP_INFO(this->get_logger(),"wheels = %ld",p3.as_int());
      //解析键
      RCLCPP_INFO(this->get_logger(),"name = %s",p1.get_name().c_str());
      RCLCPP_INFO(this->get_logger(),"type = %s",p1.get_type_name().c_str());
      RCLCPP_INFO(this->get_logger(),"valuetostring = %s",p2.value_to_string().c_str());
    }
  };
int main(int argc, char ** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<MyParam>());
  rclcpp::shutdown();
  return 0;
}