/*
  需求:创建参数服务端并操作参数。
  实现流程:
    1.包含头文件
    2.初始化 ROS2 客户端
    3.自定义节点类
      3-1.增
      3-2.删
      3-3.改
      3-4.查
    4.调用spin函数，传入自定义类对象指针
    5.释放资源
*/
#include <iostream>
#include "rclcpp/rclcpp.hpp"

class ParamServer:public rclcpp::Node{
  private:
  public:
    //如果允许删除参数，那么需要通过 NodeOptions 声明
    ParamServer():Node("param_server_node_cpp",rclcpp::NodeOptions().allow_undeclared_parameters(true)){
      RCLCPP_INFO(this->get_logger(),"参数服务端创建了！");
    }
    // 3-1.增
    void declare_param(){
      RCLCPP_INFO(this->get_logger(),"----------------增----------------");
      this->declare_parameter("car_name","Tiger");
      this->declare_parameter("height",1.68);
      this->declare_parameter("wheels",4);

      //用set_parameter增加参数，需要保证rclcpp::NodeOptions().allow_undeclared_parameters(true)被调用
      this->set_parameter(rclcpp::Parameter("width",2.00));
    }
    // 3-2.删
    void del_param(){
      RCLCPP_INFO(this->get_logger(),"----------------删----------------");
      //this->undeclare_parameter("car_name");不能删除用declare_parameter生成的参数
      this->undeclare_parameter("width");
      RCLCPP_INFO(this->get_logger(),"删除后还包含width吗？%d",this->has_parameter("width"));
    }
    // 3-3.改
    void update_param(){
      RCLCPP_INFO(this->get_logger(),"----------------改----------------");
      this->set_parameter(rclcpp::Parameter("height",1.75));
    }
    // 3-4.查
    void get_param(){
      RCLCPP_INFO(this->get_logger(),"----------------查----------------");
      //获取指定参数
      auto car = this->get_parameter("car_name");
      RCLCPP_INFO(this->get_logger(),"key = %s,value = %s",car.get_name().c_str(),car.as_string().c_str());
      //获取一些参数
      auto params = this->get_parameters({"car_name","height","wheels"});
      for (auto &&param : params){
        RCLCPP_INFO(this->get_logger(),"key = %s,value = %s",param.get_name().c_str(),param.value_to_string().c_str());
      }
      //判断是否包含
      RCLCPP_INFO(this->get_logger(),"是否包含car_name？%d",this->has_parameter("car_name"));
      RCLCPP_INFO(this->get_logger(),"是否包含width？%d",this->has_parameter("width"));
    }
  };
int main(int argc, char ** argv){
  rclcpp::init(argc,argv);
  auto node = std::make_shared<ParamServer>();
  node->declare_param();
  node->get_param();
  node->update_param();
  node->del_param();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}