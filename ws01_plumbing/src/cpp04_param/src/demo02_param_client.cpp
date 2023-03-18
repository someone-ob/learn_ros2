/*
  需求:创建参数客户端，，查询或修改服务端参数。
  实现流程:
    1.包含头文件
    2.初始化 ROS2 客户端
    3.自定义节点类
      3-1.创建参数客户端对象
      3-2.连接服务端
      3-3.查询参数
      3-4.修改参数
    4.创建自定义节点类对象，，并调用其函数实现
    5.释放资源
*/
#include <iostream>
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ParamClient:public rclcpp::Node{
  private:
    rclcpp::SyncParametersClient::SharedPtr param_client_;
  public:
    ParamClient():Node("param_client_node_cpp"){
      RCLCPP_INFO(this->get_logger(),"参数客户端创建了！");
      //3-1.创建参数客户端对象
      //参数1：当前对象所依赖的节点；
      //参数2：参数服务端节点名称。
      param_client_ = std::make_shared<rclcpp::SyncParametersClient>(this,"param_server_node_cpp");
      /*
        问题：为什么参数客户端是通过参数服务端的节点名关联？
        答：
          1.参数服务端启动后，底层封装了多个服务通信的服务端；
          2.每个服务端的话题，都是采用/服务端节点名称/xxxx；
          3.参数客户端创建后，也会封装多个服务通信的客户端；
          4.这些客户端和服务端相呼应的，也要使用相同的话题，因此参数客户端是通过参数服务端的节点名创建。
      */
    }
    // 3-2.连接服务端
    bool connect_server(){
      while(!param_client_->wait_for_service(1s)){
        if (!rclcpp::ok()){
          return false;
        }
        RCLCPP_INFO(this->get_logger(),"服务连接中！");
      }
      return true;
    }
    // 3-3.查询参数
    void get_param(){
      RCLCPP_INFO(this->get_logger(),"----------------参数查询操作----------------");
      //获取某个参数
      std::string car_name = param_client_->get_parameter<std::string>("car_name");
      double height = param_client_->get_parameter<double>("height");
      RCLCPP_INFO(this->get_logger(),"car_name = %s",car_name.c_str());
      RCLCPP_INFO(this->get_logger(),"height = %.2f",height);
      //获取多个参数
      auto params = param_client_->get_parameters({"car_name","height","wheels"});
      for (auto &&param : params){
        RCLCPP_INFO(this->get_logger(),"key = %s,value = %s",param.get_name().c_str(),param.value_to_string().c_str());
      }
      //判断是否包含参数
      RCLCPP_INFO(this->get_logger(),"是否包含car_name？%d",param_client_->has_parameter("car_name"));
      RCLCPP_INFO(this->get_logger(),"是否包含width？%d",param_client_->has_parameter("width"));
    }
    // 3-4.修改参数
    void update_param(){
      RCLCPP_INFO(this->get_logger(),"----------------参数修改操作----------------");
      param_client_->set_parameters({rclcpp::Parameter("car_name","pig"),
      rclcpp::Parameter("width",3.0),
      rclcpp::Parameter("length",5.0)});
    }
  };
int main(int argc, char ** argv){
  rclcpp::init(argc,argv);
  auto node = std::make_shared<ParamClient>();
  bool flag = node->connect_server();
  if (!flag){
    return 0;
  }
  node->get_param();
  node->update_param();
  node->get_param();
  rclcpp::shutdown();
  return 0;
}