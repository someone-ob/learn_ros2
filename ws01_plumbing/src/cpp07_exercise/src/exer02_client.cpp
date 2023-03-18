/*
  需求:客户端需要提交目标点坐标，并解析响应结果。
  实现流程:
    0.解析动态传入的数据，作为目标点坐标
    1.包含头文件
    2.初始化 ROS2 客户端
    3.自定义节点类
      3-1.创建客户端
      3-2.客户端需要连接客户端
      3-3.发送请求数据
    4.调用节点对象指针的相关函数
    5.释放资源
*/
#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/distance.hpp"

using namespace base_interfaces_demo::srv;
using namespace std::chrono_literals;

class Exer02Client:public rclcpp::Node{
  private:
    rclcpp::Client<Distance>::SharedPtr client_;
  public:
    Exer02Client():Node("exer02_client_node_cpp"){
      RCLCPP_INFO(this->get_logger(),"案例二客户端创建！");
      // 3-1.创建客户端
      client_=this->create_client<Distance>("distance");
    }
    // 3-2.客户端需要连接客户端
  bool connect_server(){
    while(!client_->wait_for_service(2s)){
      if(!rclcpp::ok()){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"强行终止客户端，程序中断");
        return false;
      }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"服务连接中！");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"服务连接成功！");
    return true;
  }
  // 3-3.发送请求数据
  rclcpp::Client<Distance>::FutureAndRequestId send_goal(float x,float y,float theta){
    auto request=std::make_shared<Distance::Request>();
    request->x = x;
    request->y = y;
    request->theta = theta;
    return client_->async_send_request(request);
  }

};
int main(int argc, char ** argv){
  if (argc != 5){
    return 1;
  }
  rclcpp::init(argc,argv);
  float goal_x = atof(argv[1]);
  float goal_y = atof(argv[2]);
  float goal_theta = atof(argv[3]);
  auto client = std::make_shared<Exer02Client>();
  //调用客户端对象的连接服务器功能
  bool flag=client->connect_server();
  if (!flag){
    /*
      rclcpp::get_logger("name")创建对象不依赖与context
    */
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"连接服务器失败，程序退出");
    return 1;
  }
  //调用请求提交函数，接收并处理响应结果
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"debug0");
  auto future=client->send_goal(goal_x,goal_y,goal_theta);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"debug1");
  //处理响应
  if (rclcpp::spin_until_future_complete(client,future) == rclcpp::FutureReturnCode::SUCCESS){//成功
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"响应成功，二者距离=%.2f",future.get()->distance);
  }else{//失败
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"响应失败！");
  }
  rclcpp::shutdown();
  return 0;
}