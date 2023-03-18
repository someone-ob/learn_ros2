/*
  需求:解析客户端提交的目标点坐标，获取原生乌龟坐标，计算两者距离并响应会客户端
  实现流程:
    1.包含头文件
    2.初始化 ROS2 客户端
    3.自定义节点类
      3-1.创建一个订阅方（原生乌龟位姿）
      3-2.创建一个服务端；
      3-3.回调函数需要解析客户端数据并响应结果到客户端
    4.调用spin函数，传入自定义类对象指针
    5.释放资源
*/
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "base_interfaces_demo/srv/distance.hpp"

using namespace base_interfaces_demo::srv;
using std::placeholders::_1;
using std::placeholders::_2;

class Exer02Server:public rclcpp::Node{
  private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
    rclcpp::Service<Distance>::SharedPtr server_;
    float x,y;
    void pose_callback(const turtlesim::msg::Pose &pose){
      x = pose.x;
      y = pose.y;
    }
    // 3-3.回调函数需要解析客户端数据并响应结果到客户端
    void distance_cb(const Distance::Request::SharedPtr request,const Distance::Response::SharedPtr response){
      //1.解析出目标坐标点
      float goal_x = request->x;
      float goal_y = request->y;
      //2.计算距离
      float distance_x = goal_x - x;
      float distance_y = goal_y - y;
      float distance = std::sqrt(distance_x * distance_x + distance_y * distance_y);
      //3.设置进响应
      response->distance = distance;
      RCLCPP_INFO(this->get_logger(),"二者距离：%.2f",distance);
    }
  public:
    Exer02Server():Node("exer02_server_node_cpp"),x(0.0),y(0.0){
      RCLCPP_INFO(this->get_logger(),"案例二服务端创建！");
      // 3-1.创建一个订阅方（原生乌龟位姿）
      sub_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose",10,std::bind(&Exer02Server::pose_callback,this,_1));
      // 3-2.创建一个服务端；
      server_ = this->create_service<Distance>("distance",std::bind(&Exer02Server::distance_cb,this,_1,_2));
      
    }
  };
int main(int argc, char ** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<Exer02Server>());
  rclcpp::shutdown();
  return 0;
}