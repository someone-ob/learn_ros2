/*
  需求:修改turtlesim_node背景颜色
  实现流程:
    1.包含头文件
    2.初始化 ROS2 客户端
    3.自定义节点类
      3-1.创建参数客户端
      3-2.连接参数服务端
      3-3.更新参数
    4.创建对象节点指针，调用其函数
    5.释放资源
*/
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class Exer06Param:public rclcpp::Node{
  private:
    rclcpp::SyncParametersClient::SharedPtr client_;
  public:
    Exer06Param():Node("exer06_param_node_name"){
      RCLCPP_INFO(this->get_logger(),"创建发布节点！");
    // 3-1.创建参数客户端
    client_ = std::make_shared<rclcpp::SyncParametersClient>(this,"/turtlesim");
    }
    // 3-2.连接参数服务端
    bool connect_server(){
      while(!client_->wait_for_service(10s)){
        if (!rclcpp::ok()){
          return false;
        }
        RCLCPP_INFO(this->get_logger(),"服务连接中！");
      }
      return true;
    }

    // 3-3.更新参数
    void update_param(){
        //1.获取参数
        int red = client_->get_parameter<int>("background_r");
        //2.编写循环，修改参数
        rclcpp::Rate rate(30.0);
        while (rclcpp::ok()){
            while (1){
                if (red >= 255){
                    break;
                }
                client_->set_parameters({rclcpp::Parameter("background_r",red)});
                red += 5;
                rate.sleep();
            }
            while (1){
                if (red <= 0){
                    break;
                }
                client_->set_parameters({rclcpp::Parameter("background_r",red)});
                red -= 5;
                rate.sleep();
            }
        }
    }
  };
int main(int argc, char ** argv){
  rclcpp::init(argc,argv);
  auto client = std::make_shared<Exer06Param>();
  bool flag = client->connect_server();
  if (!flag){
    return 0;
  }
  client->update_param();
  rclcpp::shutdown();
  return 0;
}