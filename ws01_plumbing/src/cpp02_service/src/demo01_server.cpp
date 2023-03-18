/*
    需求: 
        编写两个节点实现服务通信，客户端节点需要提交两个整数到服务器
        服务器需要解析客户端提交的数据，相加后，将结果响应回客户端，
        客户端再解析

    流程:
        1.包含头文件 
        2.初始化 ROS2 客户端
        3.自定义节点类
          3-1.创建服务端
          3-2.回调函数解析请求并发送响应
        4.调用spin函数，传入自定义类对象指针
        5.释放资源

*/
#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/addints.hpp"

using namespace base_interfaces_demo::srv;
using namespace std::placeholders;

class AddIntsServer:public rclcpp::Node{
  private:
    rclcpp::Service<Addints>::SharedPtr server_;
    void add(const Addints::Request::SharedPtr req,const Addints::Response::SharedPtr res){
      //3-2.回调函数解析请求并发送响应
      res->sum=req->num1+req->num2;
      RCLCPP_INFO(this->get_logger(),"%d+%d=%d",req->num1,req->num2,res->sum);
    }
  public:
    AddIntsServer():Node("add_ints_server_node"){
      RCLCPP_INFO(this->get_logger(),"服務端節點創建！");
      //3-1.创建服务端
      /*
        模板：服务接口类型
        参数：
            1.服务话题
            2.回调函数
        返回值：服务对象指针
      */
      server_=this->create_service<Addints>("add_ints",std::bind(&AddIntsServer::add,this,_1,_2));
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<AddIntsServer>());
  rclcpp::shutdown();
  return 0;
}
