/*
    需求: 创建客户端，组织数据并提交，然后处理响应结果。
    实现流程:
        前提：需要判断提交的参数是否正确
        1.包含头文件 
        2.初始化 ROS2 客户端
        3.自定义节点类
          3-1.创建客户端
          3-2.连接服务器(对于服务通信而言，如果客户端连接不到服务器，那么不能发送请求)
          3-3.发送请求
        4.直接创建对象指针
          需要调用连接服务的函数，根据连接的结果作下一步处理。
          连接服务后，调用请求发送函数
          再处理响应结果
        5.释放资源

*/
#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/addints.hpp"
using namespace std::chrono_literals;
using namespace base_interfaces_demo::srv;

class AddIntsClient:public rclcpp::Node{
  private:
    rclcpp::Client<Addints>::SharedPtr client_;
  public:
    AddIntsClient():Node("add_ints_client_node"){
      RCLCPP_INFO(this->get_logger(),"客户端節點創建！");
      //3-1.创建客户端
      /*
        模板：服务接口类型 
        参数：服务话题名称
        返回值：服务对象指针
      */
      client_=this->create_client<Addints>("add_ints");
    }
    //3-2.连接服务器(对于服务通信而言，如果客户端连接不到服务器，那么不能发送请求)
    bool connect_server(){
      while(!client_->wait_for_service(2s)){
        if(!rclcpp::ok()){
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"强行终止客户端，程序中断");
          return false;

        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"服务连接中！");
      }
      return true;
    }
    //3-3.发送请求
    //编写发送请求函数。 -------参数是两个整型数据，返回值是提交请求值后服务端的返回结果
    rclcpp::Client<Addints>::FutureAndRequestId send_request(int num1,int num2){
      //组织请求数据,发送
      auto request=std::make_shared<Addints::Request>();
      request->num1 = num1;
      request->num2 = num2;
      return client_->async_send_request(request);

    }
};

int main(int argc, char ** argv)
{
  if(argc!=3){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"请提交两个整型数据");
    return 1;
  }
  rclcpp::init(argc,argv);
  //创建客户端对象
  auto client=std::make_shared<AddIntsClient>();
  //调用客户端对象的连接服务器功能
  bool flag=client->connect_server();
  if (!flag){
    /*
      rclcpp::get_logger("name")创建对象不依赖与context
    */
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"连接服务器失败，程序退出");
    return 0;
  }
  //调用请求提交函数，接收并处理响应结果
  auto future=client->send_request(atoi(argv[1]),atoi(argv[2]));
  //处理响应
  if (rclcpp::spin_until_future_complete(client,future)==rclcpp::FutureReturnCode::SUCCESS){//成功
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"响应成功，sum=%d",future.get()->sum);

  }else{//失败
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"响应失败！");
  }
  rclcpp::shutdown();
  return 0;
}
