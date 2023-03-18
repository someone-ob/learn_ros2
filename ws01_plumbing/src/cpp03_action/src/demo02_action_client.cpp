/*
  需求:编写动作客户端，可以发送整型数据给服务端，并处理服务器的连续反馈和最终响应结果。
  实现流程:
    前提:可以解析终端下动态传入的参数
    1.包含头文件
    2.初始化 ROS2 客户端
    3.自定义节点类
      3-1.创建动作客户端对象；
      3-2.发送请求；
      3-3.处理关于目标值的服务端响应(回调函数);
      3-4.处理连续反馈(回调函数)；
      3-5.处理最终响应(回调函数)；
    4.调用spin函数，传入自定义类对象指针
    5.释放资源
*/
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/progress.hpp"

using namespace base_interfaces_demo::action;
using namespace std::chrono_literals;
using namespace std::placeholders;

class ProgressActionClient:public rclcpp::Node{
  private:
    rclcpp_action::Client<Progress>::SharedPtr client_;
  public:
    ProgressActionClient():Node("progress_action_client_node_cpp"){
      RCLCPP_INFO(this->get_logger(),"action客户端创建！");
      // 3-1.创建动作客户端对象；
      /*
      rclcpp_action::Client<ActionT>::SharedPtr create_client<ActionT, NodeT>(
        NodeT node, 
        const std::string &name, 
        rclcpp::CallbackGroup::SharedPtr group = nullptr, 
        const rcl_action_client_options_t &options = rcl_action_client_get_default_options())
      */
      client_=rclcpp_action::create_client<Progress>(this,"get_sum");
    }
    // 3-2.发送请求；
    void send_goal(int num){
      //需要连接到服务端
      if(!client_->wait_for_action_server(10s)){
        RCLCPP_INFO(this->get_logger(),"服务连接失败！");
      }
      //发送具体的请求
      /*
      std::shared_future<rclcpp_action::ClientGoalHandle<base_interfaces_demo::action::Progress>::SharedPtr> 
      async_send_goal(
      const base_interfaces_demo::action::Progress::Goal &goal, 
      const rclcpp_action::Client<base_interfaces_demo::action::Progress>::SendGoalOptions &options)
      */
      auto goal=Progress::Goal();
      goal.num=num;
      rclcpp_action::Client<Progress>::SendGoalOptions options;
      options.goal_response_callback=std::bind(&ProgressActionClient::goal_response_callback,this,_1);
      options.feedback_callback=std::bind(&ProgressActionClient::feedback_callback,this,_1,_2);
      options.result_callback=std::bind(&ProgressActionClient::result_callback,this,_1);
      auto future = client_->async_send_goal(goal,options);

    }
    // 3-3.处理关于目标值的服务端响应(回调函数);
    /*
    using GoalHandle = ClientGoalHandle<ActionT>;
    using GoalResponseCallback = std::function<void (typename GoalHandle::SharedPtr)>;
    */
    void goal_response_callback(rclcpp_action::ClientGoalHandle<Progress>::SharedPtr goal_handle){
      if(!goal_handle){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"目标请求被服务端拒绝！");
      }else{
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"目标处理中！");
      }
    }
    // 3-4.处理连续反馈(回调函数)；
    /*
    std::function<void (
        typename ClientGoalHandle<ActionT>::SharedPtr,
        const std::shared_ptr<const Feedback>)>;
    */
    void feedback_callback(rclcpp_action::ClientGoalHandle<Progress>::SharedPtr goal_handle,const std::shared_ptr<const Progress::Feedback> feedback){
      (void) goal_handle;
      double progress=feedback->progress;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"当前进度%.2f",progress);
    }
    // 3-5.处理最终响应(回调函数)；
    /*
    std::function<void (const WrappedResult & result)>;
    */
    void result_callback(const rclcpp_action::ClientGoalHandle<Progress>::WrappedResult & result){
      if (result.code==rclcpp_action::ResultCode::SUCCEEDED){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"最终结果%d",result.result->sum);
      }else if(result.code==rclcpp_action::ResultCode::ABORTED){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"被中断！");
      }else if(result.code==rclcpp_action::ResultCode::CANCELED){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"被取消！");
      }else{
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"未知异常！");
      }
    }
  };
int main(int argc, char ** argv){
  if(argc!=2){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"请提交一个整型数据");
    return 1;
  }
  rclcpp::init(argc,argv);
  auto node =std::make_shared<ProgressActionClient>();
  node->send_goal(atoi(argv[1]));
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}