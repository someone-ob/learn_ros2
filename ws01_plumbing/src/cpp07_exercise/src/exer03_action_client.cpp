/*
  需求:向动作服务端发送目标点数据，并处理响应结果
  实现流程:
    0.解析launch文件传入的参数
    1.包含头文件
    2.初始化 ROS2 客户端
    3.自定义节点类
      3-1.创建动作客户端
      3-2.连接服务端，发送请求
      3-3.处理目标值相关响应结果
      3-4.处理连续反馈
      3-5.处理最终结果
    4.调用spin函数，传入自定义类对象指针
    5.释放资源
*/
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/nav.hpp"

using namespace base_interfaces_demo::action;
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class Exer03ActionClient:public rclcpp::Node{
  private:
    rclcpp_action::Client<Nav>::SharedPtr client_;
  public:
    Exer03ActionClient():Node("exer03_action_client_node"){
      RCLCPP_INFO(this->get_logger(),"创建动作客户端节点！");
      // 3-1.创建动作客户端
      client_=rclcpp_action::create_client<Nav>(this,"nav");
    }
    // 3-2.连接服务端，发送请求
    void send_goal(float x,float y,float theta){
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
      auto goal=Nav::Goal();
      goal.goal_x = x;
      goal.goal_y = y;
      goal.goal_theta = theta;
      rclcpp_action::Client<Nav>::SendGoalOptions options;
      options.goal_response_callback=std::bind(&Exer03ActionClient::goal_response_callback,this,_1);
      options.feedback_callback=std::bind(&Exer03ActionClient::feedback_callback,this,_1,_2);
      options.result_callback=std::bind(&Exer03ActionClient::result_callback,this,_1);
      auto future = client_->async_send_goal(goal,options);
    }
    // 3-3.处理目标值相关响应结果
    /*
    using GoalHandle = ClientGoalHandle<ActionT>;
    using GoalResponseCallback = std::function<void (typename GoalHandle::SharedPtr)>;
    */
    void goal_response_callback(rclcpp_action::ClientGoalHandle<Nav>::SharedPtr goal_handle){
      if(!goal_handle){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"目标请求被服务端拒绝！");
      }else{
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"目标处理中！");
      }
    }
    // 3-4.处理连续反馈
    /*
    std::function<void (
        typename ClientGoalHandle<ActionT>::SharedPtr,
        const std::shared_ptr<const Feedback>)>;
    */
    void feedback_callback(rclcpp_action::ClientGoalHandle<Nav>::SharedPtr goal_handle,const std::shared_ptr<const Nav::Feedback> feedback){
      (void) goal_handle;
      float distance = feedback->distance;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"剩余距离%.2f",distance);
    }
    // 3-5.处理最终结果
    /*
    std::function<void (const WrappedResult & result)>;
    */
    void result_callback(const rclcpp_action::ClientGoalHandle<Nav>::WrappedResult & result){
      if (result.code==rclcpp_action::ResultCode::SUCCEEDED){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"最终坐标为：%.2f,%.2f",result.result->turtle_x,result.result->turtle_y);
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
  //0.解析launch文件传入的参数
    if (argc != 5){
    return 1;
  }
  rclcpp::init(argc,argv);
  auto client = std::make_shared<Exer03ActionClient>();
  client->send_goal(atof(argv[1]),atof(argv[2]),atof(argv[3]));
  rclcpp::spin(client);
  rclcpp::shutdown();
  return 0;
}