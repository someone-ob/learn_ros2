/*
  需求:编写动作服务端，需要解析客户端提交的数字，遍历该数字并累加求和，最终结果响应回客户端，
  且请求响应过程中需要连续反馈。
  分析：
    1.创建动作服务端对象
    2.处理提交的目标值
    3.生成连续反馈
    4.响应最终结果
    5.处理取消请求
  实现流程:
    1.包含头文件
    2.初始化 ROS2 客户端
    3.自定义节点类
      3-1.创建动作服务端对象
      3-2.处理提交目标值（回调函数）
      3-3.生成连续反馈与最终响应（回调函数）
      3-4.处理取消请求
    4.调用spin函数，传入自定义类对象指针
    5.释放资源
*/
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/progress.hpp"
using namespace base_interfaces_demo::action;

class ProgressActionServer:public rclcpp::Node{
  private:
    rclcpp_action::Server<Progress>::SharedPtr server_;
    
  public:
    ProgressActionServer():Node("progress_action_server_node_cpp"){
      RCLCPP_INFO(this->get_logger(),"action服务端创建！");
      // 3-1.创建动作服务端对象
      /*
      rclcpp_action::Server<ActionT>::SharedPtr create_server<ActionT, NodeT>(
        NodeT node,
        const std::string &name, 
        rclcpp_action::Server<ActionT>::GoalCallback handle_goal, 
        rclcpp_action::Server<ActionT>::CancelCallback handle_cancel, 
        rclcpp_action::Server<ActionT>::AcceptedCallback handle_accepted, 
        const rcl_action_server_options_t &options = rcl_action_server_get_default_options(), 
        rclcpp::CallbackGroup::SharedPtr group = nullptr)
      */
      server_=rclcpp_action::create_server<Progress>(this,
        "get_sum",
        std::bind(&ProgressActionServer::handle_goal,this,std::placeholders::_1,std::placeholders::_2),
        std::bind(&ProgressActionServer::handle_cancel,this,std::placeholders::_1),
        std::bind(&ProgressActionServer::handle_accepted,this,std::placeholders::_1));
    }

    // 3-2.处理提交目标值（回调函数）
    /*
    std::function<GoalResponse(const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)>;
    */
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Progress::Goal> goal ){
      (void)uuid;//未使用到该参数，防止编译器出warnning
      //业务逻辑：判断提交的数是否大于1，是就接受，否则拒绝
      if(goal->num<=1){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"提交的目标值必须大于1！");
        return rclcpp_action::GoalResponse::REJECT;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"提交的目标值合法！"); 
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // 3-3.处理取消请求
    /*
    std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)>;
    */
    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goalhandle){
      (void)goalhandle;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"接收到的任务取消请求！");
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    // 3-4.生成连续反馈与最终响应（回调函数）
    /*
    std::function<void (std::shared_ptr<ServerGoalHandle<ActionT>>)>;
    */
    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goalhandle){
        //1.生成连续反馈返回给客户端
        /*
        void publish_feedback(std::shared_ptr<base_interfaces_demo::action::Progress_Feedback> feedback_msg)
        */
        //首先获取目标值，然后遍历，遍历中进行累加，且每循环一次就计算进度，并作为连续反馈发布
        int num=goalhandle->get_goal()->num;
        int sum=0;
        auto feedback=std::make_shared<Progress::Feedback>();
        auto result=std::make_shared<Progress::Result>();
        //设置休眠
        rclcpp::Rate rate(1.0);
        for (int i=1;i<=num;i++){
          sum=+i;
          double progress=i/double(num);//计算进度
          feedback->progress=progress;
          goalhandle->publish_feedback(feedback);
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"连续反馈中，进度%.2f",progress);
          //判断是否接收到了取消请求
          if (goalhandle->is_canceling()){
            result->sum=sum;
            goalhandle->canceled(result);
            return;
          }
          rate.sleep();

        }
        //2.生成最终响应结果
        /*
        succeed(std::shared_ptr<base_interfaces_demo::action::Progress_Result> result_msg)
        */
        result->sum=sum;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"最终响应结果%d",sum);
        goalhandle->succeed(result);
    }
    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goalhandle){
        //新建子线程处理耗时的主逻辑实现
        std::thread(std::bind(&ProgressActionServer::execute,this,goalhandle)).detach();
    }

  };
int main(int argc, char ** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<ProgressActionServer>());
  rclcpp::shutdown();
  return 0;
}