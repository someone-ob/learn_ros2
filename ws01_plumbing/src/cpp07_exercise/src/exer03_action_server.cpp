/*
  需求:处理客户端发送的请求数据（目标点），控制乌龟向目标点运动，且要连续反馈剩余距离
  实现流程:
    1.包含头文件
    2.初始化 ROS2 客户端
    3.自定义节点类
      3-1.创建乌龟位姿订阅方,获取当前乌龟坐标
      3-2.创建速度指令发布方，控制乌龟的运动
      3-3.创建动作服务端
      3-4.解析动作客户端提交的数据
      3-5.处理客户端的取消请求
      3-6.实现主逻辑（耗时操作），启动子线程；
      3-7.子线程中产生连续反馈，并相应最终结果
    4.调用spin函数，传入自定义类对象指针
    5.释放资源
*/
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/nav.hpp"

using namespace base_interfaces_demo::action;
using std::placeholders::_1;
using std::placeholders::_2;

class Exer03ActionServer:public rclcpp::Node{
  private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp_action::Server<Nav>::SharedPtr action_server_;
    float x,y;
    void pose_callback(const turtlesim::msg::Pose &pose){
      x = pose.x;
      y = pose.y;
    }
  public:
    Exer03ActionServer():Node("exer03_action_server_node"),x(0.0),y(0.0){
      RCLCPP_INFO(this->get_logger(),"创建发布节点！");
      // 3-1.创建乌龟位姿订阅方,获取当前乌龟坐标
      sub_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose",10,std::bind(&Exer03ActionServer::pose_callback,this,_1));
      // 3-2.创建速度指令发布方，控制乌龟的运动
      cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);
      // 3-3.创建动作服务端
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
      action_server_ = rclcpp_action::create_server<Nav>(this,
        "nav",
        std::bind(&Exer03ActionServer::handle_goal,this,std::placeholders::_1,std::placeholders::_2),
        std::bind(&Exer03ActionServer::handle_cancel,this,std::placeholders::_1),
        std::bind(&Exer03ActionServer::handle_accepted,this,std::placeholders::_1));
    }
    // 3-4.解析动作客户端提交的数据
    /*
    std::function<GoalResponse(const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)>;
    */
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Nav::Goal> goal ){
      (void)uuid;//未使用到该参数，防止编译器出warnning
      //取出目标中的x y，判断是否超出[0,11.08],如果超出，则非法
      if(goal->goal_x<0||goal->goal_x>11.08||goal->goal_y<0||goal->goal_y>11.08){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"提交的目标值非法！");
        return rclcpp_action::GoalResponse::REJECT;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"提交的目标值合法！"); 
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    // 3-5.处理客户端的取消请求
    /*
    std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)>;
    */
    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> goalhandle){
      (void)goalhandle;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"接收到的任务取消请求！");
      return rclcpp_action::CancelResponse::ACCEPT;
    }
    // 3-6.实现主逻辑（耗时操作），启动子线程；
    /*
    std::function<void (std::shared_ptr<ServerGoalHandle<ActionT>>)>;
    */
    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> goalhandle){
      //新建子线程处理耗时的主逻辑实现
      std::thread(std::bind(&Exer03ActionServer::execute,this,goalhandle)).detach();
    }

    // 3-7.子线程中产生连续反馈，并相应最终结果
    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> goalhandle){
      //最终结果
      auto result = std::make_shared<Nav::Result>();
      auto feedback=std::make_shared<Nav::Feedback>();
      geometry_msgs::msg::Twist twist;
      //1.生成连续反馈
      rclcpp::Rate rate(1.0);
      while(true){
        //如果要取消任务，需要特殊处理
        if (goalhandle->is_canceling()){
          goalhandle->canceled(result);
          return;
        }
        //解析目标点坐标与原生乌龟坐标的距离
        float goal_x = goalhandle->get_goal()->goal_x;
        float goal_y = goalhandle->get_goal()->goal_y;
        //计算剩余距离
        float distance_x = goal_x - x;
        float distance_y = goal_y - y;
        float distance = std::sqrt(distance_x*distance_x+distance_y*distance_y);
        feedback->distance = distance;
        goalhandle->publish_feedback(feedback);
        //根据剩余距离计算速度指令并发布
        float scale = 0.5;
        float linear_x = scale * distance_x;
        float linear_y = scale * distance_y;
        twist.linear.x = linear_x;
        twist.linear.y = linear_y;
        cmd_pub_->publish(twist);
        //循环结束的条件
        if (distance <=0.05){
          break;
        }
        rate.sleep();
      }
      //3.生成最终结果
      result->turtle_x = x;
      result->turtle_y = y;
      goalhandle->succeed(result);
    } 
};
int main(int argc, char ** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<Exer03ActionServer>());
  rclcpp::shutdown();
  return 0;
}