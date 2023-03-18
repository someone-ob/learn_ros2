/*
  需求:广播laser->base_link的坐标系相对关系，然后发布point->laser的坐标，
      求解point->base_link的坐标
  实现流程:
    1.包含头文件
    2.初始化 ROS2 客户端
    3.自定义节点类
      3-1.创建坐标变换监听器；
      3-2.创建坐标点消息订阅方；
      3-3.创建过滤器，解析数据。
    4.调用spin函数，传入自定义类对象指针
    5.释放资源
*/
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "message_filters/subscriber.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class PointTFListener:public rclcpp::Node{
  private:
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    std::shared_ptr<tf2_ros::CreateTimerROS> timer_;
    message_filters::Subscriber<geometry_msgs::msg::PointStamped> point_sub_;
    std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>> filter_;
    void transform_point(const geometry_msgs::msg::PointStamped& ps){
      //实现坐标点变换
      auto out = buffer_->transform(ps,"base_link");

    }
  public:
    PointTFListener():Node("point_tf_listener_node_cpp"){
        RCLCPP_INFO(this->get_logger(),"创建发布节点！");
        // 3-1.创建坐标变换监听器；
        buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        //CreateTimerROS(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base, 
        //rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers, 
        //rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
        timer_ = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(),
            this->get_node_timers_interface());
        buffer_->setCreateTimerInterface(timer_);
        listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_,this);
        // 3-2.创建坐标点消息订阅方；
        point_sub_.subscribe(this,"point");
        // 3-3.创建过滤器
        /*
          MessageFilter(
          F & f, 订阅对象
          BufferT & buffer, 坐标监听的缓存
          const std::string & target_frame, 即base_link
          uint32_t queue_size, 设为10
          const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & node_logging,
          const rclcpp::node_interfaces::NodeClockInterface::SharedPtr & node_clock,
          std::chrono::duration<TimeRepT, TimeT> buffer_timeout = std::chrono::duration<TimeRepT, TimeT>::max())
        */
        filter_ = std::make_shared<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>>(
          point_sub_,
          *buffer_,
          "base_link",
          10,
          this->get_node_logging_interface(),
          this->get_node_clock_interface(),
          1s);
          //解析数据
          filter_->registerCallback(&PointTFListener::transform_point,this);
    }
  };
int main(int argc, char ** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<PointTFListener>());
  rclcpp::shutdown();
  return 0;
}