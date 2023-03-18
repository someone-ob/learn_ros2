/*
  需求:启动turtlesim_node节点，编写程序，发布乌龟（turtle1）相对于窗体（world）的位姿
  实现流程:
    1.包含头文件
    2.初始化 ROS2 客户端
    3.自定义节点类
      3-1.创建一个动态广播器
      3-2.创建一个乌龟位姿订阅方
      3-3.回调函数中，获取乌龟位姿并生成相对关系然后发布
    4.调用spin函数，传入自定义类对象指针
    5.释放资源
*/
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

class TFDynaBroadcaster:public rclcpp::Node{
  private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    // 3-3.回调函数中，获取乌龟位姿并生成相对关系然后发布
    void do_pose(const turtlesim::msg::Pose & pose){
        //组织消息
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp = this->now();//时间戳
        ts.header.frame_id = "world";//父坐标系
        ts.child_frame_id = "turtle1";//子坐标系
        ts.transform.translation.x = pose.x;
        ts.transform.translation.y = pose.y;
        ts.transform.translation.z = 0.0;
        //从欧拉角转换出四元数
        //乌龟的欧拉角只有yaw
        tf2::Quaternion qtn;
        qtn.setRPY(0,0,pose.theta);
        ts.transform.rotation.x = qtn.x();
        ts.transform.rotation.y = qtn.y();
        ts.transform.rotation.z = qtn.z();
        ts.transform.rotation.w = qtn.w();
        broadcaster_->sendTransform(ts);

    }
  public:
    TFDynaBroadcaster():Node("tf_dyna_broadcaster_node_cpp"){
      RCLCPP_INFO(this->get_logger(),"创建发布节点！");
      // 3-1.创建一个动态广播器
      broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
      // 3-2.创建一个乌龟位姿订阅方
      pose_sub_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose",10,std::bind(&TFDynaBroadcaster::do_pose,this,std::placeholders::_1));
    
    }
  };
int main(int argc, char ** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<TFDynaBroadcaster>());
  rclcpp::shutdown();
  return 0;
}