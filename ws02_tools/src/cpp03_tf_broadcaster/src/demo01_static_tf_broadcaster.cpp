/*
  需求:自实现静态坐标变换广播器，执行该程序时需要传递参数
      ros2 run包 可执行程序 x y z roll pitch yaw frame child_frame
  实现流程:
    0.判断传入参数是否合法
    1.包含头文件
    2.初始化 ROS2 客户端
    3.自定义节点类
      3-1.创建广播对象
      3-2.组织并发布数据
    4.调用spin函数，传入自定义类对象指针
    5.释放资源
*/
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

class TFStaticBroadcaster:public rclcpp::Node{
  private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
    void pub_static_tf(char * argv[]){
      //组织消息
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = this->now();//时间戳
      transform.header.frame_id = argv[7];//父坐标系
      transform.child_frame_id = argv[8];//子坐标系
      //设置偏移量
      transform.transform.translation.x = atof(argv[1]);
      transform.transform.translation.y = atof(argv[2]);
      transform.transform.translation.z = atof(argv[3]);
      //设置四元数
      //将欧拉角转换为四元数
      tf2::Quaternion qtn;
      qtn.setRPY(atof(argv[4]),atof(argv[5]),atof(argv[6]));
      transform.transform.rotation.x = qtn.x();
      transform.transform.rotation.y = qtn.y();
      transform.transform.rotation.z = qtn.z();
      transform.transform.rotation.w = qtn.w();
      //发布
      broadcaster_->sendTransform(transform);
    }
  public:
    TFStaticBroadcaster(char * argv[]):Node("tf_static_broadcaster_node_cpp"){
      //3-1.创建广播对象
      broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
      //3-2.组织并发布数据
      pub_static_tf(argv);
    }
  };
int main(int argc, char ** argv){
  //0.判断传入参数是否合法
  if (argc != 9){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"传入参数不合法！");
    return 0;
  }
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<TFStaticBroadcaster>(argv));
  rclcpp::shutdown();
  return 0;
}