/*
  需求:先发布laser到base_link的坐标相对关系，在发布camera到base_link的相对坐标关系，
      求解laser到camera的相对坐标关系
  实现流程:
    1.包含头文件
    2.初始化 ROS2 客户端
    3.自定义节点类
      3-1.创建一个缓存对象，融合多个坐标系相对关系为一棵坐标树；
      3-2.创建一个监听器，绑定缓存对象，会将所有广播器广播的数据写入缓存；
      3-3.编写定时器，循环实现转换
    4.调用spin函数，传入自定义类对象指针
    5.释放资源
*/
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class TFListener:public rclcpp::Node{
  private:
    std::unique_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    void on_timer(){
      //实现坐标转换
      //lookupTransform(const std::string &target_frame, const std::string &source_frame, const tf2::TimePoint &time) const
      try
      {
        auto ts = buffer_->lookupTransform("camera","laser",tf2::TimePointZero);
      }
      catch(const tf2::LookupException& e)
      {
        std::cerr << e.what() << '\n';
      }
      
      
    }
  public:
    TFListener():Node("tf_listener_node_cpp"){
      RCLCPP_INFO(this->get_logger(),"创建发布节点！");
      // 3-1.创建一个缓存对象，融合多个坐标系相对关系为一棵坐标树；
      buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      // 3-2.创建一个监听器，绑定缓存对象，会将所有广播器广播的数据写入缓存；
      listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_,this);
      // 3-3.编写定时器，循环实现转换
      timer_ = this->create_wall_timer(1s,std::bind(&TFListener::on_timer,this));
    }
  };
int main(int argc, char ** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<TFListener>());
  rclcpp::shutdown();
  return 0;
}