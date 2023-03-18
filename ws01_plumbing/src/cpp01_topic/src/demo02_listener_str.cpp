/*
    需求：订阅发布方发布的消息，并在终端输出
    流程:
        1.包含头文件 
        2.初始化 ROS2 客户端
        3.自定义节点类
            3-1.创建订阅方
            3-2.解析并输出数据
        4.调用spin函数，传入自定义类对象指针
        5.释放资源
*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Listener:public rclcpp::Node{
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    void do_cb(const std_msgs::msg::String &msg){
        // 3-2.解析并输出数据
        RCLCPP_INFO(this->get_logger(),"订阅到的消息是%s",msg.data.c_str());
    }
public:
    Listener():Node("listener_node_cpp"){
        RCLCPP_INFO(this->get_logger(),"订阅方创建");
        // 3-1.创建订阅方
        /*
            模板：消息类型；
            参数：
                1.话题名称（和发布方保持一致）
                2.Qss(消息队列长度)
                3.回调函数
            返回值：订阅对象指针
        */
        subscription_=this->create_subscription<std_msgs::msg::String>("topic",10,std::bind(&Listener::do_cb,this,std::placeholders::_1));
    };
};


int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Listener>());
    rclcpp::shutdown();
    return 0;
}
