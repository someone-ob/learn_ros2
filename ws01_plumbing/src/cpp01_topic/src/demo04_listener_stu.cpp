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
#include "base_interfaces_demo/msg/student.hpp"
using namespace base_interfaces_demo::msg;

class ListenerStu:public rclcpp::Node{
private:
    rclcpp::Subscription<Student>::SharedPtr subscription_;
        void do_cb(const Student &student){
        // 3-2.解析并输出数据
        RCLCPP_INFO(this->get_logger(),"订阅到的消息:(%d,%.2f,%s)",student.age,student.height,student.name.c_str());
    }
public:
    ListenerStu():Node("listener_node_cpp"){
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
        subscription_=this->create_subscription<Student>("chatter_stu",10,std::bind(&ListenerStu::do_cb,this,std::placeholders::_1));
    };
};


int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ListenerStu>());
    rclcpp::shutdown();
    return 0;
}
