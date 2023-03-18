/*
流程：
1.包含头文件
2.初始化ros2客户端
3.输出日志
4.输出日志
5.释放资源
*/
#include "rclcpp/rclcpp.hpp"
#include <string.h>
using namespace rclcpp;

class MyNode: public rclcpp::Node
{
public:
MyNode():Node("hello_node_cpp"){
	RCLCPP_INFO(this->get_logger(),"hello world!");
}
};


int main(int argc, char const *argv[])
{
	//2.初始化ros2客户端
	init(argc,argv);
	//3.创建节点指针
	auto node=std::make_shared<MyNode>();
	//4.释放资源
	shutdown();
	/* code */
	return 0;
}


/* int main(int argc, char ** argv)
{
	//2.初始化ros2客户端
	init(argc,argv);
	//3.创建节点指针
	auto node=rclcpp::Node::make_shared("helloworld_node_cpp");
	//4.输出日志
	RCLCPP_INFO(node->get_logger(),"hello world!");
	//5.释放资源
	shutdown();
  	return 0;
} */
