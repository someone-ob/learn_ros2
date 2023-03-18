/*
    问题：Time和Duration有什么区别？
    答：1.二者只是api使用类似
    2.二者有着本质区别：
      rclcpp::Time t2(2,500000000L);  ----指的是一个具体时刻
      rclcpp::Duration du2(2,500000000L);   ----指的是一个时间段，要持续2.5s
*/

#include <iostream>
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MyNode:public rclcpp::Node{
  private:
    void demo_rate(){
      //1.创建rate对象
      rclcpp::Rate rate1(500ms);//设置休眠时间
      rclcpp::Rate rate2(1.0);//设置执行频率
      //2.调用Rate的sleep函数
      while (rclcpp::ok()){
        RCLCPP_INFO(this->get_logger(),"-------------------");
        //rate1.sleep();
        rate2.sleep();
      }
    }

    void demo_time(){
      //1.创建 Time 对象
      rclcpp::Time t1(500000000L);
      rclcpp::Time t2(2,500000000L);
      //rclcpp::Time right_now = this->get_clock()->now();
      rclcpp::Time right_now = this->now();

      //调用 Time 对象的函数
      RCLCPP_INFO(this->get_logger(),"s = %.2f , ns = %ld", t1.seconds(),t1.nanoseconds());
      RCLCPP_INFO(this->get_logger(),"s = %.2f , ns = %ld", t2.seconds(),t2.nanoseconds());
      RCLCPP_INFO(this->get_logger(),"s = %.2f , ns = %ld", right_now.seconds(),right_now.nanoseconds());
    }

    void demo_duration(){
      //1.创建 Duration 对象
      rclcpp::Duration du1(1s);
      rclcpp::Duration du2(2,500000000L);
      //2.调用函数
      RCLCPP_INFO(this->get_logger(),"s = %.2f , ns = %ld", du1.seconds(),du1.nanoseconds());
      RCLCPP_INFO(this->get_logger(),"s = %.2f , ns = %ld", du2.seconds(),du2.nanoseconds());
    }

    void demo_opt(){
      rclcpp::Time t1(10,0);
      rclcpp::Time t2(30,0);

      rclcpp::Duration du1(8,0);
      rclcpp::Duration du2(17,0);

      //运算
      //比较运算符
      RCLCPP_INFO(this->get_logger(),"t1>=t2? %d",t1>=t2);
      //数学运算
      rclcpp::Duration du3=t2-t1;
      rclcpp::Time t3 = t1 + du1;
      rclcpp::Time t4 = t1 - du1;
    }
  public:
    MyNode():Node("time_node_cpp"){
      //demo_rate();
      //demo_time();
      //demo_duration();
      demo_opt();
    }
  };
int main(int argc, char ** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<MyNode>());
  rclcpp::shutdown();
  return 0;
}