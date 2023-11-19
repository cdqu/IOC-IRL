#include "rclcpp/rclcpp.hpp"
# include "std_msgs/msg/string.hpp"//导入消息类型
// # include "std_msgs/msg/u_int32.hpp"//导入发布者消息类型
using std::placeholders::_1;//占一个位置
using namespace std::chrono_literals;
class SolveNode:public rclcpp::Node{//自定义节点继承public rclcpp::Node

public:
//
//构造函数
//
//
    SolveNode(std::string name):Node(name), count(0){
        //RCLCPP_INFO(this->get_logger(),"这是%s的构造函数",name.c_str());//打印节点
        puber=this->create_publisher<std_msgs::msg::String>("mytopic",10);//创建发布者
        timer = this->create_wall_timer(500ms, std::bind(&SolveNode::callback, this));//创建定时器
    }

private:
//
//回调函数
//
//
    void callback(){//回调函数
        std_msgs::msg::String mytopic;//定义主题
        mytopic.data="第"+ std::to_string(count++)+"次回调函数";//设置主题内容
        puber->publish(mytopic);//发布主题
    }
//
//定义变量
//
//

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr puber;//发布者
    rclcpp::TimerBase::SharedPtr timer;//计时器
    //const int count=1;//计数
    size_t count;
};//节点类

int main(int argc,char ** argv){//argc参数个数，argv参数数组
    rclcpp::init(argc,argv);//初始化
    auto node=std::make_shared<SolveNode>("pub_node");//新建节点对象，类型是Node指针
    RCLCPP_INFO(node->get_logger(),"这是pub节点的主函数");//打印节点
    rclcpp::spin(node);//循环节点
    rclcpp::shutdown();//关闭节点
}
