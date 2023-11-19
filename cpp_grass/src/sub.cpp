# include "rclcpp/rclcpp.hpp"//导包
# include "std_msgs/msg/string.hpp"//导入消息类型
using std::placeholders::_1;//占一个位置
class SolveNode:public rclcpp::Node{//自定义节点继承public rclcpp::Node

public:
//
//
//构造函数
//
//
    SolveNode(std::string name):Node(name){
        //RCLCPP_INFO(this->get_logger(),"这是%s的构造函数",name.c_str());//打印节点
        suber=this->create_subscription<std_msgs::msg::String>("mytopic",10,std::bind(&SolveNode::callback,this,_1));
    }
private:
//
//回调函数
//
//   
    void callback(const std_msgs::msg::String::SharedPtr mytopic){//回调函数
        RCLCPP_INFO(this->get_logger(),"这是%s的回调函数",mytopic->data.c_str());//打印节点
    }
//
//
//定义变量
//
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr suber;//订阅者
};

int main(int argc,char ** argv){//argc参数个数，argv参数数组
    rclcpp::init(argc,argv);//初始化
    auto node=std::make_shared<SolveNode>("sub_node");//新建节点对象，类型是Node指针
    RCLCPP_INFO(node->get_logger(),"这是订阅节点主函数");//打印节点
    rclcpp::spin(node);//循环节点
    rclcpp::shutdown();//关闭节点
}
