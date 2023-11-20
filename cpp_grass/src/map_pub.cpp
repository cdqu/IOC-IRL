#include "rclcpp/rclcpp.hpp"
# include "std_msgs/msg/string.hpp"//导入消息类型
# include "message_interfaces/msg/mymap.hpp"
# include "message_interfaces/msg/obstacle.hpp"

using std::placeholders::_1;//占一个位置
using namespace std::chrono_literals;
class SolveNode:public rclcpp::Node //自定义节点继承public rclcpp::Node
{
public:
//
//构造函数
//
    SolveNode(std::string name):Node(name), count(0){
        puber=this->create_publisher<message_interfaces::msg::Mymap>("map_2d",10);//创建发布者
        timer = this->create_wall_timer(1000ms, std::bind(&SolveNode::callback, this));//创建定时器 publish every xx time
    }

private:
//
//回调函数
//
    void callback()
    {
        // std_msgs::msg::String mytopic;
        // mytopic.data="第"+ std::to_string(count++)+"次回调函数"; //define publish message
        message_interfaces::msg::Obstacle ob1, ob2;
        
        ob1.left_lower_pos_x = 2;
        ob1.left_lower_pos_y = 2;
        ob1.ob_height = 1;
        ob1.ob_width = 1;
        ob1.num = 0;
        ob1.if_at_bound = false;

        ob2.left_lower_pos_x = 5;
        ob2.left_lower_pos_y = 6;
        ob2.ob_height = 2;
        ob2.ob_width = 1;
        ob2.num = 1;
        ob2.if_at_bound = false;

        std::vector<message_interfaces::msg::Obstacle> obset;
        obset.push_back(ob1);
        obset.push_back(ob2);

        message_interfaces::msg::Mymap msg;
        msg.map_height = 10;
        msg.map_width = 10;
        msg.ob_num = 2;
        msg.ob_set = obset;
        puber->publish(msg); //发布主题 publish in the callback func, according to xx time
        std::cout<<"time:"<<count<<std::endl;
        count++;
    }
//
//定义变量
//
    rclcpp::Publisher<message_interfaces::msg::Mymap>::SharedPtr puber;//发布者
    rclcpp::TimerBase::SharedPtr timer;//计时器
    size_t count;
};//节点类

int main(int argc,char ** argv){//argc参数个数，argv参数数组
    rclcpp::init(argc,argv);//初始化
    auto node=std::make_shared<SolveNode>("map_pub");//新建节点对象，类型是Node指针
    RCLCPP_INFO(node->get_logger(),"map publish");//打印节点
    rclcpp::spin(node);//循环节点
    rclcpp::shutdown();//关闭节点
}
