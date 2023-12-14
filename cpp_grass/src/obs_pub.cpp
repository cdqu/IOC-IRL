//雷达检测环境中的障碍物
//node: obs_pub
//topic: obs_2d
//msg: obstacles
//从雷达数据转换为栅格数据，写成标准障碍物格式发布
# include "rclcpp/rclcpp.hpp"
# include "std_msgs/msg/string.hpp"//导入消息类型
# include "std_msgs/msg/float64_multi_array.hpp"
# include "message_interfaces/msg/astarob.hpp"
# include "message_interfaces/msg/obstacle.hpp"
#include <sstream>

using std::placeholders::_1;//占一个位置
using namespace std::chrono_literals;
std::vector<message_interfaces::msg::Obstacle> obset;
double gps_x1, gps_x2, gps_y1, gps_y2;
int map_x1, map_x2, map_y1, map_y2;
double x_0 = 0;
double y_0 = 0;
double dtheta = 0;

std::vector<double> find_xy(double _arr[4])
{
  double val1 = _arr[0];
  int j = 1;
  while (val1 == _arr[j] && j<4)
  {
    j++;
  }
  double val2 = _arr[j];
  std::vector<double> min_max;
  if (val1 < val2)
  {
    min_max.push_back(val1);
    min_max.push_back(val2);
  }
  else
  {
    min_max.push_back(val2);
    min_max.push_back(val1);
  }
  return min_max;
}


class SolveNode:public rclcpp::Node //自定义节点继承public rclcpp::Node
{
public:
//
//构造函数
//
    SolveNode(std::string name):Node(name){
        puber=this->create_publisher<message_interfaces::msg::Astarob>("obs_2d",10);//创建发布者
        suber=this->create_subscription<std_msgs::msg::Float64MultiArray>("rader_detection",20,std::bind(&SolveNode::callback,this,_1));
        //timer = this->create_wall_timer(1000ms, std::bind(&SolveNode::callback, this));//创建定时器 publish every xx time
    }

private:
//
//回调函数
//
    void callback(const std_msgs::msg::Float64MultiArray::ConstPtr& msg)
    {
        int obs_num = msg->data.size() / 9;
        std::cout << obs_num << std::endl;
        for (int i = 0; i < obs_num; i++)
        {
            double arr_x[4] = {msg->data[9 * i + 0], msg->data[9 * i + 2], msg->data[9 * i + 4], msg->data[9 * i + 6]};
            double arr_y[4] = {msg->data[9 * i + 1], msg->data[9 * i + 3], msg->data[9 * i + 5], msg->data[9 * i + 7]};
            // cout << arr_x[0] << " " << arr_x[1] << " " << arr_x[2] << " " << arr_x[3] << endl;
            //确定坐标(x1,y1是左下角，min，x2,y2是右上角，max)
            std::vector<double> x_min_max = find_xy(arr_x);
            std::vector<double> y_min_max = find_xy(arr_y);
            gps_x1 = x_min_max[0];
            gps_x2 = x_min_max[1];
            gps_y1 = y_min_max[0];
            gps_y2 = y_min_max[1];
            // cout << gps_x1 << gps_x2 << endl;
            // cout << gps_y1 << gps_y2 << endl;
            //坐标转换gps->栅格
            map_x1 = int(floor((gps_x1 - x_0) * cos(dtheta) + (gps_y1 - y_0) * sin(dtheta)));
            map_y1 = int(floor(-(gps_x1 - x_0) * sin(dtheta) + (gps_y1 - y_0) * cos(dtheta)));
            map_x2 = int(ceil((gps_x2 - x_0) * cos(dtheta) + (gps_y2 - y_0) * sin(dtheta)));
            map_y2 = int(ceil(-(gps_x2 - x_0) * sin(dtheta) + (gps_y2 - y_0) * cos(dtheta)));


            //标准障碍设置
            message_interfaces::msg::Obstacle ob;
            ob.left_lower_pos_x = map_x1;
            ob.left_lower_pos_y = map_y1;
            ob.ob_height = map_y2 - map_y1;
            ob.ob_width = map_x2 - map_x1;
            ob.num = i;
            ob.if_at_bound = false;
            obset.push_back(ob);
            std::cout << ob.left_lower_pos_x << ob.left_lower_pos_y << ob.ob_height << ob.ob_width << std::endl;
        }

        message_interfaces::msg::Astarob ob_msg;
        ob_msg.ob_set = obset;
        ob_msg.endx = -1;
        ob_msg.endy = -1;

        puber->publish(ob_msg);
        std::cout << "as_ob published" << std::endl;
        obset.clear();
    }
//
//定义puber suber
//
    rclcpp::Publisher<message_interfaces::msg::Astarob>::SharedPtr puber;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr suber;
};//节点类

int main(int argc,char ** argv){//argc参数个数，argv参数数组
    rclcpp::init(argc,argv);//初始化
    auto node=std::make_shared<SolveNode>("obs_pub");//新建节点对象，类型是Node指针
    RCLCPP_INFO(node->get_logger(),"obstacle publish");//打印节点
    rclcpp::spin(node);//循环节点
    rclcpp::shutdown();//关闭节点
}
