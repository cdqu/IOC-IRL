// 接收感知雷达传来的障碍物信息，对比已有地图，若有新障碍则发布消息
// 这一步中新出现的障碍会被拆成1*1单元格，传给astar节点

# include "rclcpp/rclcpp.hpp"
# include "std_msgs/msg/string.hpp"
#include <sstream>
# include "message_interfaces/msg/obstacle.hpp"
# include "message_interfaces/msg/astarob.hpp"
# include "message_interfaces/msg/mytraj.hpp"
# include "message_interfaces/msg/mymap.hpp"
# include <map>
using std::placeholders::_1;//占一个位置

const int column = 10;  //column
const int row = 10;  //row
char my_map[column][row];

const char ROAD = '*';
const char WALL = '#';
class Point2D  //平面点类(x,y)
{
public:
    Point2D(double pos_x, double pos_y)
    {
        x = pos_x;
        y = pos_y;
    };
    double x = 0;
    double y = 0;
};

struct obstacle  //障碍物
{
    Point2D left_lower_pos = Point2D(0, 0);  //左下角格子坐标
    int ob_width = 0;
    int ob_height = 0;
    bool if_at_bound = false;  
    int num = 0;  //标号
};


class SolveNode:public rclcpp::Node //自定义节点继承public rclcpp::Node
{
public:
//
//构造函数
//
    SolveNode(std::string name):Node(name){
        check_pub=this->create_publisher<message_interfaces::msg::Astarob>("astar_ob",10);//创建发布者
        check_sub=this->create_subscription<message_interfaces::msg::Astarob>("obs_2d",20,std::bind(&SolveNode::callback,this,_1));
        //timer = this->create_wall_timer(1000ms, std::bind(&SolveNode::callback, this));//创建定时器 publish every xx time
    }

private:
//
//回调函数
//
    //check the obs
    void callback(const message_interfaces::msg::Astarob::ConstPtr& msg)
    {
        std::vector<message_interfaces::msg::Obstacle> obset_, new_obset;
        obset_ = msg->ob_set;
        int obNum = obset_.size();
        std::vector<Point2D> obstacal_Point, obstacle_Size;
        for (int j = 0; j < obNum; j++)
        {
            obstacal_Point.push_back(Point2D(obset_[j].left_lower_pos_x, obset_[j].left_lower_pos_y)); 
            obstacle_Size.push_back(Point2D(obset_[j].ob_width, obset_[j].ob_height));
        }
        int ob_width, ob_height;
        for (int i = 0; i < obstacal_Point.size(); i++)
        {
            ob_width = obstacle_Size[i].x;
            ob_height = obstacle_Size[i].y;
            // cout << i << endl;
            for (int m = 0; m < ob_width; m++)
            {
                for (int n = 0; n < ob_height; n++)
                {
                    if(my_map[int(obstacal_Point[i].x + m)][int(obstacal_Point[i].y + n)] == WALL) //已知障碍
                    {
                        std::cout << "known obj" << std::endl;
                    }
                    else //新障碍
                    {
                        // cout << m << n << endl;
                        message_interfaces::msg::Obstacle new_ob;
                        new_ob.left_lower_pos_x = int(obstacal_Point[i].x + m);
                        new_ob.left_lower_pos_y = int(obstacal_Point[i].y + n);
                        new_ob.ob_height = 1;
                        new_ob.ob_width = 1;
                        new_ob.if_at_bound = false;
                        new_obset.push_back(new_ob);
                        std::cout << new_ob.left_lower_pos_x << new_ob.left_lower_pos_y << new_ob.ob_height << new_ob.ob_width << std::endl;

                        my_map[int(obstacal_Point[i].x + m)][int(obstacal_Point[i].y + n)] == WALL; //update the map
                    }
                }
            }
        }
        if(!new_obset.empty()) //有新障碍，非空，发消息
        {
            message_interfaces::msg::Astarob new_astar_ob;
            new_astar_ob.ob_set = new_obset;
            check_pub->publish(new_astar_ob);
            std::cout << "new_ob published" << std::endl;
        }
    }
//
//定义puber suber
//
    rclcpp::Publisher<message_interfaces::msg::Astarob>::SharedPtr check_pub;
    rclcpp::Subscription<message_interfaces::msg::Astarob>::SharedPtr check_sub;
};//节点类

int main(int argc,char ** argv){//argc参数个数，argv参数数组
    rclcpp::init(argc,argv);//初始化
    auto node=std::make_shared<SolveNode>("check_ob");//新建节点对象，类型是Node指针
    RCLCPP_INFO(node->get_logger(),"check the different obstacles");//打印节点
    rclcpp::spin(node);//循环节点
    rclcpp::shutdown();//关闭节点
}
