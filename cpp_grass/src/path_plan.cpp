#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float32.hpp"
# include "message_interfaces/msg/obstacle.hpp"
# include "message_interfaces/msg/astarob.hpp"
# include "message_interfaces/msg/mytraj.hpp"
# include "message_interfaces/msg/mymap.hpp"
# include <iostream>
# include <map>
# include <vector>
# include <deque>
# include <algorithm>
# include <iomanip>
# include <cmath>
# include <fstream>

using std::placeholders::_1; //占一个位置

// 定义为全局变量
int flag, flag_as, flag1;
double x_0 = 0;
double y_0 = 0;
double dtheta = 0;

// planning section
int MAP_HEIGHT;
int MAP_WIDTH;
#define M_PI  3.14159265358979

//算法建立在网格地图上，首先对地图进行离散化，形成n*m的网格
//网格宽为一个车宽；高为1-2个车长
//由于算法设计，需要在上下两边留出两条高为转弯半径r的长条区域，留出转弯空间

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

class Pos3D  //轨迹点类(x,y,theta)
{
public:
    Pos3D(double pos_x, double pos_y, double pos_theta, int pos_ori)
    {
        x = pos_x;
        y = pos_y;
        theta = pos_theta;
        ori = pos_ori;
    };

    double x = 0;
    double y = 0;
    double theta = 0;
    int ori = 0; // 0:forward 1:back
};

struct mapPoint  //地图结构
{
    int occupancy = 0;  //0可通行，>=1为不同障碍
    bool isVisited = false;
    int visitedNum = 0;  //访问次数

    Point2D next_point_position = Point2D(-1, -1);
    Point2D prev_point_position = Point2D(-1, -1);
};
std::map<Point2D, mapPoint> my_map;  //地图

bool operator<(const Point2D& p1, const Point2D& p2) {
    return (p1.x < p2.x) || ((p1.x == p2.x) && (p1.y < p2.y));
}

bool operator == (const Point2D& p1, const Point2D& p2)
{
    return (p1.x == p2.x && p1.y == p2.y);
}

struct obstacle  //障碍物
{
    Point2D left_lower_pos = Point2D(0, 0);  //左下角格子坐标
    int ob_width = 0;
    int ob_height = 0;
    bool if_at_bound = false;  //是否靠边界
    int num = 0;  //标号 从1开始
};
std::map<int, obstacle> ob_set;  //障碍物集


std::vector<Point2D> visited_Point;  //访问过的点集
std::vector<Pos3D> opPath;  //2D点路径
std::vector<Pos3D> Traj;  //曲线轨迹

Pos3D getUpper(Pos3D root_position)  //获取上方邻居位置
{
    Pos3D upper_positions = Pos3D(root_position.x, root_position.y + 1, 90, 0);
    return upper_positions;
}
Pos3D getLower(Pos3D root_position)  //获取下方邻居位置
{
    Pos3D lower_positions = Pos3D(root_position.x, root_position.y - 1, -90, 0);
    return lower_positions;
}


Pos3D getRighter(Pos3D root_position)  //右
{
    Pos3D right_positions = Pos3D(root_position.x + 1, root_position.y, 0,0);
    return right_positions;
}
Pos3D getLefter(Pos3D root_position)  //左
{
    Pos3D left_positions = Pos3D(root_position.x - 1, root_position.y,180,0);
    return left_positions;
}


Pos3D add_point(Point2D root_pos, double dx, double dy, double dtheta, int dori)  //对点做偏移
{
    Pos3D res_pos = Pos3D(root_pos.x + dx, root_pos.y + dy, dtheta, dori);
    return res_pos;
}

double car_r = 2.5;  //转弯半径
double car_wid = 1.2;  //车宽
double car_len = 1.7;

void turn_ar_fst(Point2D root_pos)  //第一种模式的掉头，中途七个点
{
    opPath.push_back(add_point(root_pos, 0.134 * car_r, 0.5 * car_r, 60, 0));
    opPath.push_back(add_point(root_pos, 0.5 * car_r, 0.866 * car_r, 30, 0));
    opPath.push_back(add_point(root_pos, car_r, car_r, 0, 0));
    opPath.push_back(add_point(root_pos, car_r+0.3, car_r, 0, 0));
    opPath.push_back(add_point(root_pos, 0.5 * car_wid, car_r, 0, 1));
    opPath.push_back(add_point(root_pos, car_wid - car_r, car_r, 0, 1));
    opPath.push_back(add_point(root_pos, car_wid - car_r-0.3, car_r, 0, 1));
    opPath.push_back(add_point(root_pos, car_wid - 0.5 * car_r, 0.866 * car_r, -30, 0));
    opPath.push_back(add_point(root_pos, car_wid - 0.134 * car_r, 0.5 * car_r, -60, 0));
    return;
}

void turn_ar_scd(Point2D root_pos)  //第二种模式掉头
{
    opPath.push_back(add_point(root_pos, -0.134 * car_r, -0.5 * car_r, -150, 0));
    opPath.push_back(add_point(root_pos, -0.5 * car_r, -0.866 * car_r, -120, 0));
    opPath.push_back(add_point(root_pos, -car_r, -car_r, 180, 0));
    opPath.push_back(add_point(root_pos, -car_r-0.3, -car_r, 180, 0));
    opPath.push_back(add_point(root_pos, 0.5 * car_wid, -car_r, 180, 1));
    opPath.push_back(add_point(root_pos, car_wid + car_r, -car_r, 180, 1));
    opPath.push_back(add_point(root_pos, car_wid + car_r+0.3, -car_r, 180, 1));
    opPath.push_back(add_point(root_pos, car_wid + 0.5 * car_r, -0.866 * car_r, 150, 0));
    opPath.push_back(add_point(root_pos, car_wid + 0.134 * car_r, -0.5 * car_r, 120, 0));
    return;
}

Pos3D getover_ob(Pos3D curr_position, int dir, int ob_num_)  //绕过障碍物
{
    int right_step = ob_set[ob_num_].left_lower_pos.x + ob_set[ob_num_].ob_width - curr_position.x;
    int updown_step = ob_set[ob_num_].ob_height + 1;
    double h = sqrt(car_r * car_wid - car_wid * car_wid / 4);
    // double h = car_len / 2;
    double _theta = acos((car_r - car_wid / 2) / car_r) / M_PI * 180;
    if (dir == 0)
    {
        for (int i = 0; i < right_step; i++)  //右
        {
            if(i == 0)
            {
                opPath.push_back(Pos3D(curr_position.x + car_wid / 2, curr_position.y +h-car_len, 90 + _theta, 0));
            }
            else
            {
                // opPath.push_back(getRighter(curr_position));
            }
            curr_position = getRighter(curr_position);
        }
        for (int j = 0; j < updown_step; j++)  //上
        {
            opPath.push_back(getUpper(curr_position));
            curr_position = getUpper(curr_position);
        }
        for (int i = 0; i < right_step - 1; i++)  //左
        {
            // opPath.push_back(getLefter(curr_position));
            curr_position = getLefter(curr_position);
        }
        opPath.push_back(Pos3D(curr_position.x - car_wid / 2, (curr_position.y -h+ 0.5+car_len), 90 - _theta, 0));
        curr_position = getLefter(curr_position);
        curr_position = getUpper(curr_position);
        // cout << curr_position.y << endl;
        // curr_position = getUpper(curr_position);
    }
    else
    {
        for (int i = 0; i < right_step; i++)  //右
        {
            if(i == 0)
            {
                opPath.push_back(Pos3D(curr_position.x + car_wid / 2, curr_position.y + car_len - h, -90 - _theta, 0));
            }
            else
            {
                // opPath.push_back(getRighter(curr_position));
            }
            curr_position = getRighter(curr_position);
        }
        for (int j = 0; j < updown_step; j++)  //下
        {
            opPath.push_back(getLower(curr_position));
            curr_position = getLower(curr_position);
        }
        for (int i = 0; i < right_step - 1; i++)  //左
        {
            // opPath.push_back(getLefter(curr_position));
            curr_position = getLefter(curr_position);
        }
        opPath.push_back(Pos3D(curr_position.x - car_wid / 2, curr_position.y -(-h+ 0.5+car_len), -90 + _theta, 0));
        curr_position = getLower(curr_position);
        // curr_position = getLower(curr_position);
        curr_position = getLefter(curr_position);
    }
    return curr_position;
}



int up_or_down = 0;  //判断朝上/下走，0朝上，1朝下

void completeCoveragePathPlanning(Pos3D current_position)  //全覆盖规划
{
    Pos3D nextPosition = Pos3D(-1, -1, -1, 0);
    Point2D curr_point = Point2D(nextPosition.x, nextPosition.y);
    Point2D next_point = Point2D(nextPosition.x, nextPosition.y);

    while (curr_point.x < MAP_WIDTH)
    {
        my_map[curr_point].isVisited = true;
        opPath.push_back(current_position);

        if (up_or_down == 0) //朝上走
        {
            nextPosition = getUpper(current_position);
            next_point = Point2D(nextPosition.x, nextPosition.y);
            if (next_point.y < MAP_HEIGHT)  //没超出边界
            {
                if (my_map[next_point].occupancy == 0)  //无障碍
                {
                    nextPosition = nextPosition;  //不变
                }
                else  //有障碍
                {
                    int ob_num = my_map[next_point].occupancy;
                    opPath.pop_back();
                    nextPosition = getover_ob(current_position, up_or_down, ob_num); //往右绕过
                }
            }
            else  //超边界，转弯掉头
            {
                turn_ar_fst(curr_point);
                nextPosition = Pos3D(curr_point.x + 1, curr_point.y, -90, 0);
                up_or_down = 1 - up_or_down;
            }
        }
        else if (up_or_down == 1)
        {
            nextPosition = getLower(current_position);
            next_point = Point2D(nextPosition.x, nextPosition.y);
            if (0 <= next_point.y)  //没超出边界
            {
                if (my_map[next_point].occupancy == 0)  //无障碍
                {
                    nextPosition = nextPosition;
                }
                else
                {
                    int ob_num = my_map[next_point].occupancy;
                    opPath.pop_back();
                    nextPosition = getover_ob(current_position, up_or_down, ob_num); //往右绕过
                }
            }
            else
            {
                turn_ar_scd(curr_point);
                nextPosition = Pos3D(curr_point.x + 1, curr_point.y, 90, 0);
                up_or_down = 1 - up_or_down;
            }
        }
        current_position = nextPosition;
        curr_point = next_point;
    }

}

double map_x, map_y, map_the;


//////////////// node definition ////////////////
class SolveNode:public rclcpp::Node //自定义节点继承public rclcpp::Node
{
public:
//构造函数
    SolveNode(std::string name):Node(name){
        //RCLCPP_INFO(this->get_logger(),"这是%s的构造函数",name.c_str());//打印节点
        suber=this->create_subscription<message_interfaces::msg::Mymap>("map_2d",10,std::bind(&SolveNode::callback,this,_1));
        puber=this->create_publisher<message_interfaces::msg::Mytraj>("traj",10); //trajectory publisher
        sub_as=this->create_subscription<message_interfaces::msg::Astarob>("replan",10,std::bind(&SolveNode::callback_astar,this,_1));
        pub_to_ctrl=this->create_publisher<std_msgs::msg::String>("txt_change",10);
    }

private:
//回调函数 
    void callback(const message_interfaces::msg::Mymap::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),"have %d obstacles",msg->ob_num);//打印节点
        if(flag_as == 0)//初始规划只运行一次
        {
            MAP_HEIGHT = msg->map_height;
            MAP_WIDTH = msg->map_width;
            // 建图
            for (int i = 0; i < MAP_WIDTH; i++)
            {
                for (int j = 0; j < MAP_HEIGHT; j++)
                {
                    mapPoint mapP;
                    my_map.insert(std::make_pair(Point2D(i, j), mapP));
                }
            }

            //障碍物设置

            std::vector<message_interfaces::msg::Obstacle> obset_;
            obset_ = msg->ob_set;
            int obNum = msg->ob_num;
            std::vector<Point2D> obstacal_Point, obstacle_Size;
            for (int j = 0; j < obNum; j++)
            {
                obstacal_Point.push_back(Point2D(obset_[j].left_lower_pos_x, obset_[j].left_lower_pos_y)); //障碍左下角方格坐标
                obstacle_Size.push_back(Point2D(obset_[j].ob_width, obset_[j].ob_height));                 //障碍物大小:宽度和高度
            }
    
            for (int i = 0; i < obstacal_Point.size(); i++)
            {
                obstacle ob;
                ob.left_lower_pos = obstacal_Point[i];
                ob.ob_width = obstacle_Size[i].x;
                ob.ob_height = obstacle_Size[i].y;
                ob.num = i + 1;
                ob_set.insert(std::make_pair(i + 1, ob));

                for (int m = 0; m < ob.ob_width; m++)
                {
                    for (int n = 0; n < ob.ob_height; n++)
                    {
                        my_map[Point2D(obstacal_Point[i].x + m, obstacal_Point[i].y + n)].occupancy = i + 1;
                    }
                }
            }
        

            Pos3D start = Pos3D(0, 0, 90, 0); //起点

            completeCoveragePathPlanning(start);

            //输出
            message_interfaces::msg::Mytraj mytraj_;
            mytraj_.path_len = opPath.size();
            std::vector<float> vec_path;
            int n = 0;
            up_or_down = 0;
            Pos3D new_pos = Pos3D(-1, -1, -1, 0);
            //写入文件
            if (flag == 0)
            {
                std::ofstream outfile;
                outfile.open("/home/fins/point.txt", std::ios::out | std::ios::binary);
                for (n = 0; n < opPath.size(); n++)
                {
                    //cout << "(" << opPath[n].x << "," << opPath[n].y << ")" << endl;
                    vec_path.push_back(opPath[n].x);
                    vec_path.push_back(opPath[n].y);
                    vec_path.push_back(opPath[n].theta);
                    vec_path.push_back(opPath[n].ori);
                    // map_x = (-opPath[n].x) * car_wid * cos(dtheta* M_PI / 180) - opPath[n].y * sin(dtheta* M_PI / 180) + x_0;
                    // map_y = (-opPath[n].x) * car_wid * sin(dtheta* M_PI / 180) + opPath[n].y * cos(dtheta* M_PI / 180) + y_0;
                    map_x = opPath[n].x * car_wid * cos(dtheta* M_PI / 180) - opPath[n].y * sin(dtheta* M_PI / 180) + x_0;
                    map_y = opPath[n].x * car_wid * sin(dtheta* M_PI / 180) + opPath[n].y * cos(dtheta* M_PI / 180) + y_0;
                    //outfile << opPath[n].x << "," << opPath[n].y << "," << opPath[n].theta << ","
                    outfile << map_x << "," << map_y << "," << opPath[n].theta + dtheta << ","
                            << opPath[n].ori << ","
                            << "false;" << std::endl;
                }
                outfile.close();
                std::cout << "txt change" << std::endl;
                flag = 1;
            }
            mytraj_.my_traj = vec_path;
            puber->publish(mytraj_);

            //std::cout << "receive msg:" << obset_.size() << std::endl;
            std::cout << "traj published" << std::endl;

            opPath.clear(); //每次循环要清空
        }
        else{}
    }

    //处理突然出现的障碍
    void callback_astar(const message_interfaces::msg::Astarob::ConstPtr& end_poi)
    {
        std::vector<message_interfaces::msg::Obstacle> as_obset_;
        as_obset_ = end_poi->ob_set;
        int obNum = as_obset_.size();
        std::vector<Point2D> as_obstacal_Point, as_obstacle_Size;
        for (int j = 0; j < obNum; j++)
        {
            as_obstacal_Point.push_back(Point2D(as_obset_[j].left_lower_pos_x, as_obset_[j].left_lower_pos_y)); //障碍左下角方格坐标
            as_obstacle_Size.push_back(Point2D(as_obset_[j].ob_width, as_obset_[j].ob_height));                 //障碍物大小:宽度和高度
        }
        
        for (int i = 0; i < as_obstacal_Point.size(); i++)
        {
            obstacle as_ob;
            as_ob.left_lower_pos = as_obstacal_Point[i];
            as_ob.ob_width = as_obstacle_Size[i].x;
            as_ob.ob_height = as_obstacle_Size[i].y;
            as_ob.num = i + 1;
            ob_set.insert(std::make_pair(i + 1, as_ob));

            for (int m = 0; m < as_ob.ob_width; m++)
            {
                for (int n = 0; n < as_ob.ob_height; n++)
                {
                    my_map[Point2D(as_obstacal_Point[i].x + m, as_obstacal_Point[i].y + n)].occupancy = i + 1;
                }
            }
        }

        std::cout << "start:" << end_poi->endx << end_poi->endy << std::endl;
        Pos3D start = Pos3D(end_poi->endx, end_poi->endy, 90, 0); //起点
        completeCoveragePathPlanning(start);

        //输出
        message_interfaces::msg::Mytraj mytraj_;
        mytraj_.path_len = opPath.size();
        std::vector<float> vec_path; 
        up_or_down = 0;   //////////////////////要改?好像不用
        Pos3D new_pos = Pos3D(-1,-1,-1, 0);
        //写入文件
        std::ofstream outfile;
        outfile.open("/home/fins/point.txt", std::ios::app | std::ios::binary);
        for (int n = 0; n < opPath.size(); n++)
        {
            //cout << "(" << opPath[n].x << "," << opPath[n].y << ")" << endl;
            vec_path.push_back(opPath[n].x);
            vec_path.push_back(opPath[n].y);
            vec_path.push_back(opPath[n].theta);
            vec_path.push_back(opPath[n].ori);
            map_x = opPath[n].x * car_wid * cos(dtheta* M_PI / 180) - opPath[n].y * sin(dtheta* M_PI / 180) + x_0;
            map_y = opPath[n].x * car_wid * sin(dtheta* M_PI / 180) + opPath[n].y * cos(dtheta* M_PI / 180) + y_0;
            outfile << map_x << "," << map_y << "," << opPath[n].theta + dtheta << ","
                    << opPath[n].ori << ","
                    << "false;" << std::endl;
        }
        outfile.close();
        std::cout << "txt change" << std::endl;
        mytraj_.my_traj = vec_path;
        puber->publish(mytraj_);

        std::cout << "new_traj published" << std::endl;
        std_msgs::msg::String txt_change_msg;
        txt_change_msg.data = "true";
        pub_to_ctrl->publish(txt_change_msg);

        opPath.clear(); //每次循环要清空
    }

//定义变量
    rclcpp::Subscription<message_interfaces::msg::Mymap>::SharedPtr suber;//订阅者
    rclcpp::Publisher<message_interfaces::msg::Mytraj>::SharedPtr puber;
    rclcpp::Subscription<message_interfaces::msg::Astarob>::SharedPtr sub_as;//订阅者
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_to_ctrl;
};


//////////////// main function ////////////////
int main(int argc,char ** argv) //argc参数个数，argv参数数组
{
    rclcpp::init(argc,argv);//初始化
    flag = 0;
    flag_as = 0;
    flag1 = 0;
    auto node=std::make_shared<SolveNode>("my_planning");//新建节点对象，类型是Node指针
    RCLCPP_INFO(node->get_logger(),"这是订阅节点主函数");//打印节点
    rclcpp::spin(node);//循环节点
    rclcpp::shutdown();//关闭节点
}
