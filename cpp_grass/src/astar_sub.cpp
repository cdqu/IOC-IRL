//astar_plan
//node: astar_plan
//sub_topic: obs_2d
//pub_topic: traj

#include <iostream>
#include <cstring>
#include <algorithm>
#include <cmath>
#include <list>
#include <stdio.h>
#include <fstream>
#include <cstdio>
#include <vector>
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
# include <deque>
# include <iomanip>
# include <string.h>

using std::placeholders::_1; //占一个位置

double x_0 = 0;
double y_0 = 0;
double dtheta = 0;

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
    bool if_at_bound = false;  //是否靠边界
    int num = 0;  //标号 从1开始
};
std::map<int, obstacle> ob_set;  //障碍物集

typedef struct node
{
	node()
	{
		x = y = 0;
		f = g = h = 0;
		parent = NULL;
	}
	int x, y;
	double f, g, h;
	struct node *parent;
} Node;
const double LEN = 10;
const int column = 10;  //column
const int row = 10;  //row
char my_map[column][row];
const char START = '0';
const char STOP = '1';
const char ROAD = '*';
const char WALL = '#';
    
std::list<Node*> startList, stopList;  

int sx, sy, ex, ey, flag;   
typedef struct Point
{
	double x;
	double y;
};
std::vector<Point> path; 
std::vector<Point> optPath;
std::vector<message_interfaces::msg::Obstacle> obset_;

int dx[8] = {-1,1,0,-1,1,0,-1,1};
int dy[8] = {0,0,-1,-1,-1,1,1,1};


double getDis(int x1, int y1, int x2, int y2)
{
	double xx1 = x1 * LEN + LEN / 2.0;
	double yy1 = y1 * LEN + LEN / 2.0;

	double xx2 = x2 * LEN + LEN / 2.0;
	double yy2 = y2 * LEN + LEN / 2.0;

	return sqrt((xx1 - xx2)*(xx1 - xx2) + (yy1 - yy2)*(yy1 - yy2));
}


bool in_List(Node *pnode, std::list<Node*> mlist)
{
	for (std::list<Node*>::iterator it = mlist.begin(); it != mlist.end(); it++)
	{
		if (pnode->x == (*it)->x&&pnode->y == (*it)->y)
			return true;
	}
	return false;
}

bool del(Node *pnode, std::list<Node*> &mlist)
{
	for (std::list<Node*>::iterator it = mlist.begin(); it != mlist.end(); it++)
	{
		if (pnode == (*it))
		{
			mlist.erase(it);
			return true;
		}
	}
	return false;
}

void add(Node *pnode, std::list<Node*> &mlist)
{
	mlist.push_back(pnode);
	return;
}

Node* getMin(std::list<Node*> mlist)
{
	double mmin = 100000000;
	Node *temp = NULL;
	for (std::list<Node*>::iterator it = mlist.begin(); it != mlist.end(); it++)
	{
		if ((*it)->f < mmin)
		{
			mmin = (*it)->f;
			temp = (*it);
		}
	}
	return temp;
}

void setRoad(Node *root)
{
	Point p;
	while (root->parent != NULL)
	{
		if (root->x == ex && root->y == ey)
		{
			my_map[root->x][root->y] = STOP;
		}
		else
		{
			my_map[root->x][root->y] = ROAD;
		}
		p.x = root->x;
		p.y = root->y;
		path.push_back(p);
		root = root->parent;
	}
	p.x = sx;
	p.y = sy;
	path.push_back(p);
}


void bfs()
{
	startList.clear();
	stopList.clear();

	Node *preNode = new Node;
	preNode->x = sx;
	preNode->y = sy;
	preNode->g = 0;
	preNode->h = getDis(sx, sy, ex, ey);
	preNode->f = preNode->g + preNode->h;
	preNode->parent = NULL;
	add(preNode, startList);

	while (!startList.empty())  //OpenList
	{
		preNode = getMin(startList);
		if (preNode == NULL)
		{
			std::cout << "end" << std::endl;
			return;
		}
		del(preNode, startList);
		add(preNode, stopList);
		for (int d = 0; d < 8; d++)
		{
			int cx = preNode->x + dx[d];
			int cy = preNode->y + dy[d];

			Node *curNode = new Node;
			curNode->x = cx;
			curNode->y = cy;
			curNode->g = preNode->g + getDis(cx, cy, preNode->x, preNode->y);
			curNode->h = getDis(cx, cy, ex, ey);
			curNode->f = curNode->g + curNode->h;
			curNode->parent = preNode;

			if (cx < 0 || cy < 0 || cx >= row || cy >= column) continue;
			else if (my_map[cx][cy] == WALL) continue;
			else if (in_List(curNode, startList) || in_List(curNode, stopList)) continue;

			if (cx == ex && cy == ey)
			{
				setRoad(curNode);
				return;
			}
			add(curNode, startList);
		}
	}
	std::cout << "failed" << std::endl;
	return;
}

double map_x, map_y;


//////////////// node definition ////////////////
class SolveNode:public rclcpp::Node //自定义节点继承public rclcpp::Node
{
public:
//构造函数
    SolveNode(std::string name):Node(name){
        //RCLCPP_INFO(this->get_logger(),"这是%s的构造函数",name.c_str());//打印节点
        suber=this->create_subscription<message_interfaces::msg::Astarob>("astar_ob",20,std::bind(&SolveNode::callback,this,_1));
        as_pub=this->create_publisher<message_interfaces::msg::Astarob>("replan",10); //trajectory publisher
    }

private:
//回调函数 
    void callback(const message_interfaces::msg::Astarob::ConstPtr& msg)
    {
        //get the current pos
        double start_x, start_y, next_x, next_y;
        std::ifstream infile("/home/qucd/point.txt");
        std::string s;
        if (!infile.is_open()){
            std::cout << "can not open this file" << std::endl;
        }
        else
        {
            while (getline(infile, s))
            {
                //边读边写到astar_point文件
                //outfile << s;
                if (s.find("true") == std::string::npos) //not found
                {
                    for (int i = 0; i < s.size(); i++)
                    {
                        if (s[i] == ','){
                            s[i] = ' ';
                        }
                    }
                    std::istringstream out(s);
                    out >> start_x;
                    out >> start_y;

                    getline(infile, s); //再往下读一行，判断是朝上/下
                    for (int i = 0; i < s.size(); i++)
                    {
                        if (s[i] == ','){
                            s[i] = ' ';
                        }
                    }
                    std::istringstream out_n(s);
                    out_n >> next_x;
                    out_n >> next_y;
                    break;
                }
            }
            std::cout << "the current pos:" << start_x << " " << start_y << std::endl;
        }
        infile.close();

        sx = ceil(start_x);
        sy = ceil(start_y); //当前坐标&a*起点
        ex = sx;
        ey = sy;


        // new obs
        obset_ = msg->ob_set;
        int obNum = obset_.size();
        std::vector<Point2D> obstacal_Point, obstacle_Size;
        for (int j = 0; j < obNum; j++)
        {
            obstacal_Point.push_back(Point2D(obset_[j].left_lower_pos_x, obset_[j].left_lower_pos_y)); //障碍左下角方格坐标
            obstacle_Size.push_back(Point2D(obset_[j].ob_width, obset_[j].ob_height));  //障碍物大小:宽度和高度
        }

        // whether the new ob block my way (same x?)
        int if_block = 0;  //flag: 0 no 1 yes
        for (int i = 0; i < obstacal_Point.size(); i++)
        {
            if(obstacal_Point[i].x == sx)
            {
                if_block = 1; // yes, it block --> astar
            }
            my_map[int(obstacal_Point[i].x)][int(obstacal_Point[i].y)] = WALL;  //update (all 1*1 ob)
        }  //else,no --> just update the map and traject

        if(if_block == 1)// yes, it block --> astar
        {
            //确定终点
            if(next_y>start_y)  //朝上走
            {
                int find_y = sy + 1;
                while (my_map[sx][find_y] == WALL) //找到下一个可行点
                {
                    find_y++;
                }
                ey = find_y;
            }
            else  //朝下走
            {
                int find_y = sy - 1;
                while (my_map[sx][find_y] == WALL)
                {
                    find_y--;
                }
                ey = find_y;
            }

            bfs();
            double the;
            //cout << "detect new obs" << endl;
            double out_x, out_y;
            //往astar_point文件写入新规划的a*
            std::ofstream outfile;
            outfile.open("/home/fins/point.txt", std::ios::out | std::ios::binary);
            for (int k = path.size() - 1; k >= 0; k--)
            {
                the = atan2(path[k].y - path[k + 1].y, -path[k].x + path[k + 1].x) / M_PI * 180;
                map_x = path[k].x * cos(dtheta* M_PI / 180) - path[k].y * sin(dtheta* M_PI / 180) + x_0;
                map_y = path[k].x * sin(dtheta* M_PI / 180) + path[k].y * cos(dtheta* M_PI / 180) + y_0;
                outfile << map_x << "," << map_y << "," << the + dtheta << ","
                        << 0 << ","
                        << "false;" << std::endl;
            }
            outfile.close();
            std::cout << "block my way! astar point added" << std::endl;

            path.clear();
            //主程序重新规划一次
            message_interfaces::msg::Astarob endpoint;
            endpoint.endx = ex;
            endpoint.endy = ey;
            endpoint.ob_set = obset_;
            as_pub->publish(endpoint);
            std::cout << "replan pub end:" << ex << ey << std::endl;
        }
        else //else --> just update the map and traject
        {
            //主程序重新规划一次
            message_interfaces::msg::Astarob new_map;
            new_map.endx = sx; //当前点为起点
            new_map.endy = sy;
            new_map.ob_set = obset_;
            as_pub->publish(new_map);
            std::cout << "no block, replan pub" << std::endl;
        }
    }
//定义变量
    rclcpp::Subscription<message_interfaces::msg::Astarob>::SharedPtr suber;//订阅者
    rclcpp::Publisher<message_interfaces::msg::Astarob>::SharedPtr as_pub;
};


//////////////// main function ////////////////
int main(int argc,char ** argv) //argc参数个数，argv参数数组
{
    rclcpp::init(argc,argv);//初始化
    auto node=std::make_shared<SolveNode>("astar_plan");//新建节点对象，类型是Node指针
    RCLCPP_INFO(node->get_logger(),"this is astar planning");//打印节点
    rclcpp::spin(node);//循环节点
    rclcpp::shutdown();//关闭节点
}
