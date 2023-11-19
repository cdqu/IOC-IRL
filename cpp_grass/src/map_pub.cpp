//map publisher
//node: map_pub
//topic: map_2d
//msg: (height, width) & obstacles

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sstream>
# include "message_interfaces/msg/obstacle.hpp"
# include "message_interfaces/msg/mymap.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("demo_node_name");
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<demo::msg>> pub_demo;
  ros::Publisher pub = n.advertise<message_interfaces::msg::Mymap>("map_2d", 10);
  rclcpp::Rate loop_rate(10);

  

 
  while (ros::ok())
  {    
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


    // ob1.left_lower_pos_x = 5;
    // ob1.left_lower_pos_y = 3;
    // ob1.ob_height = 4;
    // ob1.ob_width = 3;
    // ob1.num = 0;
    // ob1.if_at_bound = false;

    std::vector<message_interfaces::msg::Obstacle> obset;
    obset.push_back(ob1);
    obset.push_back(ob2);

    message_interfaces::msg::Mymap msg;
    msg.map_height = 10;
    msg.map_width = 10;
    msg.ob_num = 2;
    msg.ob_set = obset;

    pub.publish(msg);
    std::cout << "map published" << std::endl;
    //ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
