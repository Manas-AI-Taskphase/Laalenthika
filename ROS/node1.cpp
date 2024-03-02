#include <ros/ros.h>
#include <std_msgs/String.h>
#include <time.h>
#include <pwd.h>

void ChatterCallback(const std_msgs::String::ConstPtr& msg){
  ROS_INFO("[%s]", msg->data.c_str());
}
int main(int argc, char **argv){
  ros::init(argc, argv, "node1");
  ros::NodeHandle n;

  ros::Publisher publisher = n.advertise<std_msgs::String>("chatter",1000);
  ros::Subscriber subscriber = n.subscribe("chatter", 1000, ChatterCallback);
 // std::String user_message;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Rate loop_rate(10);

  while (ros::ok()){
    std_msgs::String msg;

    std::string user_message;
    std::cout << "Enter message: " << std::endl;
    std::cin >> user_message;
    msg.data = user_message;
    publisher.publish(msg);

   // subscriber.subscribe();
    ros::spinOnce();
    loop_rate.sleep();
  }

return 0;
}


