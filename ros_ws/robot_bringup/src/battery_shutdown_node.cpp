#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <robot_msgs/BatteryStatus.h>
#include <cstdlib>

ros::Publisher shutdown_cmd_pub;
bool shutdown_triggered = false;

void batteryCallback(const robot_msgs::BatteryStatus::ConstPtr &msg)
{
  if (shutdown_triggered)
    return;

  if (!msg->low_warning && msg->percent > 20.0f)
    return;

  shutdown_triggered = true;
  ROS_WARN("Battery at %.1f%% — shutting down Jetson.", msg->percent);

  std_msgs::Bool cmd;
  cmd.data = true;
  shutdown_cmd_pub.publish(cmd);

  ros::Duration(1.0).sleep();
  system("sudo shutdown -h now");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "battery_shutdown_node");
  ros::NodeHandle nh;

  shutdown_cmd_pub = nh.advertise<std_msgs::Bool>("shutdown_cmd", 1, true);
  ros::Subscriber sub = nh.subscribe("battery_status", 10, batteryCallback);

  ROS_INFO("Battery shutdown node ready.");
  ros::spin();
  return 0;
}
