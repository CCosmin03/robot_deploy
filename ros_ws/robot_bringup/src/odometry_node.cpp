#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <robot_msgs/EncoderData.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath>

class OdometryNode
{
public:
  OdometryNode()
    : x_(0.0), y_(0.0), theta_(0.0),
      prev_left_ticks_(0), prev_right_ticks_(0),
      initialized_(false)
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param("wheel_radius",     wheel_radius_,     0.03208);
    pnh.param("wheel_separation", wheel_separation_, 0.2025);
    pnh.param("ticks_per_rev",    ticks_per_rev_,    11800.0);
    pnh.param("odom_frame",       odom_frame_,       std::string("odom"));
    pnh.param("base_frame",       base_frame_,       std::string("base_footprint"));

    meters_per_tick_ = (2.0 * M_PI * wheel_radius_) / ticks_per_rev_;

    odom_pub_    = nh.advertise<nav_msgs::Odometry>("odom", 50);
    encoder_sub_ = nh.subscribe("encoder_data", 50, &OdometryNode::encoderCb, this);

    ROS_INFO("Odometry node started — radius=%.4fm, separation=%.3fm, ticks/rev=%.0f",
             wheel_radius_, wheel_separation_, ticks_per_rev_);
  }

private:
  void encoderCb(const robot_msgs::EncoderData::ConstPtr& msg)
  {
    if (!initialized_)
    {
      prev_left_ticks_  = msg->left_ticks;
      prev_right_ticks_ = msg->right_ticks;
      last_time_        = ros::Time::now();
      initialized_      = true;
      return;
    }

    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time_).toSec();

    if (dt <= 0.0)
      return;

    long delta_left_ticks  = msg->left_ticks  - prev_left_ticks_;
    long delta_right_ticks = msg->right_ticks - prev_right_ticks_;

    // Detectie reset Arduino: daca delta e nerealist de mare, ignoram mesajul
    if (std::abs(delta_left_ticks)  > ticks_per_rev_ * 5 ||
        std::abs(delta_right_ticks) > ticks_per_rev_ * 5)
    {
      prev_left_ticks_  = msg->left_ticks;
      prev_right_ticks_ = msg->right_ticks;
      last_time_        = current_time;
      ROS_WARN("Encoder jump detected — possible Arduino reset. Skipping.");
      return;
    }

    prev_left_ticks_  = msg->left_ticks;
    prev_right_ticks_ = msg->right_ticks;
    last_time_        = current_time;

    double delta_left  = delta_left_ticks  * meters_per_tick_;
    double delta_right = delta_right_ticks * meters_per_tick_;

    // Midpoint integration for differential drive
    double delta_s     = (delta_right + delta_left) / 2.0;
    double delta_theta = (delta_right - delta_left) / wheel_separation_;

    x_     += delta_s * std::cos(theta_ + delta_theta / 2.0);
    y_     += delta_s * std::sin(theta_ + delta_theta / 2.0);
    theta_ += delta_theta;
    theta_  = std::atan2(std::sin(theta_), std::cos(theta_));

    double vx  = delta_s     / dt;
    double vth = delta_theta / dt;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

    // Broadcast TF: odom -> base_footprint
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp            = current_time;
    tf_msg.header.frame_id         = odom_frame_;
    tf_msg.child_frame_id          = base_frame_;
    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation      = odom_quat;
    tf_broadcaster_.sendTransform(tf_msg);

    // Publish nav_msgs/Odometry
    nav_msgs::Odometry odom;
    odom.header.stamp    = current_time;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id  = base_frame_;

    odom.pose.pose.position.x  = x_;
    odom.pose.pose.position.y  = y_;
    odom.pose.pose.position.z  = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // Diagonal pose covariance — z/roll/pitch not observable on flat ground
    odom.pose.covariance[0]  = 1e-3;
    odom.pose.covariance[7]  = 1e-3;
    odom.pose.covariance[14] = 1e6;
    odom.pose.covariance[21] = 1e6;
    odom.pose.covariance[28] = 1e6;
    odom.pose.covariance[35] = 1e-2;

    odom.twist.twist.linear.x  = vx;
    odom.twist.twist.angular.z = vth;

    odom.twist.covariance[0]  = 1e-3;
    odom.twist.covariance[7]  = 1e-3;
    odom.twist.covariance[14] = 1e6;
    odom.twist.covariance[21] = 1e6;
    odom.twist.covariance[28] = 1e6;
    odom.twist.covariance[35] = 1e-2;

    odom_pub_.publish(odom);
  }

  ros::Subscriber          encoder_sub_;
  ros::Publisher           odom_pub_;
  tf::TransformBroadcaster tf_broadcaster_;

  double wheel_radius_;
  double wheel_separation_;
  double ticks_per_rev_;
  double meters_per_tick_;

  std::string odom_frame_;
  std::string base_frame_;

  double    x_, y_, theta_;
  long      prev_left_ticks_, prev_right_ticks_;
  ros::Time last_time_;
  bool      initialized_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_node");
  OdometryNode node;
  ros::spin();
  return 0;
}
