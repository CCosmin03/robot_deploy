#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <string>

ros::Publisher diagnostics_pub;

diagnostic_msgs::DiagnosticStatus imu_state;
diagnostic_msgs::DiagnosticStatus LDS_state;

void setDiagnosisMsg(diagnostic_msgs::DiagnosticStatus *diag, uint8_t level, std::string name, std::string message, std::string hardware_id)
{
  diag->level = level;
  diag->name  = name;
  diag->message = message;
  diag->hardware_id = hardware_id;
}

void setIMUDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&imu_state, level, "IMU Sensor", message, "BNO055");
}

void setLDSDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&LDS_state, level, "Lidar Sensor", message, "RPLIDAR");
}

void imuMsgCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  setIMUDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
}

void LDSMsgCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  setLDSDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
}

void msgPub()
{
  diagnostic_msgs::DiagnosticArray diagnostics;

  diagnostics.header.stamp = ros::Time::now();

  diagnostics.status.clear();
  diagnostics.status.push_back(imu_state);
  diagnostics.status.push_back(LDS_state);

  diagnostics_pub.publish(diagnostics);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_diagnostic");
  ros::NodeHandle nh;

  diagnostics_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);

  ros::Subscriber imu = nh.subscribe("imu", 10, imuMsgCallback);
  ros::Subscriber lds = nh.subscribe("scan", 10, LDSMsgCallback);

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    msgPub();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
