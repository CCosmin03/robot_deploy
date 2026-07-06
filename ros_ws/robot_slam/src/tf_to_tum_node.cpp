#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/TransformStamped.h>
#include <fstream>
#include <string>
#include <ctime>
#include <iomanip>

// ── TfToTumNode ──────────────────────────────────────────────────────────────
// Extrage traiectoria map->base_footprint din /tf si o scrie in format TUM.
// Format TUM: timestamp tx ty tz qx qy qz qw
// ─────────────────────────────────────────────────────────────────────────────

class TfToTumNode
{
public:
    TfToTumNode()
        : tf_buffer_(ros::Duration(120.0))
        , tf_listener_(tf_buffer_)
        , first_map_received_(false)
        , slam_start_wall_time_(ros::WallTime::now().toSec())
        , pose_count_(0)
    {
        ros::NodeHandle pnh("~");

        pnh.param<std::string>("output_file",    output_file_,    "trajectory.tum");
        pnh.param<std::string>("startup_file",   startup_file_,   "startup_time.txt");
        pnh.param<double>     ("rate_hz",        rate_hz_,        10.0);

        tum_file_.open(output_file_);
        if (!tum_file_.is_open())
            ROS_FATAL("Nu pot deschide fisierul TUM: %s", output_file_.c_str());

        map_sub_ = nh_.subscribe("/map", 1, &TfToTumNode::mapCallback, this);

        ROS_INFO("tf_to_tum_node pornit");
        ROS_INFO("  Output TUM : %s", output_file_.c_str());
        ROS_INFO("  Startup    : %s", startup_file_.c_str());
        ROS_INFO("  Rate       : %.1f Hz", rate_hz_);
    }

    void spin()
    {
        ros::Rate rate(rate_hz_);
        while (ros::ok())
        {
            ros::spinOnce();
            tryLookupTf();
            rate.sleep();
        }
    }

    ~TfToTumNode()
    {
        if (tum_file_.is_open())
            tum_file_.close();
        ROS_INFO("tf_to_tum_node: salvat %d pose in %s", pose_count_, output_file_.c_str());
    }

private:
    ros::NodeHandle nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Subscriber map_sub_;

    std::ofstream tum_file_;
    std::string output_file_;
    std::string startup_file_;
    double rate_hz_;
    bool first_map_received_;
    double slam_start_wall_time_;
    int pose_count_;

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& /*msg*/)
    {
        if (first_map_received_)
            return;
        first_map_received_ = true;
        double startup_s = ros::WallTime::now().toSec() - slam_start_wall_time_;
        std::ofstream sf(startup_file_);
        if (sf.is_open())
        {
            sf << std::fixed << std::setprecision(2) << startup_s << "\n";
            sf.close();
        }
        ROS_INFO("Prima harta primita — startup time: %.2f s", startup_s);
    }

    void tryLookupTf()
    {
        if (!tum_file_.is_open())
            return;
        try
        {
            geometry_msgs::TransformStamped ts =
                tf_buffer_.lookupTransform("map", "base_footprint",
                                           ros::Time(0), ros::Duration(0.05));

            double t   = ts.header.stamp.toSec();
            double tx  = ts.transform.translation.x;
            double ty  = ts.transform.translation.y;
            double tz  = ts.transform.translation.z;
            double qx  = ts.transform.rotation.x;
            double qy  = ts.transform.rotation.y;
            double qz  = ts.transform.rotation.z;
            double qw  = ts.transform.rotation.w;

            tum_file_ << std::fixed << std::setprecision(6)
                      << t  << " "
                      << tx << " " << ty << " " << tz << " "
                      << qx << " " << qy << " " << qz << " " << qw << "\n";
            ++pose_count_;
        }
        catch (const tf2::TransformException& e)
        {
            // normal la start; ignora
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_to_tum_node");
    TfToTumNode node;
    node.spin();
    return 0;
}
