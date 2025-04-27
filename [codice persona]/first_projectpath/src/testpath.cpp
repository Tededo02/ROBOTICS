#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

class OdomToPath
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber gps_odom_sub_;
    ros::Publisher path_pub_;
    ros::Publisher gps_path_pub_;

    nav_msgs::Path path_;
    nav_msgs::Path gps_path_;
    unsigned int max_path_length_;

public:
    OdomToPath() : max_path_length_(10000)
    {
        // Inizializzazione
        odom_sub_ = nh_.subscribe("/odom", 10, &OdomToPath::odomCallback, this);
        gps_odom_sub_ = nh_.subscribe("/gps_odom", 10, &OdomToPath::gpsOdomCallback, this);

        path_pub_ = nh_.advertise<nav_msgs::Path>("/odom_path", 10);
        gps_path_pub_ = nh_.advertise<nav_msgs::Path>("/gps_odom_path", 10);

        path_.header.frame_id = "odom";
        gps_path_.header.frame_id = "odom";
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;

        path_.poses.push_back(pose);
        if (path_.poses.size() > max_path_length_)
            path_.poses.erase(path_.poses.begin());

        path_.header.stamp = ros::Time::now();
        path_pub_.publish(path_);
    }

    void gpsOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;

        gps_path_.poses.push_back(pose);
        if (gps_path_.poses.size() > max_path_length_)
            gps_path_.poses.erase(gps_path_.poses.begin());

        gps_path_.header.stamp = ros::Time::now();
        gps_path_pub_.publish(gps_path_);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_to_path");
    OdomToPath otp;
    ros::spin();
    return 0;
}
