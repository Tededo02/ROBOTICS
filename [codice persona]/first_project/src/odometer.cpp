#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>

class Odometer {
public:
    Odometer() : x(0.0), y(0.0), theta(0.0), last_time(ros::Time::now()) {
        // Subscribe to /speedsteer topic
        sub = nSub.subscribe("/speedsteer", 1000, &Odometer::callback, this);
        
        // Publisher for odometry messages
        pub = nPub.advertise<nav_msgs::Odometry>("/odom", 1);
    }

    void callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        // Extract speed (km/h) and steering angle (degrees)
        double speed_mps = msg->point.x / 3.6; // Convert speed to m/s
        double steering_angle_rad = msg->point.y * M_PI / 180.0; // Convert steering angle to radians

        // Compute time delta
        ros::Time current_time = ros::Time::now();
        double delta_time = (current_time - last_time).toSec();
        last_time = current_time;

        // Compute odometry using vehicle kinematics
        double delta_x = speed_mps * cos(theta) * delta_time;
        double delta_y = speed_mps * sin(theta) * delta_time;
        double delta_theta = (speed_mps / wheelbase) * tan(steering_angle_rad) * delta_time;

        x += delta_x;
        y += delta_y;
        theta += delta_theta;

        // Publish odometry message
        nav_msgs::Odometry odomMsg;
        odomMsg.header.stamp = current_time;
        odomMsg.header.frame_id = "odom";

        // Set position
        odomMsg.pose.pose.position.x = x;
        odomMsg.pose.pose.position.y = y;
        odomMsg.pose.pose.position.z = 0.0;
        odomMsg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

        // Set velocity
        odomMsg.twist.twist.linear.x = speed_mps;
        odomMsg.twist.twist.angular.z = delta_theta / delta_time;

        pub.publish(odomMsg);

        // Broadcast the updated transform
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x, y, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, theta);
        transform.setRotation(q);

        brPub.sendTransform(tf::StampedTransform(transform, current_time, "odom", "vehicle"));
    }

private:
    ros::NodeHandle nSub, nPub;
    ros::Subscriber sub;
    ros::Publisher pub;
    tf::TransformBroadcaster brPub;

    // Vehicle parameters
    const double wheelbase = 1.765; // Distance from front to rear wheels in meters
    double x, y, theta; // Odometry state
    ros::Time last_time; // Last callback time
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometer");

    Odometer odometer;
    ros::spin();

    return 0;
}
