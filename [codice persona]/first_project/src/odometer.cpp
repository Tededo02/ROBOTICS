#include "ros/ros.h"
#include <tf/transform_broadcaster.h>

class Odometer {
public:
Odometer() {
        // Subscribe to the topic and bind the callback method.
        sub = nSub.subscribe("speedsteer", 1000, &Odometer::callback, this);
    }

    void callback(const geometry_msgs::PointStamped::ConstPtr& msg) {

        pub = nPub.advertise<nav_msgs::Odometry>("odom", 1);
        nav_msgs::Odometry odomMsg;
        // odometry message
        pub.publish(odomMsg);

        // Update the transform's origin with the new pose
        transform.setOrigin(tf::Vector3(msg->x, msg->y, 0));
        // Update the quaternion based on the new theta value
        q.setRPY(0, 0, msg->theta);
        transform.setRotation(q);
        // Publish the updated transform
        brPub.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom_vehicle"));
    }

private:
    ros::NodeHandle nSub;
    tf::TransformBroadcaster brPub;
    ros::Subscriber sub;
    // Declare transform and quaternion as class members to reuse them in each callback
    tf::Transform transform; //da mettere nel terzo nodo??
    tf::Quaternion q;

    ros::NodeHandle nPub;
    ros::Publisher pub;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometer");

    Odometer odometer;
    ros::spin();

    return 0;
}

