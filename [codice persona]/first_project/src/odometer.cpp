#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <cmath>
#include <deque>
#include <angles/angles.h>

class Odometer {
public:
    Odometer() : x(0.0), y(0.0), theta(0.0), got_first_time(false) {
        wheelbase     = 176.5 / 100.0;     // → metri
        rear_baseline = 130.0 / 100.0;     // → metri (non usato per ora)

        sub = nSub.subscribe("/speedsteer", 1000, &Odometer::callback, this);
        pub = nPub.advertise<nav_msgs::Odometry>("/odom", 1000);
        ros::Time init_time = ros::Time::now();
        publishOdomTF(init_time, 0.0, 0.0);
    }

    void callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        double speed_mps = msg->point.x / 3.6;               // km/h → m/s
        double raw_sf    = msg->point.y;                     // steering angle of the steering wheel [grad]
        double omega;        // massimo angolo sterzo [rad]
        // Normalizzo steering factor e ricavo steering angle
        double steering_angle =raw_sf / steering_factor; // steering angle [grad]
        steering_angle = steering_angle*M_PI / 180.0; // [rad]

        ros::Time current_time = msg->header.stamp;
        // primo messaggio: inoltro posizione iniziale e TF
        if (!got_first_time) {
            last_time      = current_time;
            x = 0.0; y = 0.0; theta = 0.0;
            publishOdomTF(current_time, 0.0, 0.0);
            got_first_time = true;
            return;
        }
        double dt = (current_time - last_time).toSec();
        if (dt <= 0.0) return;

        // Cinematica bicycle model
        double dx=0.0, dy=0.0, dtheta=0.0;
        const double EPS_ANG = 1e-3;
        if (fabs(steering_angle) < EPS_ANG) {
            // moto rettilineo
            dx = speed_mps * cos(theta) * dt;
            dy = speed_mps * sin(theta) * dt;
            omega = 0.0;
        } else {
            double R = wheelbase / tan(steering_angle)+rear_baseline/2.0;
            dtheta = (speed_mps / R) * dt;
            dx = R * (sin(theta + dtheta) - sin(theta));
            dy = -R * (cos(theta + dtheta) - cos(theta));
            theta += dtheta;
             omega = dtheta / dt;
        }
        x += dx;
        y += dy;

        // normalizzo theta
        theta = angles::normalize_angle(theta);

        // pubblica odom e tf
        publishOdomTF(current_time, speed_mps, omega);
        last_time = current_time;
    }

private:
    void publishOdomTF(const ros::Time& time, double speed_mps, double omega) {
        nav_msgs::Odometry odom;
        odom.header.stamp    = time;
        odom.header.frame_id = "odom";
        odom.child_frame_id  = "vehicle";
        odom.pose.pose.position.x  = x;
        odom.pose.pose.position.y  = y;
        odom.pose.pose.position.z  = 0;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
        odom.twist.twist.linear.x  = speed_mps;
        odom.twist.twist.linear.y  = 0;
        odom.twist.twist.angular.z = omega;
        pub.publish(odom);

        tf::Transform t;
        t.setOrigin(tf::Vector3(x, y, 0));
        t.setRotation(tf::createQuaternionFromYaw(theta));
        brPub.sendTransform(tf::StampedTransform(t, time, "odom", "vehicle"));
    }
    double steering_factor=32.0; // steering factor [grad]
    ros::NodeHandle nSub, nPub;
    ros::Subscriber sub;
    ros::Publisher pub;
    tf::TransformBroadcaster brPub;
    double wheelbase, rear_baseline;
    double x, y, theta;
    ros::Time last_time;
    bool got_first_time;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometer");
    Odometer odometer;
    ros::spin();
    return 0;
}

