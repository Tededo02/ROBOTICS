#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <cmath>
#include <deque>
#include <angles/angles.h>

class Odometer {
    public:
    // Costruttore della classe
    Odometer()
        : nh("~"), x(0.0), y(0.0), theta(85 * M_PI / 180.0), got_first_time(false)
    {
        // Inizializzazione dei parametri del veicolo
        wheelbase     = 176.5 / 100.0;     // metri
        rear_baseline = 130.0 / 100.0;     // metri

        // Sottoscrizione al topic dei comandi di velocità e sterzo
        sub = nh.subscribe("/speedsteer", 1000, &Odometer::callback, this);
        
        // Publisher per il topic dell’odometria
        pub = nh.advertise<nav_msgs::Odometry>("/odom", 1000);
    }

    void callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        double speed_mps = msg->point.y / 3.6;               // km/h → m/s
        double raw_sf    = msg->point.x;                     // steering angle of the steering wheel [grad]
        double omega;        // massimo angolo sterzo [rad]
        double steering_angle;
        // Normalizzo steering factor e ricavo steering angle
        if (raw_sf > 0){
             steering_angle =(raw_sf) / steering_factor;
        }else{ // normalizzo [grad]
        steering_angle =(raw_sf+5.5) / steering_factor; // steering angle [grad] era 5.5
        }
        steering_angle = steering_angle*M_PI / 180.0; // [rad]

        ros::Time current_time = msg->header.stamp;
        // primo messaggio: inoltro posizione iniziale e TF
        if (!got_first_time) {
            last_time      = current_time;
            x = 0.0; y = 0.0; theta = 85*M_PI/180.0;
            publishOdomTF(current_time, 0.0, theta);
            got_first_time = true;
            return;
        }
        double dt = (current_time - last_time).toSec();
        if (dt <= 0.0) return;

        // Cinematica bicycle model
        double dx=0.0, dy=0.0, dtheta=0.0;
        const double EPS_ANG = 0.00260; //era 0.00265
        if (fabs(steering_angle) < EPS_ANG) { //fabs(steering_angle) < EPS_ANG) {
            // moto rettilineo
            dtheta =   0.0;
            omega = 0.0;
            dx =  speed_mps*dt * cos(theta+(omega*dt)/2.0);
            dy = speed_mps*dt * sin(theta+(omega*dt)/2.0);

        } else {
            if(steering_angle > 0.01){
                steering_angle +=0.008;
                }else{
                    steering_angle-(0.00273);//era 0.00273
                } //correzione angolo sterzo [grad]
            double R = wheelbase / tan(steering_angle)+rear_baseline/2; // raggio di curvatura [m]
            dtheta = (speed_mps / R) * dt;
            omega = dtheta / dt;
            dx = R * (sin(theta + dtheta) - sin(theta));
            dy = -R * (cos(theta + dtheta) - cos(theta));
            theta += dtheta;
             
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
        odom.twist.twist.linear.y  = 0;//il veicolo non può muoversi lateralmente nel suo frame
        odom.twist.twist.angular.z = omega;
        pub.publish(odom);

        tf::Transform t;
        t.setOrigin(tf::Vector3(x, y, 0));
        t.setRotation(tf::createQuaternionFromYaw(theta));
        brPub.sendTransform(tf::StampedTransform(t, time, "odom", "vehicle"));
    }
    double steering_factor=30.0; // steering factor [grad]
    ros::NodeHandle nh;
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

