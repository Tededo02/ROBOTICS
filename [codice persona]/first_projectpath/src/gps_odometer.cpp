#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

class GpsOdometer {
public:
    // Costruttore della classe
    GpsOdometer()
        : nh("~"), gotPrev(false), filteredYaw(0.0)
    {
        // Sottoscrizione al topic GPS
        gpsSub = nh.subscribe("/swiftnav/front/gps_pose", 1000,
                              &GpsOdometer::gpsCallback, this);
        // Publisher per il topic di odometria
        odomPub = nh.advertise<nav_msgs::Odometry>("/gps_odom", 1000);

        // Lettura dei parametri dal file di launch
        nh.param("gps_odometer/lat_ref", latRef,
                 45.618932386592405); // Default: coordinate Milano
        nh.param("gps_odometer/lon_ref", lonRef,
                 9.281178887031235);
        nh.param("gps_odometer/alt_ref", altRef,
                 229.04906147731415);

        // Conversione da gradi a radianti
        latRef = latRef * M_PI / 180.0;
        lonRef = lonRef * M_PI / 180.0;
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber gpsSub;
    ros::Publisher odomPub;
    tf::TransformBroadcaster brPub;

    double latRef, lonRef, altRef;
    bool gotPrev;
    double prevX, prevY;
    double filteredYaw;
    ros::Time lastTime;

    const double MIN_DIST = 0.05; // Soglia minima [m]
    const double ALPHA    = 0.1;  // Fattore di smoothing

    // Costanti WGS84
    const double a  = 6378137.0;
    const double e2 = 6.69437999014e-3;

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        double vx = 0.0, vy = 0.0, omega = 0.0;
        // Converto lat/lon/alt a ECEF
        double lat = msg->latitude  * M_PI / 180.0;
        double lon = msg->longitude * M_PI / 180.0;
        double alt = msg->altitude;

        double N  = a / sqrt(1 - e2 * pow(sin(lat), 2));
        double x  = (N + alt) * cos(lat) * cos(lon);
        double y  = (N + alt) * cos(lat) * sin(lon);
        double z  = (N * (1 - e2) + alt) * sin(lat);

        // Punto di riferimento in ECEF
        double N0 = a / sqrt(1 - e2 * pow(sin(latRef), 2));
        double x0 = (N0 + altRef) * cos(latRef) * cos(lonRef);
        double y0 = (N0 + altRef) * cos(latRef) * sin(lonRef);
        double z0 = (N0 * (1 - e2) + altRef) * sin(latRef);

        // Differenza
        double dx = x - x0;
        double dy = y - y0;
        double dz = z - z0;

        // Conversione ECEF → ENU
        double enuX = -sin(lonRef) * dx + cos(lonRef) * dy;
        double enuY = -sin(latRef) * cos(lonRef) * dx
                    - sin(latRef) * sin(lonRef) * dy
                    + cos(latRef) * dz;
        double enuZ =  cos(latRef) * cos(lonRef) * dx
                    + cos(latRef) * sin(lonRef) * dy
                    + sin(latRef) * dz;

        // Calcolo dello yaw filtrato
        double yaw = filteredYaw;

        ros::Time now = ros::Time::now();
        ros::Duration dt = now - lastTime;
        if(lat==0 && lon==0){
            return;
        }
        if (gotPrev) {
            double dX   = enuX - prevX;
            double dY   = enuY - prevY;
            double dist = sqrt(dX*dX + dY*dY);
            if (dist > MIN_DIST) {
                double rawYaw = atan2(dY, dX);
                yaw = filteredYaw;
                filteredYaw   = (1 - ALPHA) * filteredYaw + ALPHA * rawYaw;

                //potrebbe non servire
                vx = dX / dt.toSec();
                vy = dY / dt.toSec();
                omega = (yaw - filteredYaw) / dt.toSec();
            }
        }else{
            lastTime = now;
        }

        prevX = enuX;
        prevY = enuY;
        gotPrev = true;

        // Pubblico Odometry
        nav_msgs::Odometry odom;
        odom.header.stamp    = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id  = "gps";

        odom.pose.pose.position.x    = enuX;
        odom.pose.pose.position.y    = enuY;
        odom.pose.pose.position.z    = enuZ;
        geometry_msgs::Quaternion q;
        q = tf::createQuaternionMsgFromYaw(yaw);
        odom.pose.pose.orientation    = q;

        // Aggiungo twist stimato
        odom.twist.twist.linear.x  = vx;
        odom.twist.twist.linear.y  = vy;
        odom.twist.twist.angular.z = omega;

        odomPub.publish(odom);

        // Pubblico TF 
        tf::Transform tfTrans;
        tfTrans.setOrigin(tf::Vector3(enuX, enuY, enuZ));
        tf::Quaternion tfQ;
        tfQ.setRPY(0, 0, yaw);
        tfTrans.setRotation(tfQ);
        brPub.sendTransform(tf::StampedTransform(tfTrans, now, "odom", "gps"));
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_odometer");
    GpsOdometer node;
    ros::spin();
    return 0;
}
