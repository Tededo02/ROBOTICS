#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

class GpsOdometer {
public:
    // Questo è il costruttore della classe
    GpsOdometer() : nh("~"), gotPrev(false) { //prev potrebbe essere non necessario
        // Sottoscrizione al topic GPS
        gpsSub = nh.subscribe("/swiftnav/front/gps_pose", 1, &GpsOdometer::gpsCallback, this);
    
        // Publisher per il topic di odometria
        odomPub = nh.advertise<nav_msgs::Odometry>("/gps_odom", 1);
    
        // Lettura dei parametri dal file di launch che indicano la posizione iniziale dell'auto dal GPS
        // Posizione che utilizzo anche come riferimento per il calcolo della posizione
        nh.param("gps_odometer/lat_ref", latRef, 10.0); // Default: 10.0 gradi
        nh.param("gps_odometer/lon_ref", lonRef, 10.0); // Default: 10.0 gradi
        nh.param("gps_odometer/alt_ref", altRef, 10.0); // Default: 10.0 metri
    
        // Conversione dei parametri di latitudine e longitudine da gradi a radianti
        latRef = latRef * M_PI / 180.0;
        lonRef = lonRef * M_PI / 180.0;
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber gpsSub;
    ros::Publisher odomPub;
    tf2_ros::TransformBroadcaster tfBroadcaster;

    double latRef, lonRef, altRef;
    bool gotPrev;
    double prevX, prevY;

    // WGS84 ellipsoid constants
    const double a = 6378137.0;        // semi-major axis
    const double e2 = 6.69437999014e-3; // eccentricity squared

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        //  Converto inizialmente lat/lon/alt a ECEF
        double lat = msg->latitude * M_PI / 180.0;
        double lon = msg->longitude * M_PI / 180.0;
        double alt = msg->altitude;

        double N = a / sqrt(1 - e2 * pow(sin(lat), 2));
        double x = (N + alt) * cos(lat) * cos(lon);
        double y = (N + alt) * cos(lat) * sin(lon);
        double z = (N * (1 - e2) + alt) * sin(lat);

        //  (Converto anche il punto di riferimento anche in ECEF se necessario)
        double N0 = a / sqrt(1 - e2 * pow(sin(latRef), 2));
        double x0 = (N0 + altRef) * cos(latRef) * cos(lonRef);
        double y0 = (N0 + altRef) * cos(latRef) * sin(lonRef);
        double z0 = (N0 * (1 - e2) + altRef) * sin(latRef);

        // Calcolo differenza
        double dx = x - x0;
        double dy = y - y0;
        double dz = z - z0;

        // Adesso passo da ECEF a ENU
        double enuX = -sin(lonRef) * dx + cos(lonRef) * dy;
        double enuY = -sin(latRef) * cos(lonRef) * dx
                    - sin(latRef) * sin(lonRef) * dy
                    + cos(latRef) * dz;
        double enuZ =  cos(latRef) * cos(lonRef) * dx
                    + cos(latRef) * sin(lonRef) * dy
                    + sin(latRef) * dz;

        // Per ultimo calcolo orientamento, posso farlo solo se ho il punto precedente
        // Indica angolo tra asse x e asse y
        double yaw = 0.0;
        if (gotPrev) {
            double dX = enuX - prevX;
            double dY = enuY - prevY;
            yaw = atan2(dY, dX);
        }

        prevX = enuX;
        prevY = enuY;
        gotPrev = true;

        // Pubblico Odometry
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom"; // Questo è il frame di riferimento del GPS
        odom.child_frame_id = "gps"; // Questo è il frame di riferimento dell'auto, sto pubblicando la posizione dell'auto rispetto al GPS
        // Posizione dell'auto rispetto al GPS sistema ENU
        odom.pose.pose.position.x = enuX;
        odom.pose.pose.position.y = enuY;
        odom.pose.pose.position.z = enuZ;

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        //Pubblico sul topi /gps_odom
        odomPub.publish(odom);

        // Pubblica TF, serve per visualizzare la posizione dell'auto rispetto al GPS, e salvarlo per poi visualizzarlo con rviz
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = odom.header.stamp;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "gps";
        transform.transform.translation.x = enuX;
        transform.transform.translation.y = enuY;
        transform.transform.translation.z = enuZ;
        transform.transform.rotation = odom.pose.pose.orientation;

        tfBroadcaster.sendTransform(transform);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_odometer");
    GpsOdometer node;
    ros::spin();
    return 0;
}
