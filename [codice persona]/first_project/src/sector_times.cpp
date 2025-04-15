#include "ros/ros.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "first_project/sector_times.h" // Messaggio custom

class SectorTimes {
public:
    SectorTimes() : current_sector(1), current_sector_time(0.0), current_sector_speed_sum(0.0), speed_count(0) {
        // Subscribe to /speedsteer (vehicle data)
        speedsteer_sub = nh.subscribe("/speedsteer", 1000, &SectorTimes::speedsteerCallback, this);

        // Subscribe to /swiftnav/front/gps_pose (GPS data)
        gps_sub = nh.subscribe("/swiftnav/front/gps_pose", 1000, &SectorTimes::gpsCallback, this);

        // Publisher for sector times
        sector_pub = nh.advertise<first_project::sector_times>("/sector_times", 10);
    }

    void speedsteerCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        double speed = msg->point.x / 3.6; // Convert speed from km/h to m/s
        current_sector_speed_sum += speed;
        speed_count++;
    }

    void gpsCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        ros::Time now = ros::Time::now();
        if (current_sector_start_time.isZero()) {
            // Initialize sector time
            current_sector_start_time = now;
            last_position = msg->pose.position;
            return;
        }

        // Calculate distance to determine sector transitions
        double dx = msg->pose.position.x - last_position.x;
        double dy = msg->pose.position.y - last_position.y;
        double distance = sqrt(dx * dx + dy * dy);

        if (distance >= sector_length) {
            // End current sector
            current_sector_time = (now - current_sector_start_time).toSec();
            double mean_speed = current_sector_speed_sum / speed_count;

            // Publish sector times
            first_project::sector_times sector_msg;
            sector_msg.current_sector = current_sector;
            sector_msg.current_sector_time = current_sector_time;
            sector_msg.current_sector_mean_speed = mean_speed;
            sector_pub.publish(sector_msg);

            // Reset for the next sector
            current_sector++;
            current_sector_start_time = now;
            last_position = msg->pose.position;
            current_sector_speed_sum = 0.0;
            speed_count = 0;
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber speedsteer_sub, gps_sub;
    ros::Publisher sector_pub;

    // Sector information
    int current_sector;
    double current_sector_time;
    double current_sector_speed_sum;
    int speed_count;

    ros::Time current_sector_start_time;
    geometry_msgs::Point last_position;
    const double sector_length = 100.0; // Length of each sector in meters (example value)
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "sector_times");

    SectorTimes sectorTimes;
    ros::spin();

    return 0;
}
