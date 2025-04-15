#include "ros/ros.h"
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include "first_project/sector_times.h" // Messaggio custom
#include <cmath>

class SectorTimes {
public:
    SectorTimes()
        : current_sector(1), current_sector_time(0.0),
          current_sector_speed_sum(0.0), speed_count(0),
          last_latitude(0.0), last_longitude(0.0), has_initial_position(false)
    {
        // Subscribe to /speedsteer (vehicle data)
        speedsteer_sub = nh.subscribe("/speedsteer", 1000, &SectorTimes::speedsteerCallback, this);

        // Subscribe to GPS topic with NavSatFix
        gps_sub = nh.subscribe("/swiftnav/front/gps_pose", 1000, &SectorTimes::gpsCallback, this);

        // Publisher for sector times
        sector_pub = nh.advertise<first_project::sector_times>("/sector_times", 10);
    }

    void speedsteerCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        double speed = msg->point.x / 3.6; // Convert speed from km/h to m/s
        current_sector_speed_sum += speed;
        speed_count++;
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        ros::Time now = ros::Time::now();

        double lat = msg->latitude;
        double lon = msg->longitude;

        if (!has_initial_position) {
            last_latitude = lat;
            last_longitude = lon;
            current_sector_start_time = now;
            has_initial_position = true;
            return;
        }

        // Calcola distanza approssimativa in metri (Haversine semplificato per piccole distanze)
        double distance = haversineDistance(lat, lon, last_latitude, last_longitude);

        if (distance >= sector_length) {
            current_sector_time = (now - current_sector_start_time).toSec();
            double mean_speed = current_sector_speed_sum / speed_count;

            // Publish sector times
            first_project::sector_times sector_msg;
            sector_msg.current_sector = current_sector;
            sector_msg.current_sector_time = current_sector_time;
            sector_msg.current_sector_mean_speed = mean_speed;
            sector_pub.publish(sector_msg);

            // Reset for next sector
            current_sector++;
            current_sector_start_time = now;
            last_latitude = lat;
            last_longitude = lon;
            current_sector_speed_sum = 0.0;
            speed_count = 0;
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber speedsteer_sub, gps_sub;
    ros::Publisher sector_pub;

    // Sector info
    int current_sector;
    double current_sector_time;
    double current_sector_speed_sum;
    int speed_count;

    ros::Time current_sector_start_time;
    double last_latitude;
    double last_longitude;
    bool has_initial_position;

    const double sector_length = 100.0; // meters

    // Haversine distance in meters
    double haversineDistance(double lat1, double lon1, double lat2, double lon2) {
        const double R = 6371000; // Earth radius in meters
        double dLat = (lat2 - lat1) * M_PI / 180.0;
        double dLon = (lon2 - lon1) * M_PI / 180.0;

        lat1 = lat1 * M_PI / 180.0;
        lat2 = lat2 * M_PI / 180.0;

        double a = sin(dLat / 2) * sin(dLat / 2) +
                   sin(dLon / 2) * sin(dLon / 2) * cos(lat1) * cos(lat2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));
        return R * c;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "sector_times");

    SectorTimes sectorTimes;
    ros::spin();

    return 0;
}
