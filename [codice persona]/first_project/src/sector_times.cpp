#include "ros/ros.h"
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include "first_project/sector_times.h" // Messaggio custom
#include <cmath>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

class SectorTimes {
public:
    SectorTimes()
        : current_sector(1), current_sector_time(0.0),
          current_sector_speed_sum(0.0), speed_count(0),
          last_latitude(0.0), last_longitude(0.0)
          {
            // Subscribe to /speedsteer (vehicle data)
            speedsteer_sub.subscribe(nh, "/speedsteer", 1000);
            gps_sub.subscribe(nh, "/swiftnav/front/gps_pose", 1000);
    
            sync.reset(new message_filters::TimeSynchronizer<geometry_msgs::PointStamped, sensor_msgs::NavSatFix>(
                speedsteer_sub, gps_sub, 10));
            sync->registerCallback(boost::bind(&SectorTimes::callback, this, _1, _2));
    
            sector_pub = nh.advertise<first_project::sector_times>("/sector_times", 10);
        }
    

    void callback(const geometry_msgs::PointStamped::ConstPtr& speed_msg,
        const sensor_msgs::NavSatFix::ConstPtr& gps_msg) {
double speed = speed_msg->point.x / 3.6; // Convert speed from km/h to m/s
double lat = gps_msg->latitude;
double lon = gps_msg->longitude;
//aggiorno la velocità media del settore corrente
ros::Time now = ros::Time::now();
mean_speed = (mean_speed*current_sector_time + speed*(now - current_sector_start_time).toSec()) / (current_sector_time + (now - current_sector_start_time).toSec());

// Calcola distanza e tempo
double distance = haversineDistance(lat, lon, last_latitude, last_longitude);
current_sector_time = (now - current_sector_start_time).toSec();
//calcola il settore corrente

  // Crea e pubblica il messaggio custom
  first_project::sector_times sector_msg;
  sector_msg.current_sector = current_sector;
  sector_msg.current_sector_time = current_sector_time;
  sector_msg.current_sector_mean_speed = speed; // Usa la velocità corrente
  sector_pub.publish(sector_msg);

            // Reset per il prossimo settore
            last_latitude = lat;
            last_longitude = lon;
            current_sector_start_time = now;
}
    private:
    ros::NodeHandle nh;
    message_filters::Subscriber<geometry_msgs::PointStamped> speedsteer_sub;
    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub;
    boost::shared_ptr<message_filters::TimeSynchronizer<geometry_msgs::PointStamped, sensor_msgs::NavSatFix>> sync;
    ros::Publisher sector_pub;

    // Sector info
    int current_sector;
    double current_sector_time;
    double current_sector_speed_sum;
    int speed_count;

    ros::Time current_sector_start_time;
    double last_latitude= 45.618932386592405;
    double last_longitude= 9.281178887031235;
    double last_sector=1;
    double mean_speed=0.0;

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
