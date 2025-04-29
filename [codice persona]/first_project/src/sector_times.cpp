#include "ros/ros.h"
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include "first_project/sector_times.h" // Messaggio custom
#include <cmath>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

class SectorTimes {
public:
    SectorTimes()
        : current_sector(1), current_sector_time(0.0), current_sector_mean_speed(0.0)
    {
        speedsteer_sub.subscribe(nh, "/speedsteer", 10);
        gps_sub.subscribe(nh, "/swiftnav/front/gps_pose", 10);

        sync.reset(new Sync(MySyncPolicy(10), speedsteer_sub, gps_sub));
        sync->registerCallback(boost::bind(&SectorTimes::callback, this, _1, _2));

        sector_pub = nh.advertise<first_project::sector_times>("/sector_times", 10);
    }

    void callback(const geometry_msgs::PointStamped::ConstPtr& speed_msg,
                  const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
    {
        double speed = speed_msg->point.y / 3.6; // km/h -> m/s
        double lat = gps_msg->latitude;
        double lon = gps_msg->longitude;
        ros::Time now;
        if(got_first==false){
            got_first=true;
            last_time = ros::Time::now();
            now=last_time;
        }else{
            now = ros::Time::now();
        }
        double dt = (now - last_time).toSec();
        last_time = now;

        if (dt <= 0.0) return; // Protezione contro deltaTime negativi o nulli

        // Gestione transizioni tra settori (con tolleranza sulla latitudine)
        if (std::abs(lat - latSet2) < 0.0001 && lon >= lonSet2 && current_sector == 1) {
            current_sector = 2;
            // ROS_INFO("tempo settore 1 : %f", current_sector_time);
            // ROS_INFO("media velocità settore 1 : %f m/s", current_sector_mean_speed);
            current_sector_time = 0.0;
            current_sector_mean_speed = speed; // km/h -> m/s
            
        }
        else if (std::abs(lat - latSet3) < 0.0001 && lon <= lonSet3 && current_sector == 2) {
            current_sector = 3;
            // ROS_INFO("tempo settore 2 : %f", current_sector_time);
            // ROS_INFO("media velocità settore 2 : %f m/s", current_sector_mean_speed);
            current_sector_time = 0.0;
            current_sector_mean_speed =speed; // km/h -> m/s

        }
        else if (std::abs(lat - latSet1) < 0.0001 && std::abs(lon - lonSet1)<0.0001 && current_sector == 3) {
            current_sector = 1;
            // ROS_INFO("tempo settore 3 : %f", current_sector_time);
            // ROS_INFO("media velocità settore 3 : %f m/s", current_sector_mean_speed);
            current_sector_time = 0.0;
            current_sector_mean_speed = speed; // km/h -> m/s
        }

        // Aggiorna media e tempo
        current_sector_mean_speed = (current_sector_mean_speed * current_sector_time + speed * dt) / (current_sector_time + dt);
        current_sector_time += dt;

        // Crea e pubblica il messaggio
        first_project::sector_times msg;
        msg.current_sector = current_sector;
        msg.current_sector_time = current_sector_time;
        msg.current_sector_mean_speed = current_sector_mean_speed*3.6;//riconverto in km/h
        sector_pub.publish(msg);
    }

private:
    ros::NodeHandle nh;
    message_filters::Subscriber<geometry_msgs::PointStamped> speedsteer_sub;
    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub;

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped, sensor_msgs::NavSatFix> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    ros::Publisher sector_pub;

    // Coordinate settori
    double latSet1 = 45.615833;
    double lonSet1 = 9.280556;
    double latSet2 = 45.630087;
    double lonSet2 = 9.289486;
    double latSet3 = 45.623333;
    double lonSet3 = 9.287222;

    // Stato attuale
    bool got_first=false;
    int current_sector;
    double current_sector_time;
    double current_sector_mean_speed;
    ros::Time last_time;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sector_times");
    SectorTimes node;
    ros::spin();
    return 0;
}
