#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "first_project/custom.h"
#include "geometry_msgs/PointStamped.h"
#include <cmath>
#include <deque>

class SectorTimer {
public:
    SectorTimer() : private_n("~"), current_sector(1), count(0), v_average(0.0) {
        gps_sub = n.subscribe("/swiftnav/front/gps_pose", 1, &SectorTimer::gpsCallback, this);
        speed_sub = n.subscribe("/speedsteer", 1, &SectorTimer::speedCallback, this);
        pub = n.advertise<first_project::custom>("/sector_times", 10);

        private_n.param("sector1_lat", sec1_lat , 45.630106);
        private_n.param("sector1_longit", sec1_longit, 9.289490);

        private_n.param("sector2_lat", sec2_lat , 45.623570);
        private_n.param("sector2_longit", sec2_longit, 9.287297);

        private_n.param("sector3_lat", sec3_lat , 45.616042);
        private_n.param("sector3_longit", sec3_longit, 9.280767);

        //last_time = ros::Time::now();

        max_distance_threshold_ = 1;
        window_size_ = 5;
    }

private:
    ros::NodeHandle n;
    ros::NodeHandle private_n;
    ros::Subscriber gps_sub;
    ros::Subscriber speed_sub;
    ros::Publisher pub;


    bool received_first_msg = false;

    double sec1_lat, sec1_longit;
    double sec2_lat, sec2_longit;
    double sec3_lat, sec3_longit;
    ros::Time last_time;
    double v_average;
    int count;
    int current_sector=1;
    double current_speed;

    const double tol = 0.0005; // tolleranza per confronto coordinate

    double max_distance_threshold_;
    int window_size_;
    std::deque<std::pair<double, double>> valid_points_; // lat, lon

    void addValidPoint(double lat, double lon)
    {
        if (valid_points_.size() >= window_size_) {
            valid_points_.pop_front();
        }
        valid_points_.emplace_back(lat, lon);
    }

    std::pair<double, double> computeAverage()
    {
        double sum_lat = 0.0, sum_lon = 0.0;
        for (const auto& pt : valid_points_) {
            sum_lat += pt.first;
            sum_lon += pt.second;
        }
        int n = valid_points_.size();
        return { sum_lat / n, sum_lon / n };
    }

    void speedCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        current_speed = msg->point.y;
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        double lat = msg->latitude;
        double longit = msg->longitude;
        
        ros::Time now = ros::Time::now();

        if (!received_first_msg) {
        last_time = now;
        received_first_msg = true;
    }

        if (valid_points_.empty()) {
            addValidPoint(lat, longit);
            
            
        }

        auto last = valid_points_.back();

        if (abs(lat - last.first) > max_distance_threshold_ || abs(longit - last.second) > max_distance_threshold_){
            ROS_WARN("GPS outlier detected . Using average fallback.");
            auto avg = computeAverage();
            lat = avg.first;
            longit = avg.second;
            
        } else {
            addValidPoint(lat, longit);
            
        }
        
        
        // Settore 1
        if (std::abs(lat - sec1_lat) < tol && std::abs(longit - sec1_longit) < tol) {
            if (current_sector == 1) {
                last_time = now;
                v_average = 0;
                count = 0;
                current_sector = 2;
                ROS_INFO("Entrato nel settore 2");
            }
        }
        // Settore 2
        else if (std::abs(lat - sec2_lat) < tol && std::abs(longit - sec2_longit) < tol) {
            if (current_sector == 2) {
                last_time = now;
                v_average = 0;
                count = 0;
                current_sector = 3;
                ROS_INFO("Entrato nel settore 3");
            }
        }
        // Settore 3
        else if (std::abs(lat - sec3_lat) < tol && std::abs(longit - sec3_longit) < tol) {
            if (current_sector == 3) {
                last_time = now;
                v_average = 0;
                count = 0;
                current_sector = 1;
                ROS_INFO("Entrato nel settore 1");
            }
        }

        // Calcolo tempo e velocità media
        count++;
        v_average = ((v_average * (count - 1)) + current_speed) / count;

        first_project::custom msg_out;
        msg_out.current_sector = current_sector;
        msg_out.current_sector_time = (now - last_time).toSec();
        msg_out.current_sector_mean_speed = v_average;

        pub.publish(msg_out);
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "sector_times");
    SectorTimer sector_timer;
    ros::spin();
    return 0;
}

