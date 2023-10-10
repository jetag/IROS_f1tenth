#include <sstream>
#include <string>
#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <Eigen/Geometry>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


using std::placeholders::_1;
using namespace std;

class PurePursuit : public rclcpp::Node
{
private:
    string path = "/home/nvidia/f1tenth_ws/src/pure_pursuit/checkpoints/";
    string filename = "final_raceline.csv";
    vector<vector<double>> waypoints;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_goal_pub ;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_path_pub ;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::ConstSharedPtr scan_sub;

    // hyperparameters here
    double look_ahead, kp, vel_min, vel_max, vel_inter, velocity, ttc, scale;
    double pure_x,pure_y, base_x, base_y, yaw, steering_angle;
    bool drive_flag;

    void set_hyperparamets()
    {
        rclcpp::Parameter look_ahead_param = this->get_parameter("look_ahead");
        rclcpp::Parameter kp_param = this->get_parameter("kp");
        rclcpp::Parameter vel_min_param = this->get_parameter("vel_min");
        rclcpp::Parameter vel_max_param = this->get_parameter("vel_max");
        rclcpp::Parameter vel_inter_param = this->get_parameter("vel_inter");
        rclcpp::Parameter drive_flag_param = this->get_parameter("drive_flag");
        rclcpp::Parameter ttc_param = this->get_parameter("ttc");
        rclcpp::Parameter scale_param = this->get_parameter("scale");


        look_ahead = look_ahead_param.as_double();
        kp = kp_param.as_double();
        vel_min = vel_min_param.as_double();
        vel_max = vel_max_param.as_double();
        vel_inter = vel_inter_param.as_double();
        drive_flag = drive_flag_param.as_bool();
        ttc = ttc_param.as_double();
        scale = scale_param.as_double();
    }

public:
    PurePursuit() : Node("pure_pursuit_node")
    {
        
        // pf_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&WallFollow::scan_callback, this, _1));
        this->declare_parameter("look_ahead", 1.8);
        this->declare_parameter("kp", 0.75);
        this->declare_parameter("vel_min", 3.0);
        this->declare_parameter("vel_max", 6.0);
        this->declare_parameter("vel_inter", 4.0);
        this->declare_parameter("ttc", 2.0);
        this->declare_parameter("drive_flag", true);
        this->declare_parameter("scale", 0.77);
        


        drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
        vis_goal_pub = this->create_publisher<visualization_msgs::msg::Marker>( "visualization_goal_topic", 10 );
        vis_path_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>( "visualization_path_topic", 10);
        scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>( "scan_viz", 10);        
	
	    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/pf/pose/odom", 10, std::bind(&PurePursuit::pose_callback, this, _1));
        scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&PurePursuit::scan_callback, this, _1));


        parseCSV();
    }

    double get_vel(double x)
    {
        double steer_max = 0.5;
        double a = (vel_min - vel_max) / (steer_max*steer_max);
        return a * (x*x) + vel_max;
    }
    
    double deg_to_rad(float deg){
        return deg*M_PI/180;
    }
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        //get ranges in a float array
        std::vector<float> ranges = scan_msg->ranges;
        float angle_max = scan_msg->angle_max;
        float angle_min = scan_msg->angle_min;
        float angle_increment = scan_msg->angle_increment;
        float fov = deg_to_rad(30);

        //calc idx for angle range
        int idx_min = (int) ((angle_max-fov)/angle_increment);
        int idx_max = (int) ((angle_max+fov)/angle_increment);

        check_pure_pursuit();
        //velocity = 4.0;
        int controller_flag = 0;
        for (int i=0;i<ranges.size();i++){
            if (i<idx_max && i>idx_min ){
                float angle = angle_min + i*angle_increment;
                float angle_vel = max(velocity* cos(angle),0.001);
                float ttc_now = ranges[i]/angle_vel;

                // RCLCPP_INFO(this->get_logger(), "TTC: %f", ttc_now);
                if (ttc_now < ttc && ranges[i]<2.0){
                    controller_flag = 1;
                }
                else{
                    controller_flag = 0;
                }
            }
        }

        if (controller_flag == 0){
            publish_pure_pursuit();
            //publish_gap_follow(ranges);

        }
        else{
            publish_pure_pursuit();
            //publish_gap_follow(ranges);
            
        }

    }
    void publish_gap_follow(std::vector<float> ranges){

        RCLCPP_INFO(this->get_logger(), "Doing gap follow");
        
        float max_gap_dist = 2.0;
        //preprocess ranges
        std::vector<float> ranges_processed;
        for (int i=0;i<ranges.size();i++){
            if (ranges[i] > max_gap_dist){
                ranges_processed.push_back(max_gap_dist);
            }
            else{
                ranges_processed.push_back(ranges[i]);
            }
        }
        //viz scan
        auto scan_msg = sensor_msgs::msg::LaserScan();
        scan_msg.header.frame_id = "ego_racecar/laser";
        scan_msg.header.stamp = rclcpp::Clock().now();
        scan_msg.angle_min = -2.3499;
        scan_msg.angle_max = 2.3499;
        scan_msg.angle_increment = 0.00435;
        scan_msg.range_min = 0.0;
        scan_msg.range_max = 10.0;
        scan_msg.ranges = ranges_processed;
        scan_pub->publish(scan_msg);

        //find max gap
        int max_gap = 0;
        int max_gap_idx = 0;
        int gap = 0;
        for (int i=0;i<ranges_processed.size();i++){
            if (ranges_processed[i] == max_gap_dist){
                gap += 1;
            }
            else{
                if (gap > max_gap){
                    max_gap = gap;
                    max_gap_idx = i;
                }
                gap = 0;
            }
        }


        //start and end idx of max gap
        int start_idx = max_gap_idx - max_gap;
        int end_idx = max_gap_idx;
    
        //find mid point of max gap
        int mid_idx = (int)(start_idx + end_idx)/2;

        //find angle of mid point
        float angle_min = -2.3499;
        float angle_increment = 0.00435;
        float angle = angle_min + mid_idx*angle_increment;

        // RCLCPP_INFO(this->get_logger(), "angle: %f", angle*(180/M_PI));

        //publish on drive_pub
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = angle;
        
        //velocity = 4.0;

        if (drive_flag){
            drive_msg.drive.speed = velocity*scale;
            // RCLCPP_INFO(this->get_logger(), "VeL: %f", drive_msg.drive.speed);
        }
        drive_pub->publish(drive_msg);
    }


    void check_pure_pursuit(){
        double min_dist = 100000;
        double min_y = 0;
        int min_idx = 0;
        
        for(int i=0; i<waypoints.size(); ++i)
        {
            //always checks in robot origin frame
            double x_local = (waypoints[i][0]-base_x) * cos(yaw) + (waypoints[i][1]-base_y) * sin(yaw);
            double y_local = -(waypoints[i][0]-base_x) * sin(yaw) + (waypoints[i][1]-base_y) * cos(yaw);
            double dist = sqrt(x_local*x_local + y_local*y_local);
            double theta = atan2(y_local, x_local);
            if(  (dist >= look_ahead && dist < min_dist) && (-M_PI/4 < theta && theta < M_PI/4)  )
            {
                min_dist = dist;
                min_idx = i;
                min_y = y_local;
            }            
        }
        pure_x = waypoints[min_idx][0];
        pure_y = waypoints[min_idx][1];
        steering_angle = kp* 2 * min_y / (min_dist*min_dist); 
        velocity = scale*waypoints[min_idx][2];
        
    }
    void publish_pure_pursuit(){
        //turn radius from pure pursuit logic
        RCLCPP_INFO(this->get_logger(), "Doing pure pursuit");

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = steering_angle;

        if (drive_flag){
            drive_msg.drive.speed = velocity;
            // RCLCPP_INFO(this->get_logger(), "VeL: %f", drive_msg.drive.speed);
        }

        drive_pub->publish(drive_msg);
    }

    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) // use this for sim, odom
    { 
      
        set_hyperparamets();
        geometry_msgs::msg::Pose::ConstSharedPtr pose_msg = std::make_shared<geometry_msgs::msg::Pose>(odom_msg->pose.pose); // use this only for sim

        tf2::Quaternion q(pose_msg->orientation.x, pose_msg->orientation.y, pose_msg->orientation.z, pose_msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll,pitch,yaw);

        base_x = pose_msg->position.x;
        base_y = pose_msg->position.y;
        

        // visualize goal marker
        auto marker = visualization_msgs::msg::Marker();
        marker.header = odom_msg->header;
        marker.header.frame_id = "map";
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;
        marker.color.a = 1.0; 
        marker.color.r = 1.0;
        marker.pose.position.x = pure_x;
        marker.pose.position.y = pure_y;
        marker.id += 1;
        vis_goal_pub->publish(marker);
    }

    void visualize_path_points()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Clock().now();
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;
        marker.color.a = 1.0; 
        marker.color.g = 1.0;

        for(int i=0; i<waypoints.size(); ++i)
        {
            marker.pose.position.x = waypoints[i][0];
            marker.pose.position.y = waypoints[i][1];
            marker.id = i;
            marker_array.markers.push_back(marker);
        }

        vis_path_pub->publish(marker_array);
    }

    void parseCSV()
    {
        vector<double> row;
        string line, word;
     
        fstream file(path+filename, ios::in);
        if(file.is_open())
        {
            while(getline(file, line))
            {
                row.clear();
     
                stringstream str(line);
     
                while(getline(str, word, ','))
                    row.push_back(stod(word)); // string to double
                waypoints.push_back(row);
            }

            visualize_path_points();
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "\n\n!!!File could not be opened!!!\n");
        }
    }

    ~PurePursuit() {}
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}
