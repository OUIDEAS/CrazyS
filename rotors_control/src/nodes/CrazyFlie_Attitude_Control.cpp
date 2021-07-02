#include "position_controller_node.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/default_topics.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <time.h>
#include <chrono>
#include "rotors_control/parameters_ros.h"
#include "rotors_control/stabilizer_types.h"
#include "rotors_control/crazyflie_complementary_filter.h"
#include "std_msgs/Float64.h"


class Odom
{
    public:
        // Initiate node
        ros::NodeHandle n;
        ros::Subscriber sub;
        double posx, posy, posz, rot_x, rot_y, rot_z, vx, vy, vz, w_x, w_y, w_z, w, x, y, z;
        
        Odom(): posx(0), posy(0), posz(0), rot_x(0), rot_y(0), rot_z(0), vx(0), vy(0), vz(0), w_x(0), w_y(0), w_z(0)
        {
            // Subscribe to Odometry
            sub = n.subscribe(mav_msgs::default_topics::ODOMETRY, 1, &Odom::OdometryCallback, this);
        }

        void OdometryCallback(const nav_msgs::OdometryConstPtr &msg)
        {
            // Store Position Data
            this->posx = msg->pose.pose.position.x;
            this->posy = msg->pose.pose.position.y;
            this->posz = msg->pose.pose.position.z;

            // Read Orientation in Quaternions
            w = msg->pose.pose.orientation.w;
            x = msg->pose.pose.orientation.x;
            y = msg->pose.pose.orientation.y;
            z = msg->pose.pose.orientation.z;

            // Quaternions to Euler Angles
            double sinr_cosp = 2 * (w*x + y*z);
            double cosr_cosp = 1 - 2 * (x*x + y*y);
            double sinp = 2 * (w*y - z*x);
            double siny_cosp = 2 * (w*z + x*y);
            double cosy_cosp = 1 - 2 * (y*y + z*z);

            // Store Orientation Data in Euler Angles
            // pitch
            if (std::abs(sinp) > 1){
                this->rot_y = std::copysign(M_PI/2, sinp);
            }else{
                this->rot_y = std::asin(sinp);
            }
            // roll
            this->rot_x = std::atan2(sinr_cosp, cosr_cosp);
            
            // yaw
            this->rot_z = std::atan2(siny_cosp, cosy_cosp);
            
            // Store Linear Velocity data
            this->vx = msg->twist.twist.linear.x;
            this->vy = msg->twist.twist.linear.y;
            this->vz = msg->twist.twist.linear.z;

            // Store Angular Velocities 
            this->w_x = msg->twist.twist.angular.x;
            this->w_y = msg->twist.twist.angular.y;
            this->w_z = msg->twist.twist.angular.z;
        }
};

int main(int argc, char **argv){
    // Initiate node
    ros::init(argc, argv, "CrazyFlie_Attitude_Control");
    ros::NodeHandle r;
    ros::Publisher roll_pub  = r.advertise<std_msgs::Float64>("roll", 1, true);
    ros::Publisher pitch_pub = r.advertise<std_msgs::Float64>("pitch", 1, true); 
    ros::Publisher yaw_pub   = r.advertise<std_msgs::Float64>("yaw", 1, true); 

    // Subscribes to the Odometry using Odom Class
    // Values obtained directly from Gazebo
    Odom Gazebo;
    
    double P, I, D, err_x, err_y, err_vx, err_vy, target_vx, target_vy, target_posx, target_posy, ax, ay, dt, vx_prev, vy_prev, roll, pitch, yaw;
    std_msgs::Float64 target_roll;
    std_msgs::Float64 target_pitch;
    std_msgs::Float64 target_heading;

    P = 1;
    I = 0;
    D = 2;
    err_x = 0;
    err_y = 0;
    err_vx = 0;
    err_vy = 0;

    target_posx = 10;
    target_posy = 10;

    double last_time = ros::Time::now().toSec();
    double current_time = ros::Time::now().toSec();

    while(ros::ok())
    {
        ax = (Gazebo.vx - vx_prev) / dt;
        ay = (Gazebo.vy - vy_prev) / dt;

        dt = current_time - last_time;
        last_time = ros::Time::now().toSec();

        // **********************************************************
        // Position PID
        target_vx = P * (target_posx - Gazebo.posx) + I * err_x + D * (0 - Gazebo.vx);
        target_vy = P * (target_posy - Gazebo.posy) + I * err_y + D * (0 - Gazebo.vy);
        // **********************************************************

        // **********************************************************
        // Attitude PID
        roll = P * (target_vy - Gazebo.vy) + I * err_vy + D * (0 - ay);
        pitch = P * (target_vx - Gazebo.vx) + I * err_vx + D * (0 - ax);
        // **********************************************************

        vx_prev = Gazebo.vx;
        vy_prev = Gazebo.vy;

        target_roll.data = roll; //roll;
        target_pitch.data = pitch; //pitch;
        target_heading.data = 0; //yaw

        // **********************************************************
        // Publish Target Attitude
        roll_pub.publish(target_roll);
        pitch_pub.publish(target_pitch);
        yaw_pub.publish(target_heading);
        // **********************************************************

        ros::spinOnce();
        // Operate at 100 Hz
        ros::Rate(100.0).sleep();
        double current_time = ros::Time::now().toSec();
    }
    return 0;
}
    
