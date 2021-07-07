// Bob Geng

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
#include <std_srvs/Empty.h>
#include <thread>

#define ATTITUDE_UPDATE_DT 0.004  /* ATTITUDE UPDATE RATE [s] - 500Hz */
#define RATE_UPDATE_DT 0.002      /* RATE UPDATE RATE [s] - 250Hz */
#define SAMPLING_TIME  0.01       /* SAMPLING CONTROLLER TIME [s] - 100Hz */

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

class Target_Pitch
{
    public:
        ros::NodeHandle p;
        ros::Subscriber pitch_sub;
        double pitch;

        Target_Pitch(): pitch(0)
        {
            pitch_sub = p.subscribe("/crazyflie2/pitch", 1, &Target_Pitch::pitch_callback, this);
        }
        void pitch_callback(const std_msgs::Float64::ConstPtr& msg)
        {
            this->pitch = msg->data;
        }
};

class Target_Roll
{
    public:
        ros::NodeHandle r;
        ros::Subscriber roll_sub;
        double roll;

        Target_Roll(): roll(0)
        {
            roll_sub = r.subscribe("/crazyflie2/roll", 1, &Target_Roll::roll_callback, this);
        }
        void roll_callback(const std_msgs::Float64::ConstPtr& msg)
        {
            this->roll = msg->data;
        }
};

class Target_Yaw
{
    public:
        ros::NodeHandle y;
        ros::Subscriber yaw_sub;
        double yaw;

        Target_Yaw(): yaw(0)
        {
            yaw_sub = y.subscribe("/crazyflie2/yaw", 1, &Target_Yaw::yaw_callback, this);
        }
        void yaw_callback(const std_msgs::Float64::ConstPtr& msg)
        {
            this->yaw = msg->data;
        }
};

int main(int argc, char **argv){
    // Initiate node
    ros::init(argc, argv, "CrazyFlie_Position_Control");
    ros::NodeHandle POSCTL;

    std_srvs::Empty srv;
    bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);

    while(!unpaused){
        std::this_thread::sleep_for(std::chrono::seconds(1));
        unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    }
    // Let gazebo fully start up before sending motor commands
    ros::Duration(2.0).sleep(); 

    // Subscribes to the Odometry using Odom Class
    // Values obtained directly from Gazebo
    Odom Gazebo; 
    Target_Roll theta;
    Target_Pitch phi;
    Target_Yaw heading;
    // Establish a publisher for motor commands
    ros::Publisher motor_cmd = POSCTL.advertise<mav_msgs::Actuators>(mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

    // Set up message to send to actuators
    mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

    // Clear acuator_msg fields
    actuator_msg->angular_velocities.clear();
    actuator_msg->angles.clear();
    actuator_msg->normalized.clear();

    // Rotor commands
    double r1, r2, r3, r4;

    // Controller Parameters
    double P_z, I_z, D_z, P_r, I_r, D_r, P_p, I_p, D_p, P_y, I_y, D_y, PID_hover, PID_roll, PID_roll_set, PID_pitch, PID_pitch_set, PID_yaw, Err, satmax, satmin, HoverSpeed, setpoint;

    // Motor Saturation Limits
    satmax = 5000;
    satmin = 0;

    // Set PID values (MOVE TO YAML file)
    P_z = 1250;
    I_z = 100;
    D_z = 1500;
    Err = 0;
    P_r = 50;
    I_r = 0;
    D_r = 500;
    P_p = 50;
    I_p = 0;
    D_p = 500;
    P_y = 0;
    I_y = 0;
    D_y = 0;
    PID_roll = 0;
    PID_pitch = 0;
    PID_yaw = 0;
    HoverSpeed = 2273;

    double last_time = ros::Time::now().toSec();
    double current_time = ros::Time::now().toSec();
    double dt;
    while(ros::ok())
    {   
        // Calculate dt for Integral term
        dt = current_time - last_time;
        last_time = ros::Time::now().toSec();

        //Calculate Error
        Err += (2.0 - Gazebo.posz)*dt;

        // Clear motor velocities before setting new ones
        actuator_msg->angular_velocities.clear();
    
        PID_hover = P_z * (2.0 - Gazebo.posz) + I_z * Err + D_z * (0 - Gazebo.vz);
        PID_pitch = P_p * (phi.pitch - Gazebo.rot_y) + D_p * (0 - Gazebo.w_y);
        PID_roll = P_r * (theta.roll - Gazebo.rot_x) + D_r * (0 - Gazebo.w_x); 
        PID_yaw = P_y * (heading.yaw - Gazebo.rot_z) + D_y * (0 - Gazebo.w_z);
        // Set motor velocities
        r1 = PID_hover - PID_roll/2 - PID_pitch/2 + PID_yaw/2;
        r2 = PID_hover - PID_roll/2 + PID_pitch/2 - PID_yaw/2;
        r3 = PID_hover + PID_roll/2 + PID_pitch/2 + PID_yaw/2;
        r4 = PID_hover + PID_roll/2 - PID_pitch/2 - PID_yaw/2;

        if(r1 < satmin){
            r1 = satmin;
        }else if (r1 > satmax){
            r1 = satmax;
        }
        if(r2 < satmin){
            r2 = satmin;
        }else if (r2 > satmax){
            r2 = satmax;
        }
        if(r3 < satmin){
            r3 = satmin;
        }else if (r3 > satmax){
            r3 = satmax;
        }
        if(r4 < satmin){
            r4 = satmin;
        }else if (r4 > satmax){
            r4 = satmax;
        }
        
        // Send motor commands to motors
        actuator_msg->angular_velocities.push_back(r1);
        actuator_msg->angular_velocities.push_back(r2);
        actuator_msg->angular_velocities.push_back(r3);
        actuator_msg->angular_velocities.push_back(r4);
        motor_cmd.publish(actuator_msg);
        
        ros::spinOnce();
        // Set internal loop to 500 Hz       
        ros::Rate(500.0).sleep();
        current_time = ros::Time::now().toSec();
    }
    return 0;
}