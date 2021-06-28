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

            // Store Orientation Data
            w = msg->pose.pose.orientation.w;
            x = msg->pose.pose.orientation.x;
            y = msg->pose.pose.orientation.y;
            z = msg->pose.pose.orientation.z;
            double sinr_cosp = 2 * (w*x + y*z);
            double cosr_cosp = 1 - 2 * (x*x + y*y);
            double sinp = 2 * (w*y - z*x);
            double siny_cosp = 2 * (w*z + x*y);
            double cosy_cosp = 1 - 2 * (y*y + z*z);
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
    ros::init(argc, argv, "CrazyFlie_Position_Control");
    ros::NodeHandle POSCTL;

    // Let gazebo fully start up before sending motor commands
    ros::Duration(2.0).sleep(); 

    // Subscribes to the Odometry using Odom Class
    // Values obtained directly from Gazebo
    Odom Gazebo; 
    
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
    double P_z, I_z, D_z, P_r, I_r, D_r, P_p, I_p, D_p, P_y, I_y, D_y, PID_hover, PID_roll, PID_pitch, PID_yaw, Err, satmax, satmin, HoverSpeed, setpoint;

    // Counter Variable
    int c = 1;

    

    // Motor Saturation Limits
    //satmax = 2450;
    satmin = 0;


    // Set PID values
    P_z = 1250;
    I_z = 1000;//2;
    D_z = 1500;//.3;
    Err = 0;
    P_r = 100;
    I_r = 100;
    D_r = 100;
    P_p = 100;
    I_p = 100;
    D_p = 100;
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
        dt = current_time - last_time;
        current_time = ros::Time::now().toSec();
        Err += (2.0 - Gazebo.posz)*dt*-1;
        ROS_INFO("%f", Gazebo.posz);
        actuator_msg->angular_velocities.clear();
        //ROS_INFO("SETPOINT: [%f], POSITION: [%f]", setpoint, Gazebo.posz);
        // Create a PID controller to set Rotor speeds to achieve desired altitude
      
        PID_hover = P_z * (2.0 - Gazebo.posz) + I_z * Err + D_z * (0 - Gazebo.vz);
        PID_roll = P_r * (0 - Gazebo.rot_x) + D_r * (0 - Gazebo.w_x);
        PID_pitch = P_p * (0 - Gazebo.rot_y) + D_p * (0 - Gazebo.w_y);

        if((PID_hover)<satmin){
            PID_hover = satmin;
        }

        r1 = PID_hover - PID_roll - PID_pitch - PID_yaw;
        r2 = PID_hover - PID_roll + PID_pitch + PID_yaw;
        r3 = PID_hover + PID_roll + PID_pitch - PID_yaw;
        r4 = PID_hover + PID_roll - PID_pitch + PID_yaw;

        ROS_INFO("%f",r1);

        actuator_msg->angular_velocities.push_back(r1);
        actuator_msg->angular_velocities.push_back(r2);
        actuator_msg->angular_velocities.push_back(r3);
        actuator_msg->angular_velocities.push_back(r4);
        
        motor_cmd.publish(actuator_msg);
        ros::spinOnce();
        c++;       
        last_time = ros::Time::now().toSec(); 
        
    }

    return 0;
}