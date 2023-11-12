#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <airsim_ros_pkgs/VelCmd.h>
#include <airsim_ros_pkgs/Takeoff.h>

#define POSITION_GAIN 3
#define YAW_GAIN 1

sensor_msgs::Joy current_joy;
void joy_cb(const sensor_msgs::Joy::ConstPtr& msg) {
    current_joy = *msg;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "joy_airsim_node");
    ros::NodeHandle nh;

    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>
            ("joy", 1, joy_cb);
    ros::Publisher vel_pub = nh.advertise<airsim_ros_pkgs::VelCmd>
            ("/airsim_node/SimpleFlight/vel_cmd_body_frame", 1);
    ros::ServiceClient takeoff_client = nh.serviceClient<airsim_ros_pkgs::Takeoff>
            ("/airsim_node/SimpleFlight/takeoff");

    ros::Rate rate(30.0);
    airsim_ros_pkgs::VelCmd vel;
    while (ros::ok()) {

        if (current_joy.axes.size() >= 4) {
            vel.twist.linear.x =  POSITION_GAIN * current_joy.axes[3];
            vel.twist.linear.y = -POSITION_GAIN * current_joy.axes[2];
            vel.twist.linear.z = -POSITION_GAIN * current_joy.axes[1];
            vel.twist.angular.z = -YAW_GAIN * current_joy.axes[0];
        }

        vel_pub.publish(vel);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
