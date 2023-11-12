#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Joy.h>

#define POSITION_GAIN 1
#define YAW_GAIN 1

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
}

sensor_msgs::Joy current_joy;
void joy_cb(const sensor_msgs::Joy::ConstPtr& msg) {
    current_joy = *msg;
}

void arm(ros::ServiceClient& arming_client) {

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();

    ros::Rate rate(20.0);
    while (ros::ok()) {

        if (current_state.armed) {
            ROS_INFO("Vehicle armed");
            return;
        }

        if ((ros::Time::now() - last_request) > ros::Duration(5.0)) {
            last_request = ros::Time::now();
            ROS_INFO("Trying to arm...");
            arming_client.call(arm_cmd);
        }

        ros::spinOnce();
        rate.sleep();
    }
}

void takeOff(ros::Publisher& local_pos_pub, ros::ServiceClient& set_mode_client) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    ros::Time last_request = ros::Time::now();

    ros::Rate rate(20.0);
    while (ros::ok()) {

        double diff_z = pose.pose.position.z - current_pose.pose.position.z;
        ROS_INFO("diff_z = %lf", diff_z);

        if (fabs(diff_z) < 0.5) {
            ROS_INFO("Flying!");
            return;
        }

        if ((current_state.mode != "OFFBOARD") && ((ros::Time::now() - last_request) > ros::Duration(5.0))) {
            last_request = ros::Time::now();
            ROS_INFO("Enabling OFFBOARD...");
            set_mode_client.call(offb_set_mode);
        }

        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "joy_mavros_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 1, pose_cb);
    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>
            ("joy", 1, joy_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 10);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 1);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    ros::Rate rate(30.0);

    // Wait for FCU connection
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected!");

    arm(arming_client);
    takeOff(local_pos_pub, set_mode_client);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 angular;

    geometry_msgs::Twist vel;
    while (ros::ok()) {

        if (current_joy.axes.size() >= 4) {

            linear.x = POSITION_GAIN * current_joy.axes[3];
            linear.y = POSITION_GAIN * current_joy.axes[2];
            linear.z = POSITION_GAIN * current_joy.axes[1];
            angular.z = YAW_GAIN * current_joy.axes[0];
        }

        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        
        tf2::doTransform(linear, vel.linear, transformStamped);
        vel.angular = angular;

        vel_pub.publish(vel);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
