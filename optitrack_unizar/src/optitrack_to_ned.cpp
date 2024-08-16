#include <ros/ros.h>
#include <tf/tf.h>

#include <geometry_msgs/Pose.h>

#include <tf2_ros/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <optitrack_unizar/optitrack_to_ned.hpp>

OptitrackToNed::OptitrackToNed() : tf_listener_(tf_buffer_, nh)
{
    nh = ros::NodeHandle();
    // TODO: This is currently using public topics. It should be avoided. Use remap in launchfile instead
    subs = nh.subscribe<geometry_msgs::PoseStamped>("/optitrack/pose", 10, &OptitrackToNed::poseCallback, this);
    pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

    nh.param<std::string>("tf_from", tf_from, "odom");
    nh.param<std::string>("tf_to", tf_to, "odom_ned");
}

void OptitrackToNed::poseCallback(geometry_msgs::PoseStamped recieved_pose)
{
    geometry_msgs::TransformStamped tf_odom_odomNED;
    try
    {
        tf_odom_odomNED = tf_buffer_.lookupTransform(tf_from, tf_to, ros::Time(0));
    }
    catch (tf2::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
    tf2::doTransform(recieved_pose, transformed_pose, tf_odom_odomNED);
    transformed_pose.header.frame_id = tf_to;
    transformed_pose.header.stamp = recieved_pose.header.stamp;
}

void OptitrackToNed::timedPubCallback(void)
{
    pub.publish(transformed_pose);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "optitrack_to_ned");
    OptitrackToNed optitrack_to_ned;
    ros::Rate loop_rate(40);
    while (ros::ok())
    {
        optitrack_to_ned.timedPubCallback();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
