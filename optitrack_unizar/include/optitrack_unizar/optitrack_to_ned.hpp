#include <ros/ros.h>
#include <tf/tf.h>

#include <geometry_msgs/Pose.h>

#include <tf2_ros/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class OptitrackToNed
{
private:
    ros::NodeHandle nh;
    ros::Subscriber subs;
    ros::Publisher pub;
    geometry_msgs::PoseStamped transformed_pose;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string tf_from;
    std::string tf_to;

public:

    OptitrackToNed();
    void poseCallback(geometry_msgs::PoseStamped recieved_pose);
    void timedPubCallback(void);


};