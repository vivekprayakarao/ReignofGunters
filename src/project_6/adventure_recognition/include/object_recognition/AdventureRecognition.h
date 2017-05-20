#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/Twist.h"
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "object_recognition/ObjectRecognition.h"
#include<vector>

float diagonal_size_thresh;
class AdventureRecognition {
private:
ros::NodeHandle nh_;
image_transport::ImageTransport it_;
image_transport::Subscriber image_sub_;
ObjectRecognition objrec;
cv_bridge::CvImagePtr cv_ptr;;
ros::Publisher vel_pub;
ros::Publisher display_publisher;
geometry_msgs::Pose target_pose1, target_pose2, target_pose3, target_pose4;

void setupArmTrajectory();
void setupWaypoints();
void attacknow();
bool moveToGoal(double xGoal, double yGoal);

std::vector<std::vector<double> > waypoints;
float init_error;
int waypt_num;
bool attacked_once;
bool process_image;
bool move_now;
bool attack_now;
bool bottle_is_near;

// For Keeping a track of bottle number
int count;

moveit::planning_interface::MoveGroup group;
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac;

public:
AdventureRecognition(ros::NodeHandle n_);
void imageCb(const sensor_msgs::ImageConstPtr& msg);
};
