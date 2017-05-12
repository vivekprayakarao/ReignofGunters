#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(2.0);
  

  moveit::planning_interface::MoveGroup group("arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  ROS_INFO("Group names: %s", 	group.getName().c_str());

  std::cout<<group.getCurrentPose();


  /*geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x =  -0.993009;
  target_pose1.orientation.y = 0.0457311;
  target_pose1.orientation.z = 0.108702;
  target_pose1.orientation.w = 0.005008;
  target_pose1.position.x = 0.259775;
  target_pose1.position.y = -0.0184402;
  target_pose1.position.z =  0.256358;*/

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x =  0.922849;
  target_pose1.orientation.y = 0.00235894;
  target_pose1.orientation.z = -0.385154;
  target_pose1.orientation.w = 0.00098235;
  target_pose1.position.x = 0.166023;
  target_pose1.position.y = 0.000541269;
  target_pose1.position.z =  0.273873;
  group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    

  //sleep(5.0);
 
  if (1)
  {
    ROS_INFO("Visualizing plan 1 (again)");    
    display_trajectory.trajectory_start = my_plan.start_state_;
    display_trajectory.trajectory.push_back(my_plan.trajectory_);
    display_publisher.publish(display_trajectory);
    sleep(5.0);
  }
  
  // Moving to a pose goal
 
  /* Uncomment below line when working with a real robot*/
  group.move();
  sleep(2.0);

  geometry_msgs::Pose target_pose2;
  target_pose2.orientation.x =  0.880169;
  target_pose2.orientation.y = 0.136085;
  target_pose2.orientation.z = -0.449396;
  target_pose2.orientation.w = 0.0694798;
  target_pose2.position.x = 0.29454;
  target_pose2.position.y = 0.0743012;
  target_pose2.position.z =  0.147096;
  group.setPoseTarget(target_pose2);
  
  // Moving to a pose goal
 
  /* Uncomment below line when working with a real robot*/
  group.move();

  sleep(2.0);

  geometry_msgs::Pose target_pose3;
  target_pose3.orientation.x =  -0.844858;
  target_pose3.orientation.y = 0.237063;
  target_pose3.orientation.z = 0.461766;
  target_pose3.orientation.w = 0.129571;
  target_pose3.position.x = 0.283182;
  target_pose3.position.y = -0.135952;
  target_pose3.position.z =  0.103461;
  group.setPoseTarget(target_pose3);

  group.move();

  sleep(2.0);
  geometry_msgs::Pose target_pose4;
  target_pose4.orientation.x =  0.922849;
  target_pose4.orientation.y = 0.00235894;
  target_pose4.orientation.z = -0.385154;
  target_pose4.orientation.w = 0.00098235;
  target_pose4.position.x = 0.166023;
  target_pose4.position.y = 0.000541269;
  target_pose4.position.z =  0.273873;
  group.setPoseTarget(target_pose4);

  group.move();
  ros::shutdown();  
  return 0;
}
