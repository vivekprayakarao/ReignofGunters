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
  sleep(20.0);
  

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


  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = 0.302278;
  target_pose1.orientation.y = -0.639938;
  target_pose1.orientation.z = 0.301736;
  target_pose1.orientation.w = 0.638798;
  target_pose1.position.x = 0.0351457;
  target_pose1.position.y = -0.0302251;
  target_pose1.position.z = 0.442074;
  group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    

  sleep(5.0);
 
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
  
  ros::shutdown();  
  return 0;
}

