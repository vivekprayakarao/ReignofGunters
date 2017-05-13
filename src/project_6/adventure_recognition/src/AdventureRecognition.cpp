#include "object_recognition/AdventureRecognition.h"

AdventureRecognition::AdventureRecognition(ros::NodeHandle n_)
    : it_(n_), group("arm")
{
    this->nh_ = n_;
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
      &AdventureRecognition::imageCb, this);
    // image_pub_ = it_.advertise("/image_converter/output_video", 1);
    process_image = false;
    //bottle_positioned_correctly = false;
    bottle_is_near = false;
    attack_now = false;
    move_now = true;
    attacked_once = false;
    waypt_num = 0;
    init_error = -1;

    try {
         this->tf_listener_odom_footprint.waitForTransform( "/odom","/base_footprint",ros::Time(0), ros::Duration(50.0) );
     }
  catch (tf::TransformException &ex) {
            ROS_ERROR("[adventure_slam]: (wait) %s", ex.what());
            ros::Duration(1.0).sleep();
  }

  this->vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 10);
  this->display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

  setupArmTrajectory();
  setupWaypoints();
}

void AdventureRecognition::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
     geometry_msgs::Twist twist_msg;
     if (move_now)
     { 
        ROS_INFO("Came in move now");
        moveToGoal(waypoints[waypt_num][0], waypoints[waypt_num][1]);
     }
 
    //cv_bridge::CvImagePtr cv_ptr;
   if (process_image) //Should be activated after a particular waypoint is attained
    {      
       float kw = 0.0001;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
   
       std::vector<float> bbox_centroid;
       float diagonal;
       if (objrec.findMatchingFeatures(cv_ptr->image, bbox_centroid, diagonal))
       {
         //TODO:Check whether aspect ratio changed a lot, skip this frame

         // Update GUI Window
          cv::imshow(OPENCV_WINDOW, objrec.img_matches);
          cv::waitKey(3);
          
           ROS_INFO("The diagonal is %f ", diagonal);
           if (diagonal > diagonal_size_thresh)
               bottle_is_near = true;

          //Code for checking Bottle Positioned Correctly
           if(bbox_centroid[0] - (cv_ptr->image.cols) / 2 < 0)
		init_error = -1;
           else
                init_error = 1;

	   twist_msg.linear.x = 0.1;
	   twist_msg.angular.z = 0;
	   vel_pub.publish(twist_msg);
           //ROS_INFO("The error is %f", init_err);

           if (!bottle_is_near)
           {
              twist_msg.linear.x = 0.1;
	      twist_msg.angular.z = 0;
	      vel_pub.publish(twist_msg);
           }
           else 
           {
 	     twist_msg.linear.x = 0;
	     twist_msg.angular.z = 0;
	     vel_pub.publish(twist_msg);
           }

       }
       else
       {
          if (attacked_once)
          {
              attacked_once = false;
              move_now = true;
              process_image = false;
              ROS_INFO("Change the bottle");
              objrec.setObjectKeyPoints();
          }
          else
          {
             twist_msg.linear.x = 0;
      	     twist_msg.angular.z = 0.23 * init_error;
             vel_pub.publish(twist_msg);
          }
       }
    }
       //else show a blank screen - "Not Processing at the moment sort of message"

  if (bottle_is_near)
  {
    attacknow();
    attacked_once = true;
    bottle_is_near = false;
  }
    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
}

bool AdventureRecognition::moveToGoal(double xGoal, double yGoal){

   //define a client for to send goal requests to the move_base server through a SimpleActionClient
   actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

   //wait for the action server to come up
   while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
   }

   move_base_msgs::MoveBaseGoal goal;

   //set up the frame parameters
   goal.target_pose.header.frame_id = "map";
   goal.target_pose.header.stamp = ros::Time::now();

   /* moving towards the goal*/

   goal.target_pose.pose.position.x =  xGoal;
   goal.target_pose.pose.position.y =  yGoal;
   goal.target_pose.pose.position.z =  0.0;
   goal.target_pose.pose.orientation.x = 0.0;
   goal.target_pose.pose.orientation.y = 0.0;
   goal.target_pose.pose.orientation.z = 0.0;
   goal.target_pose.pose.orientation.w = 1.0;

   ROS_INFO("Sending goal location ...");
   ac.sendGoal(goal);

   ac.waitForResult();

   if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("You have reached the destination");
      this->process_image = true;
      this->move_now = false;
      this->waypt_num += 1;
      return true;
   }
   else{
      ROS_INFO("The robot failed to reach the destination");
      return false;
   }

}

void AdventureRecognition::setupArmTrajectory()
{
  //Pose 1
  target_pose1.orientation.x =  0.922849;
  target_pose1.orientation.y = 0.00235894;
  target_pose1.orientation.z = -0.385154;
  target_pose1.orientation.w = 0.00098235;
  target_pose1.position.x = 0.166023;
  target_pose1.position.y = 0.000541269;
  target_pose1.position.z =  0.273873;

  //Pose 2
  target_pose2.orientation.x =  0.912501;
  target_pose2.orientation.y = 0.251016;
  target_pose2.orientation.z = -0.311437;
  target_pose2.orientation.w = 0.0856695;
  target_pose2.position.x = 0.304445;
  target_pose2.position.y = 0.145496;
  target_pose2.position.z = 0.0913248 ;
  
  //Pose 3
  target_pose3.orientation.x =  -0.907507;
  target_pose3.orientation.y = 0.334166;
  target_pose3.orientation.z = 0.238813;
  target_pose3.orientation.w = 0.0879377;
  target_pose3.position.x = 0.288953;
  target_pose3.position.y = -0.19506;
  target_pose3.position.z =  0.11154;

  //Pose 4
  target_pose4.orientation.x =  0.922849;
  target_pose4.orientation.y = 0.00235894;
  target_pose4.orientation.z = -0.385154;
  target_pose4.orientation.w = 0.00098235;
  target_pose4.position.x = 0.166023;
  target_pose4.position.y = 0.000541269;
  target_pose4.position.z =  0.273873;

  
}

void AdventureRecognition::setupWaypoints()
{
   /** declare the coordinates of interest **/
   double x_waypoint0 = 2.7;
   double y_waypoint0= -1.6;
   double x_waypoint1 = 0.4 ;
   double y_waypoint1= 0.0;
   double x_waypoint2 = 0.7 ;
   double y_waypoint2= -1.6;
   double x_waypoint3 = 0.0 ;
   double y_waypoint3= 0.0;

   std::vector<double> waypoint;
   waypoint.push_back(x_waypoint0);
   waypoint.push_back(y_waypoint0);
   waypoints.push_back(waypoint);
   waypoint.clear();

  
   waypoint.push_back(x_waypoint1);
   waypoint.push_back(y_waypoint1);
   waypoints.push_back(waypoint);
   waypoint.clear();

   waypoint.push_back(x_waypoint2);
   waypoint.push_back(y_waypoint2);
   waypoints.push_back(waypoint);
   waypoint.clear();
 
   waypoint.push_back(x_waypoint3);
   waypoint.push_back(y_waypoint3);
   waypoints.push_back(waypoint);
   waypoint.clear();
  
}

void AdventureRecognition::attacknow()
{
  ROS_INFO("The attack function called");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(2.0);

  //moveit::planning_interface::MoveGroup group("arm");
  //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  ROS_INFO("Group names: %s", 	group.getName().c_str());

  std::cout<<group.getCurrentPose();

  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    

  //sleep(5.0);
  moveit_msgs::DisplayTrajectory display_trajectory;
  
  if (1)
  {
    ROS_INFO("Visualizing plan 1 (again)");    
    display_trajectory.trajectory_start = my_plan.start_state_;
    display_trajectory.trajectory.push_back(my_plan.trajectory_);
    display_publisher.publish(display_trajectory);
    sleep(5.0);
  }
  
  // Moving to target poses
  group.setPoseTarget(target_pose1);
  group.move();
  sleep(2.0);
  group.setPoseTarget(target_pose2);
  group.move();
  sleep(2.0);
  group.setPoseTarget(target_pose3);
  group.move();
  sleep(2.0);
  group.setPoseTarget(target_pose4);
  group.move(); 

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "adventure_recognition");
  ros::NodeHandle n;
  AdventureRecognition advrec(n);
  // It doesn't get called from here
  /*if (advrec.bottle_is_near)
  {
    advrec.attacknow();
    advrec.bottle_is_near = false;
  }*/
  
  ros::spin();
  return 0;
}
