#include "object_recognition/AdventureRecognition.h"

AdventureRecognition::AdventureRecognition(ros::NodeHandle n_)
    : it_(n_)
{
    this->nh_ = n_;
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
      &AdventureRecognition::imageCb, this);
    // image_pub_ = it_.advertise("/image_converter/output_video", 1);
    process_image = true; //For now
    bottle_positioned_correctly = false;
    attack_now = false;
    init_error = 1;

    try {
         this->tf_listener_odom_footprint.waitForTransform( "/odom","/base_footprint",ros::Time(0), ros::Duration(20.0) );
     }
  catch (tf::TransformException &ex) {
            ROS_ERROR("[adventure_slam]: (wait) %s", ex.what());
            ros::Duration(1.0).sleep();
  }

  this->vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 5);
}

void AdventureRecognition::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    //cv_bridge::CvImagePtr cv_ptr;
   if (process_image) //Should be activated after a particular waypoint is attained
    {
       geometry_msgs::Twist twist_msg;
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

    /*// Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));*/
   
       std::vector<float> bbox_centroid;
       if (objrec.findMatchingFeatures(cv_ptr->image, bbox_centroid))
       {
         //Check whether aspect ratio changed a lot, skip this frame

         // Update GUI Window
          cv::imshow(OPENCV_WINDOW, objrec.img_matches);
          cv::waitKey(3);
          //Code for checking Bottle Positioned Correctly

           if(bbox_centroid[0] - (cv_ptr->image.cols) / 2 < 0)
		init_error = 1;
           else
                init_error = -1;

           //ROS_INFO("The error is %f", init_err);
 
           twist_msg.linear.x = 0.1;
      	   twist_msg.angular.z = 0;
           vel_pub.publish(twist_msg);

          /*if (err > 50 && !bottle_positioned_correctly)
	  {
             while (ros::duration)
             twist_msg.linear.x = 0;
      	     twist_msg.angular.z = kw * err;
             vel_pub.publish(twist_msg);
          }
          else 
          {
             bottle_positioned_correctly = true;
             twist_msg.linear.x = 0.1;
      	     twist_msg.angular.z = 0;
             vel_pub.publish(twist_msg);            
          }*/

       }
       else 
       {
          //Sign of velocity should change
          twist_msg.linear.x = 0;
      	  twist_msg.angular.z = 0.1 * init_error;
          vel_pub.publish(twist_msg);
       }
    }
       //else show a blank screen - "Not Processing at the moment sort of message"

    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "adventure_recognition");
  ros::NodeHandle n;
  AdventureRecognition advrec(n);
  ros::spin();
  return 0;
}
