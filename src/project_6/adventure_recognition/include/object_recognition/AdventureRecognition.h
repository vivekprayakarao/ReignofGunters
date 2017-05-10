#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/Twist.h"

#include "object_recognition/ObjectRecognition.h"

static const float bottle_centre_tol = 20; //pixels
static const float bounding_box_size_thresh = 300000;
class AdventureRecognition {
private:
ros::NodeHandle nh_;
image_transport::ImageTransport it_;
image_transport::Subscriber image_sub_;
ObjectRecognition objrec;
cv_bridge::CvImagePtr cv_ptr;
tf::TransformListener tf_listener_odom_footprint;
ros::Publisher vel_pub;
bool process_image;
bool bottle_positioned_correctly;
bool attack_now;
float init_error;

public:
AdventureRecognition(ros::NodeHandle n_);
void imageCb(const sensor_msgs::ImageConstPtr& msg);

};
