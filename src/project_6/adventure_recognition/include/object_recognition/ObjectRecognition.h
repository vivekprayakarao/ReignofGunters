#include <ros/package.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;
static const std::string OPENCV_WINDOW = "Matches found";
size_t MIN_MATCH_COUNT = 10;

class ObjectRecognition {
private:
//ros::NodeHandle nh_;
//image_transport::ImageTransport it_;
//image_transport::Subscriber image_sub_;

//Feature detection params, right now focusing on a single object - this should later become arrays
SiftFeatureDetector detector;
std::vector<KeyPoint> keypoints_object;
Mat descriptors_object;
SiftDescriptorExtractor extractor;
Mat img_object;
//image_transport::Publisher image_pub_;

void setObjectKeyPoints();
std::vector<Point2f> getBBox(const Mat & img_scene, const std::vector<KeyPoint> & keypoints_scene, const std::vector<DMatch> & good_matches, bool showMatches = false);

public:
ObjectRecognition();
~ObjectRecognition();
//void imageCb(const sensor_msgs::ImageConstPtr& msg);
bool findMatchingFeatures(Mat img_scene, std::vector<float>& bbox_centroid);
Mat img_matches;
bool processImage;
};
