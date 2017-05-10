
#include "object_recognition/ObjectRecognition.h"

ObjectRecognition::ObjectRecognition(ros::NodeHandle n_)
    : it_(n_)
{
    this->nh_ = n_;
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1,
      &ObjectRecognition::imageCb, this);
    // image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
    setObjectKeyPoints();
    processImage = false;
}

ObjectRecognition::~ObjectRecognition()
{
   cv::destroyWindow(OPENCV_WINDOW);
}

void ObjectRecognition::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
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
   
    if (processImage)
    {
       std::vector<float> bbox_centroid;
       if (findMatchingFeatures(cv_ptr->image, bbox_centroid))
       {
         // Update GUI Window
          cv::imshow(OPENCV_WINDOW, img_matches);
          cv::waitKey(3);
       }
       //else show a blank screen?
    }

    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
}

void ObjectRecognition::setObjectKeyPoints()
 {
    std::string obj_path = (ros::package::getPath("adventure_recognition") + "/images/train/object.jpg");
    img_object = imread( obj_path, CV_LOAD_IMAGE_GRAYSCALE );
    detector.detect( img_object, keypoints_object );
    extractor.compute( img_object, keypoints_object, descriptors_object );
 }

std::vector<Point2f> ObjectRecognition::getBBox(const Mat & img_scene, const std::vector<KeyPoint> & keypoints_scene, const std::vector<DMatch> & good_matches, bool showMatches)
{
    (this->img_matches) = Mat();
    //-- Localize the object
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;
    drawMatches(img_object, keypoints_object, img_scene, keypoints_scene, good_matches, (this->img_matches), Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    for(size_t i = 0;i < good_matches.size();i++){
	//-- Get the keypoints from the good matches
	obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
	scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
    }

    Mat H = findHomography( obj, scene, CV_RANSAC, 3);
    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0, 0);
    obj_corners[1] = cvPoint(img_object.cols, 0);
    obj_corners[2] = cvPoint(img_object.cols, img_object.rows);
    obj_corners[3] = cvPoint(0, img_object.rows);
    std::vector<Point2f> scene_corners(4);
    perspectiveTransform(obj_corners, scene_corners, H);

    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    line(img_matches, scene_corners[0] + Point2f(img_object.cols, 0), scene_corners[1] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
    line(img_matches, scene_corners[1] + Point2f(img_object.cols, 0), scene_corners[2] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
    line(img_matches, scene_corners[2] + Point2f(img_object.cols, 0), scene_corners[3] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
    line(img_matches, scene_corners[3] + Point2f(img_object.cols, 0), scene_corners[0] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);

    //-- Show detected matches
    if(showMatches)
    	imshow("Good Matches & Object detection", img_matches);

    return scene_corners;
}

 bool ObjectRecognition::findMatchingFeatures(Mat img_scene, std::vector<float>& bbox_centroid) //TODO: Make a struct
{
  cv::initModule_nonfree();

  //-- Step 1: Detect the keypoints in the scene using SIFT Detector

  std::vector<KeyPoint> keypoints_scene;

  detector.detect( img_scene, keypoints_scene );

  //-- Step 2: Calculate descriptors (feature vectors)

  Mat descriptors_scene;
  extractor.compute( img_scene, keypoints_scene, descriptors_scene );

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector< vector<DMatch> > matches;
  matcher.knnMatch(descriptors_object,descriptors_scene, matches,2);

  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
  std::vector< DMatch > good_matches;

  for(size_t i = 0; i < matches.size(); i++)
  {
    if (matches[i].size() == 2 && (matches[i][0].distance < 0.7 * matches[i][1].distance))
        good_matches.push_back(matches[i][0]);

  }
 
  if (good_matches.size() > MIN_MATCH_COUNT)
  {
	  std::vector<Point2f> bbox_corners = getBBox(img_scene, keypoints_scene, good_matches );
	  bbox_centroid.push_back((bbox_corners[0].x + bbox_corners[2].x) / 2);
	  bbox_centroid.push_back((bbox_corners[0].y + bbox_corners[2].y) / 2);
          return true;
  }
  else
  {
        return false;
  }
 }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_recognition");
  ros::NodeHandle n;
  ObjectRecognition objrec(n);
  ros::spin();
  return 0;
}

