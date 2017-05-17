
#include "object_recognition/ObjectRecognition.h"
#include <cmath>
#include "ros/ros.h"

ObjectRecognition::ObjectRecognition()
{
    cv::namedWindow(OPENCV_WINDOW);
    setObjectKeyPoints(0);
    //count = 0;
    /*bottle_nums = new char[4];
    bottle_nums[0] = '0';
    bottle_nums[1] = '1';
    bottle_nums[2] = '2';
    bottle_nums[3] = '\0';*/
}

ObjectRecognition::~ObjectRecognition()
{
   cv::destroyWindow(OPENCV_WINDOW);
   //delete obj_paths;
}

void ObjectRecognition::setObjectKeyPoints(int count)
 {
    /*char bottle_nums[] = {'0', '1', '2', '\0'};
    std::stringstream ss;
    std::string s;
    char c = bottle_nums[count];*/
    std::stringstream ss;
    std::string s;
    char c = count + '0';
    ss << c;
    ss >> s;
    //std::string s = std::to_string(count);
    //std::string sub_str = "/images/train/bottle_" + s + ".jpg";
    //std::string obj_path = (ros::package::getPath("adventure_recognition") + sub_str);

    std::string obj_path;
    ROS_INFO("count = %d", count);
    if(count == 0)
      obj_path = (ros::package::getPath("adventure_recognition") + "/images/train/bottle_0.jpg");
    else if(count == 1)
      obj_path = (ros::package::getPath("adventure_recognition") + "/images/train/bottle_1.jpg");
    else
      obj_path = (ros::package::getPath("adventure_recognition") + "/images/train/bottle_2.jpg");

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

 bool ObjectRecognition::findMatchingFeatures(Mat img_scene, std::vector<float>& bbox_centroid, float& diagonal)
{
  //cv::initModule_nonfree();

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
          diagonal = std::sqrt(std::pow((bbox_corners[0].x - bbox_corners[2].x),2) + 
                     std::pow(std::abs(bbox_corners[0].y - bbox_corners[2].y),2));
          
          return true;
  }
  else
  {
        return false;
  }
 }

