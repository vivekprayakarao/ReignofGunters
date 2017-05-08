#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

using namespace cv;

void readme();

/** @function main */
int main( int argc, char** argv )
{
  if( argc != 3 )
  { readme(); return -1; }

  Mat img_object = imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE );
  Mat img_scene = imread( argv[2], CV_LOAD_IMAGE_GRAYSCALE );

  if( !img_object.data || !img_scene.data )
  { std::cout<< " --(!) Error reading images " << std::endl; return -1; }

  //-- Step 1: Detect the keypoints using SIFT Detector

  SiftFeatureDetector detector;

  std::vector<KeyPoint> keypoints_object, keypoints_scene;

  detector.detect( img_object, keypoints_object );
  detector.detect( img_scene, keypoints_scene );

  //-- Step 2: Calculate descriptors (feature vectors)
  SiftDescriptorExtractor extractor;

  Mat descriptors_object, descriptors_scene;

  extractor.compute( img_object, keypoints_object, descriptors_object );
  extractor.compute( img_scene, keypoints_scene, descriptors_scene );

  //-- Step 3: Matching descriptor vectors using FLANN matcher
	std::vector<std::vector<cv::DMatch> > matches;
	cv::FlannBasedMatcher matcher;
	matcher.knnMatch(descriptors_object, descriptors_scene, matches, 2);

	vector<cv::DMatch> good_matches;
	for (int i = 0; i < matches.size(); ++i)
	{
    const float ratio = 0.7; // As in Lowe's paper; can be tuned
    if (matches[i][0].distance < ratio * matches[i][1].distance)
    {
        good_matches.push_back(matches[i][0]);
    }
}

  Mat img_matches;
  drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  //-- Localize the object
  std::vector<Point2f> obj;
  std::vector<Point2f> scene;

  for( int i = 0; i < good_matches.size(); i++ )
  {
    //-- Get the keypoints from the good matches
    obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
  }

  Mat H = findHomography( obj, scene, CV_RANSAC );

  //-- Get the corners from the image_1 ( the object to be "detected" )
  std::vector<Point2f> obj_corners(4);
  obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
  obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
  std::vector<Point2f> scene_corners(4);

  perspectiveTransform( obj_corners, scene_corners, H);

  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
  line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
  line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
  line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
  line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );

  //-- Show detected matches
  imshow( "Good Matches & Object detection", img_matches );

  waitKey(0);
  return 0;
  }

  /** @function readme */
  void readme()
  { std::cout << " Usage: ./SIFT_descriptor <img1> <img2>" << std::endl; }
