#include "Features.h"
#include "ReadRGBD.h"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <iostream>

using namespace cv;
using namespace std;

Features::Features():ReadRGBD("/home/lili/workspace/rgbd_dataset_freiburg1_desk")
{
    /* The following detector types are supported:
         "FAST" --FastFeatureDetector
         "STAR" --StarFeatureDetector
         "SIFT" --SIFT(nonfree module)
         "SURF" --SURF(nonfree module)
         "ORB" -- ORB
         "BRISK" -- BRISK
         "MSER" -- MSER
         "GFTT" -- GoodFeaturesToTrackDetector
         "HARRIS" --GoodFeaturesToTrackDetecor with Harris detector enabled
         "Dense" --DenseFeatureDetector
         "SimpleBlob" -- SimpleBlobDetector
         */

         /*The following types of descriptor extractors are supported:
         "SIFT" --SIFT
         "SURF" --SURF
         "BRIEF" -- BriefDescriptorExtractor
         "ORB" --ORB
         "FREAK" --FREAK
         */

     cv::initModule_nonfree();   // only needed when using surf or sift

     //--Step1: Detect the keypoints
     detector = FeatureDetector::create("SIFT");

     //--Step2: Calculate descriptors
     extractor = DescriptorExtractor::create("SIFT");

        }

void Features::copyOldDataFrame()
{
    previous_rgb=rgb.clone();
    previous_depth=depth.clone();
    previous_timestamp=timestamp;
    previous_keypoints=keypoints;
    previous_descriptors=descriptors.clone();

	return;
}


void Features::project_feature_points()
{
    feature_point3D.clear();
    for (int i=0; i<keypoints.size(); i++)
    {
        auto depthValue = depth.at<double>(keypoints[i].pt.y, keypoints[i].pt.x);
        if(depthValue > min_dis && depthValue < max_dis )
        {
             Z = depthValue/factor;
             X = (keypoints[i].pt.x-cx)*Z/fx;
             Y = (keypoints[i].pt.y-cy)*Z/fy;
             feature_point3D.push_back(Point3d(X,Y,Z));
        }
    }
}


void Features::updataframe()
{
    if(capture(rgb, depth, timestamp)==false)
    {
        cout<<"cannot capture data frame!!! Wait a second"<<endl;
        waitKey(1000);
    }

    detector->detect(rgb,keypoints);
    extractor->compute(rgb, keypoints, descriptors);
    project_feature_points();
}



void Features::drawAndShowFeatures()
{
  Mat img_keypoints_1, img_keypoints_2;
  drawKeypoints( previous_rgb, previous_keypoints, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
  drawKeypoints( rgb, keypoints, img_keypoints_2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

  namedWindow( "Previous_Feature_Keypoints", WINDOW_NORMAL );
  imshow("Previous_Feature_Keypoints", img_keypoints_1);
  namedWindow( "Current_Feature_Keypoints", WINDOW_NORMAL );
  imshow("Current_Feature_Keypoints", img_keypoints_2);
  waitKey(5);
}


void Features::descriptorMatching()
{
    /*
    //Matching descriptors
    The following types of descriptor matchers are supported:
          BruteForce (it uses L2)
          BruteForce -- L1
          BruteForce -- Hamming
          BruteForce -- Hamming(2)
          FlannBased
    */
     matcher = DescriptorMatcher::create("FlannBased");
     matcher->match(previous_descriptors, descriptors, matches);
     good_matches.clear();

}

void Features::FeaturesTesting()
{
  updataframe();

  while(capture(rgb,depth,timestamp)==true)
  {
    copyOldDataFrame();
    updataframe();
    drawAndShowFeatures();
  }

  return ;
}

void Features::drawGoodMatches()
{

    double max_dist = 0; double min_dist =100;

    for(int i=0; i<previous_descriptors.rows; i++)
    {
        double dist=matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    //Draw only "good matches" (i.e. whose distnace is less than 2*min_dist)

    for(int i=0; i<previous_descriptors.rows;i++)
    {
        if(matches[i].distance <= max(2 * min_dist, 0.02))
        {
        good_matches.push_back(matches[i]);
        }
    }

   Mat img_matches;
   drawMatches( previous_rgb, previous_keypoints, rgb, keypoints,good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
   //drawMatches(previous_rgb, previous_keypoints, rgb, keypoints, matches, img_matches);

   //namedWindow( "Good Matches", WINDOW_NORMAL );
   imshow("Good Matches", img_matches);
}

void Features::descriptorMatchingTesting()
{
    updataframe();
    while(capture(rgb, depth, timestamp)==true)
    {
       copyOldDataFrame();
       updataframe();
       descriptorMatching();
       drawGoodMatches();
       waitKey(5);
    }

}

void Features::find_Homography()
{
    int minFeatures = 10;
    vector<Point2f> previous;
    vector<Point2f> current;

    if(good_matches.size()>=minFeatures)
    {
        for(int i=0; i<good_matches.size();i++)
        {
            previous.push_back(previous_keypoints[good_matches[i].queryIdx].pt);
            current.push_back(keypoints[good_matches[i].trainIdx].pt);
        }

        H = findHomography(previous, current, CV_RANSAC).clone();
    }
    else
    {
        cout<<"Not enough valid features" <<endl;

        //--look for the keypoints from all the matches rather than just good matches
        for(int i=0; i<matches.size(); i++)
        {
            previous.push_back(previous_keypoints[matches[i].queryIdx].pt);
            current.push_back(keypoints[matches[i].trainIdx].pt);
        }

        H = findHomography(previous, current, CV_RANSAC).clone();
    }

}

void Features::depthToW( Mat& rgb, Mat& depth ){

   for(int i = 0; i < rgb.rows; i++)
   {

      for(int j=0; j<rgb.cols; j++)
      {

        auto depthValuePlainPoint = depth.at<double>(j,i);
        plainZ = depthValuePlainPoint/factor;
        plainX = (rgbx-cx)*plainZ/fx;
        plainY= (rgby-cy)*plainZ/fy;
        plain_point3D.push_back(Point3d(plainX,plainY,plainZ));

      }
    }

}

 Vec2i Features::wToRGB() {

    rgbpoint[0] = (int) round( (plainX * fx/ plainZ) + cx);
    rgbpoint[1] = (int) round( (plainY * fy/ plainZ) + cy);

    return rgbpoint;
}



void Features::rgbdmap() {

    updataframe();
    while(capture(rgb, depth, timestamp)==true)
    {
       copyOldDataFrame();
       updataframe();
       Mat color = Mat( depth.size(), CV_8UC3, Scalar(0) );
       for( int y = 0; y < depth.rows; y++ )
       {
          ushort* raw_image_ptr = depth.ptr<ushort>( y );

          for( int x = 0; x < depth.cols; x++ )
          {
                if( raw_image_ptr[x] >= 2047 || raw_image_ptr[x] <= 0 )
                continue;
                depthToW(rgb,depth);
                rgbpoint = wToRGB();
                color.at<Vec3b>(y, x) = rgb.at<Vec3b>(rgbpoint[0], rgbpoint[1]);
            }
    }
    imshow("window", color);
    waitKey(5);
    }

}
