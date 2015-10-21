#include "VisualSLAM.h"
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

VisualSLAM::VisualSLAM()
{

}

VisualSLAM::~VisualSLAM()
{
    //dtor
}

void VisualSLAM::vSlamInitialization()
{
    Features f;
    f.updataframe();
}

void VisualSLAM::descriptorMatching()
{
    //Matching descriptors
    /*The following types of descriptor matchers are supported:
          BruteForce (it uses L2)
          BruteForce -- L1
          BruteForce -- Hamming
          BruteForce -- Hamming(2)
          FlannBased
    */

     descriptorMatcher = DescriptorMatcher::create("FlannBased");

     descriptorMatcher->match(previous_descriptors, descriptors, matches);

     double max_dist = 0; double min_dist =100;

    for(int i=0; i<descriptors.rows; i++)
    {
        double dist=matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    //Draw only "good matches" (i.e. whose distnace is less than 2*min_dist)

    for(int i=0; i<descriptors.rows;i++)
    {
        if(matches[i].distance < max(2 * min_dist, 0.1))
        {
        good_matches.push_back(matches[i]);
        }
    }

	//drawMatches(rgb, keypoints, previous_rgb, previous_keypoints,good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);//-- Draw matches
	drawMatches(previous_rgb, previous_keypoints, rgb, keypoints, matches, img_matches);                                                                                              //draw all the matches
	imshow("Good Matches", img_matches);//-- Show detected matches
  // drawMatches(previous_rgb, previous_keypoints, rgb, keypoints, matches, img_matches);

   //namedWindow( "Good Matches", WINDOW_NORMAL );

}

void VisualSLAM::drawAndShowMatches()
{
    //Quick Calculation of max and min distances between keypoints

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
        if(matches[i].distance < max(2 * min_dist, 0.02))
        {
        good_matches.push_back(matches[i]);
        }
    }

	drawMatches(previous_rgb, previous_keypoints,rgb, keypoints, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);//-- Draw matches
	//drawMatches(pr_rgb, pr_keypoints, rgb, keypoints, matches, img_matches);                                                                                              //draw all the matches
	imshow("Good Matches", img_matches);//-- Show detected matches
  // drawMatches(previous_rgb, previous_keypoints, rgb, keypoints, matches, img_matches);

   //namedWindow( "Good Matches", WINDOW_NORMAL );

}

void VisualSLAM::VisualSLAMTesting()
{

     vSlamInitialization();
     while(true)
     {
     copyOldDataFrame();
     updataframe();
     descriptorMatching();
     drawAndShowMatches();
     waitKey(5);
     }

}


/*void VisualSLAM::findHomography(Mat& previousImage, Mat& CurrentImage, int CV_RANSAC)
{
   H = findHomography( previousImage, CurrentImage, CV_RANSAC );
}
*/
