

#ifndef FEATURES_H
#define FEATURES_H

#include "ReadRGBD.h"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/core/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <iostream>

using namespace cv;
using namespace std;

class Features:ReadRGBD
{
    public:
        Features();
        //virtual ~Features();

    void updataframe();
    void project_feature_points();
    void copyOldDataFrame();
    void descriptorMatching();
    void drawAndShowFeatures();
    void FeaturesTesting();
    void drawGoodMatches();
    void descriptorMatchingTesting();
    void find_Homography();
    void cameraPoseFromHomography();
    Vec2i wToRGB();

    void rgbdmap();
    void  depthToW(Mat& rgb, Mat& depth );



    Mat rgb, previous_rgb;
    Mat depth, previous_depth;
    double timestamp, previous_timestamp;

    Ptr<DescriptorExtractor> extractor;
	Ptr<FeatureDetector> detector;

	vector<KeyPoint> keypoints, previous_keypoints;
	Mat descriptors, previous_descriptors;

    vector<Point3d>  plain_point3D;
    vector<Point3d>  feature_point3D;

	vector< DMatch > matches;
    vector< DMatch > good_matches;

    Ptr<DescriptorMatcher> matcher;

    Mat H;
    int rgbx, rgby;
    Vec2i rgbpoint;

    //camera parameters
    double fx = 525.0; //focal length x
    double fy = 525.0; //focal length y
    double cx = 319.5; //optical centre x
    double cy = 239.5; //optical centre y

    double min_dis = 800;
    double max_dis = 35000;

    double X, Y, Z;
    double plainX, plainY, plainZ;
    double factor = 5000;

    /* factor = 5000 for the 16-bit PNG files
    or factor =1 for the 32-bit float images in the ROS bag files

    for v in range (depth_image.height):
    for u in range (depth_image.width):

    Z = depth_image[v,u]/factor;
    X = (u-cx) * Z / fx;
    Y = (v-cy) * Z / fy;
    */


};

#endif // FEATURES_H
