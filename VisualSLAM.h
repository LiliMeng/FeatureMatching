#ifndef VISUALSLAM_H
#define VISUALSLAM_H
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "ReadRGBD.h"
#include "Features.h"
using namespace cv;
using namespace std;

class VisualSLAM:private Features
{
    public:
        VisualSLAM();
        void vSlamInitialization();
        void descriptorMatching();
        void drawAndShowMatches();
        void VisualSLAMTesting();

        //void findHomography(Mat&, Mat&, int);
        virtual ~VisualSLAM();
    protected:
    private:
    double timestamp, previous_timestamp;
    Mat rgb, previous_rgb;
    Mat depth, previous_depth;
    Mat descriptors, previous_descriptors;
    Mat H;
    vector< DMatch > matches;
    vector< DMatch > good_matches;
    Mat img_matches;
    Ptr<DescriptorMatcher> descriptorMatcher;
};

#endif // VISUALSLAM_H
