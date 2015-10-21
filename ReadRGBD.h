#ifndef READRGBD_H
#define READRGBD_H

#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>  //read data
#include <opencv2/core/core.hpp>   //Basic OpenCV structures (cv::Mat, Scalar)

using namespace std;
using namespace cv;

class ReadRGBD
{
	public:
	ReadRGBD(string location);
	~ReadRGBD();
	bool capture(Mat& rgb_frame, Mat& depth_frame, double& timestamp);


	int index;

	string dataset_dir;

     vector<string> RGB_filelist;
     vector<double> RGB_timestamp;
     vector<string> D_filelist;
     vector<double> D_timestamp;

     void get_filelist(string filename);


};



#endif // READRGBD_H
