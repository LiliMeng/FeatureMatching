#include "ReadRGBD.h"

ReadRGBD::ReadRGBD(string location)
{
   dataset_dir = location+"/";
   get_filelist("rgb");
   get_filelist("depth");
   index = 0;
}

ReadRGBD::~ReadRGBD()
{
    //dtor
}

bool ReadRGBD::capture(Mat& rgb_frame, Mat& depth_frame, double& timestamp)
{
    if (index >=RGB_timestamp.size())
	{
		return false;
	}
    timestamp = RGB_timestamp[index];
    rgb_frame = imread(RGB_filelist[index], 1);
    depth_frame = imread(D_filelist[index], CV_LOAD_IMAGE_ANYDEPTH);
    index++;
    return true;
}

void ReadRGBD::get_filelist(string filename)
{
	string location;
	string datafile_name;
	datafile_name = dataset_dir + filename + ".txt";

	double a;
	string b;

	fstream fin;
	fin.open(datafile_name);

	if(!fin)
	{
		std::cout<<"Error! Cannot open file!"<<endl;
	}



	if(filename=="rgb")
	{
		while(fin>>a)
		{

			RGB_timestamp.push_back(a);
			fin>>b;
			RGB_filelist.push_back(dataset_dir+b);
		  }
	}

	if(filename=="depth")
	{
		while(fin>>a)
		{
			D_timestamp.push_back(a);
			fin>>b;
			D_filelist.push_back(dataset_dir+b);
		}
	fin.close();
   }
}
