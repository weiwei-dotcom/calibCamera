#pragma once
#include<opencv2/opencv.hpp>
#include<fstream>

using namespace cv;
using namespace std;

class cameraCalibrate {
public:
	Mat Intrinsic;
	cameraCalibrate();
	bool getIntrinsic(string cameraType);
};