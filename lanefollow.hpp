#ifndef _LANEFOLLOW_H_
#define _LANEFOLLOW_H_
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include "dxl.hpp"
#define K 2.5
#define RPM 200
using namespace std;
using namespace cv;
namespace hsc {
	void ROI_SET(Mat& f, Mat& d, Mat& r);
	Mat L_ROI(Mat& r);
	Mat R_ROI(Mat& r);
	//void Lab(Mat& r, Mat& l,  Mat& s, Mat& c, int cnt );
	void Center_Gravity(Mat& r, Mat& c, vector<Point2d>& pts, bool& Center, int cnt);
	void Lane_Center_Gravity(Mat& r, vector<Point2d>& cen, vector<Point2d>& lpts, vector<Point2d>& rpts);
	//void Lane_Appear(Mat& r, vector<Point2d>& lpts, vector<Point2d>& rpts);
	int Get_Error(Mat& r, vector<Point2d>& pts);
}
#endif