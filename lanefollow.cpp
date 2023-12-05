#include "lanefollow.hpp"
namespace hsc {
	void ROI_SET(Mat& f, Mat& d, Mat& r) {
		d = f(Rect(0, 270, 640, 90));
		cvtColor(d, d, COLOR_BGR2GRAY); 
		//bilateralFilter(d, d, -1, 10, 5); 
		r = d + (100 - mean(d)[0]);  
		threshold(r, r, 145, 255, THRESH_BINARY); 
	}
	Mat L_ROI(Mat& r) {
		Mat d;
		d = r(Rect(0, 0, r.cols/2, r.rows));
		return d;
	}
	Mat R_ROI(Mat& r) {
		Mat d;
		d = r(Rect(r.cols/2, 0, r.cols/2,r.rows));
		return d;
	}
	// void Lab(Mat& r, Mat& l,  Mat& s, Mat& c, int cnt ){
	// 	cnt = connectedComponentsWithStats(r, l, s, c);
	// 	cvtColor(r, r, COLOR_GRAY2BGR);
	// }
	void Center_Gravity(Mat& r, Mat& c, vector<Point2d>& pts, bool& Center, int cnt) {
		if (cnt >= 2) {
			if (Center) { pts.push_back(Point2d(r.cols / 2.0, r.rows / 2.0)); Center = false; }

			double min = 0;
			Point2d other(c.at<double>(1, 0), c.at<double>(1, 1));
			min = norm(pts.at(0) - other);
			for (int i = 2; i < cnt; i++) {
				double current_norm = norm(pts.at(0) - Point2d(c.at<double>(i, 0), c.at<double>(i, 1)));
				if (min > current_norm) {
					min = current_norm;
					other = Point2d(c.at<double>(i, 0), c.at<double>(i, 1));
				}
			}
			if (!((abs(pts.at(0).x - other.x) > 120 || abs(pts.at(0).y - other.y) > 40))) {
				pts.insert(pts.begin(), 1, other);
			}
		}
		circle(r, pts.at(0), 1, Scalar(0, 0, 255), 2);
	}
	

	// void Lane_Appear(Mat& r, vector<Point2d>& lpts, vector<Point2d>& rpts) {
	// 	double right_Cx = rpts.at(0).x + r.cols / 2;
	// 	double right_Cy = rpts.at(0).y;

	// 	Point2d RightPts(right_Cx, right_Cy);
	// 	circle(r, lpts.at(0), 1, Scalar(0, 0, 255), 2);
	// 	circle(r, RightPts, 1, Scalar(0, 0, 255), 2);
	// }

	void Lane_Center_Gravity(Mat& r, vector<Point2d>& cen, vector<Point2d>& lpts, vector<Point2d>& rpts){
		Point2d center((lpts.at(0).x+(rpts.at(0).x + 320))/2, (rpts.at(0).y+rpts.at(0).y)/2);
		cen.insert(cen.begin(), 1, center);
	}
	int Get_Error(Mat& r, vector<Point2d>& pts) {
		int error = 0;
		error = (r.cols / 2) - pts.at(0).x;
		return error;
	}
}

