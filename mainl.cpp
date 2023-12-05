#include "lanefollow.hpp"
using namespace hsc;
bool mode = false;
bool ctrl_c_pressed = false;
void ctrlc_handler(int) { ctrl_c_pressed = true; }
int main(void)
{
	// VideoCapture source("lanefollow_100rpm_cw.mp4");
	// if (!source.isOpened()) { cerr << "Video open failed!" << endl; return-1; }

	Dxl mx;
	struct timeval start, end1;
	double diff1;
	int rightvel = 0, leftvel = 0;

	signal(SIGINT, ctrlc_handler); 				
	if (!mx.open()) { cout << "dxl open error" << endl; return -1; } 

	string src = "nvarguscamerasrc sensor-id=0 ! \
	video/x-raw(memory:NVMM), width=(int)640, height=(int)360, \
	format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, \
	width=(int)640, height=(int)360, format=(string)BGRx ! \
	videoconvert ! video/x-raw, format=(string)BGR ! appsink";
	VideoCapture source(src, CAP_GSTREAMER);
	if (!source.isOpened()) { cout << "Camera error" << endl; return -1; }

	string dst1 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
	nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
	h264parse ! rtph264pay pt=96 ! \
	udpsink host=192.168.0.51 port=6100 sync=false";
	// 167, 151
	VideoWriter writer1(dst1, 0, (double)30, Size(640, 360), true);
	if (!writer1.isOpened()) { cerr << "Writer open failed!" << endl; return -1; }

	string dst2 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
	nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
	h264parse ! rtph264pay pt=96 ! \
	udpsink host=192.168.0.51 port=6200 sync=false";
	// 167, 151
	VideoWriter writer2(dst2, 0, (double)30, Size(320, 90), true);
	if (!writer2.isOpened()) { cerr << "Writer open failed!" << endl; return -1; }

	string dst3 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
	nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
	h264parse ! rtph264pay pt=96 ! \
	udpsink host=192.168.0.51 port=6300 sync=false";
	// 167, 151
	VideoWriter writer3(dst3, 0, (double)30, Size(320, 90), true);
	if (!writer3.isOpened()) { cerr << "Writer open failed!" << endl; return -1; }

	vector<Point2d> pts, Lpts, Rpts, cen;
	int cnt, Lcnt, Rcnt, error = 0, lerror = 0, rerror = 0;
	bool Center = true;
	bool LCenter = true;
	bool RCenter = true;
	Mat frame, dst, ROI, LROI, RROI;
	Mat labels, stats, centroids;
	Mat Llabels, Lstats, Lcentroids;
	Mat Rlabels, Rstats, Rcentroids;
	while (true)
	{
		gettimeofday(&start, NULL);
		if (mx.kbhit())
		{
			char ch = mx.getch();
			if (ch == 's') mode = true;
		}

		source >> frame;
		if (frame.empty())break;
		writer1 << frame;

		ROI_SET(frame, dst,ROI);
		LROI = L_ROI(ROI);
		RROI = R_ROI(ROI);

		cnt = connectedComponentsWithStats(ROI, labels, stats, centroids);
	 	cvtColor(ROI, ROI, COLOR_GRAY2BGR);
		Lcnt = connectedComponentsWithStats(LROI, Llabels, Lstats, Lcentroids);
	 	cvtColor(LROI, LROI, COLOR_GRAY2BGR);
		Rcnt = connectedComponentsWithStats(RROI, Rlabels, Rstats, Rcentroids);
		cvtColor(RROI, RROI, COLOR_GRAY2BGR);
		// Lab(ROI, labels, stats, centroids, cnt);
		// Lab(LROI, Llabels, Lstats, Lcentroids, Lcnt);
		// Lab(RROI, Rlabels, Rstats, Rcentroids, Rcnt);

		
		Center_Gravity(LROI, Lcentroids, Lpts, LCenter, Lcnt);
		Center_Gravity(RROI, Rcentroids, Rpts, RCenter, Rcnt);

		Lane_Center_Gravity(ROI, cen, Lpts, Rpts);

		//Lane_Appear(ROI, Lpts, Rpts);

		// lerror = Get_Error(LROI, Lpts);
		// rerror = Get_Error(RROI, Rpts);

		error = Get_Error(ROI, cen);
		//error = lerror + rerror;
		
		writer2 << LROI;
		writer3 << RROI;

		leftvel = RPM - K * error;
		rightvel = -(RPM + K * error);

		cout << "Error:" << error;

		if (ctrl_c_pressed) break;
		if (mode) mx.setVelocity(leftvel, rightvel);

		usleep(5 * 1000);
		gettimeofday(&end1, NULL);
		diff1 = end1.tv_sec + end1.tv_usec / 1000000.0 - start.tv_sec - start.tv_usec / 1000000.0;
		cout << " ,leftvel:" << leftvel << ',' << "rightvel:" << rightvel << ",time:" << diff1 << endl;
		cout << "RPM:"<<RPM <<", K:"<<K<<endl;
		// imshow("frame", frame);
		// imshow("ROI", ROI);
		// imshow("L_ROI", LROI);
		// imshow("R_ROI", RROI);
		// if (waitKey(30) == 27)break;
	}
	mx.close();
	return 0;
}