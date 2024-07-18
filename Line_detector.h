#include<iostream>
#include<opencv2/opencv.hpp>
#include<Windows.h>
using namespace std;

extern std::mutex my_mutex;
extern std::mutex my_mutex2;
extern std::mutex my_mutex3;

class Line_detector
{
private:
	char camera_state = '0';


	cv::VideoCapture capture1;
	cv::VideoCapture capture2;

	int k_size = 27;//27  //47
	int canny_low_threshold = 10;  //12    //14  //10
	int canny_high_threshold = 20;  //26   //28  //20
	int h_min = 50;
	int h_gap = 500;
	int thres = 1;

	float k_pre = 0;
	int b_pre = 0;
	float k_cur = 0;
	int b_cur = 0;

	bool Is_find = false;
	float line_angle_min = 0;
	int line_b_min = 0;
	float line_angle_max = 0.5; //1
	int line_b_max = 15; //40

	int roi_limit = 0;
	int min_ = 0, max_ = 0;
	int left_edge = 0;//30
	float initial_scale = 0.15;
	float scale = initial_scale;

	bool first_check = true;
	bool Is_longline = false;

	//PID linespeed = 0.05 
	float kp = 4, ki = 0.010, kd = 0; //0.45 //ki=0    // 3m/s kp = 0.2, ki = 0.0010, kd = 0;  //4m/s kp = 0.8, ki = 0.0010, kd = 0
	float tp = 0.0400, ti = 0.001, td = 0;  //3m/s tp = 0.000400, ti = 0.00001, td = 0;  //4m/s   tp = 0.000450, ti = 0.00001, td = 0

	//First check

	bool In_line = false;
	float Find_k_max = 1; //0.5
	int Find_b_max = 30;//20
	int light_touch_num = 0;

	int b_trans = 0;//change the initial position of b;

public:
	char* b_camera = NULL;
	char* light_state = NULL;
	double* angle = NULL;

	Line_detector(char &b_camera, char& light_state, double& angle);
	virtual ~Line_detector() {};

	int main_process();

	bool image_processing(cv::Mat imgOri);

	vector<vector<int>> mad(vector<cv::Vec4i> vec, float s);
	vector<cv::Point> mad2(vector<cv::Point> vec, float s);


	void LSM(vector<vector<int>>& res);

	void enhance_line(const cv::Mat& img_color, vector<vector<cv::Point>>& select_contours);

	bool extract_single_line(cv::Mat& imgOri, cv::Mat& imgCan);
	bool extract_single_line2(cv::Mat& imgOri, cv::Mat& imgCan);

	void vibrate_suppression();

	void select_roi(const cv::Mat& img_color, cv::Mat& img_roi, float k, int b, float scale = 0.2);

	bool line_filter(const cv::Mat& img_color);

	void draw_HoughLinesP(cv::Mat& imgHoughP, vector<cv::Vec4i>& Plines);
	void draw_HoughLinesP(cv::Mat& imgHoughP, vector<vector<int>>& Plines);

	void init_parama();
};

