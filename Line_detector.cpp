#include "Line_detector.h"
#include<iostream>
#include<opencv2/opencv.hpp>
#include <algorithm>
#include"PID.hpp"


using namespace std;

std::mutex my_mutex;
std::mutex my_mutex2;

Line_detector::Line_detector(char& b_camera, char& light_state, double& angle)
{
	this->b_camera = &b_camera;
	this->light_state = &light_state;
	this->angle = &angle;
}



int Line_detector::main_process()
{
	cv::Mat img, img_roi;
	control::PID<>::Param pidParam(kp, ki, kd);
	control::PID<>::Param pidParamTrans(tp, ti, td);
	control::PID<> pidTrans = control::PID<>(pidParamTrans);
	control::PID<> pid = control::PID<>(pidParam);

	
	capture1.open(0);
	//capture1.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
	//capture1.set(CAP_PROP_FPS, 30);
	//capture1.set(CAP_PROP_FRAME_WIDTH, 1280);
	//capture1.set(CAP_PROP_FRAME_HEIGHT, 720);

	capture2.open(1);
	//capture2.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
	//capture2.set(CAP_PROP_FPS, 30);
	//capture2.set(CAP_PROP_FRAME_WIDTH, 1280);
	//capture2.set(CAP_PROP_FRAME_HEIGHT, 720);

	capture1.set(cv::CAP_PROP_EXPOSURE, -4);
	capture2.set(cv::CAP_PROP_EXPOSURE, -7);//-4 //-7

	if (false == (capture1.isOpened() || capture2.isOpened()))
	{
		cout << "cannot open the camera." << endl;
		return -1;
	}
	{
		std::lock_guard<std::mutex> lockGuard(my_mutex2);
		*light_state = 'S';
	}
	Sleep(1000);
	{
		std::lock_guard<std::mutex> lockGuard(my_mutex2);
		*light_state = '0';
	}
	while (1)
	{

		{
			std::lock_guard<std::mutex> lockGuard(my_mutex);
			camera_state = *b_camera;
		}
		
		if (camera_state == 'L') capture1 >> img;
		else if (camera_state == 'R') capture2 >> img;
		else if (camera_state == '0')
		{
			cout << "camera havn't been open." << endl;
			init_parama();
			light_touch_num = 0;
			{
				std::lock_guard<std::mutex> lockGuard(my_mutex2);
				*light_state = '0';
			}
			{
				std::lock_guard<std::mutex> lockGuard(my_mutex);
				*angle = 0;  //*spinSpeed = spinCmd + transCmd;
			}
			Sleep(400);//调试用
			continue;
		}
		else { cout << "error" << endl; continue; }
		if (img.empty())
		{
			capture1.open(0);
			capture2.open(1);
			continue;
		}
		select_roi(img, img_roi, k_cur, b_cur, scale);

		image_processing(img_roi);

		cv::Mat imgHoughPt = img.clone();
		rectangle(imgHoughPt, cv::Rect(left_edge, min_, imgHoughPt.cols - 2 * left_edge, max_ - min_), cv::Scalar(0, 0, 255), 1, 1, 0);
		line(imgHoughPt, cv::Point2i(left_edge, int(k_cur * left_edge + b_cur)), cv::Point2i(img.cols, int(k_cur * img.cols + b_cur)), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
		line(imgHoughPt, cv::Point2i(0, img.rows / 2 + b_trans), cv::Point2i(img.cols, img.rows / 2 + b_trans), cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
		//resize(imgHoughPt, imgHoughPt, Size(imgHoughPt.cols / 4, imgHoughPt.rows / 4), 0, 0, INTER_LINEAR);
		imshow("imgHoughPt", imgHoughPt);

		char c = (char)cv::waitKey(30);
		if (c == 27)
			break;

		if (In_line == false)
		{
			{
				std::lock_guard<std::mutex> lockGuard(my_mutex2);
				*light_state = 'W';
			}
			if (fabs(atan(k_cur) * 180 / CV_PI) > Find_k_max || fabs(int(k_cur * img.cols / 2 + b_cur) - img.rows / 2) > Find_b_max)
			{
				init_parama();
				continue;
			}
			if (light_touch_num > 20)
			{
				b_trans = int(k_cur * img.cols / 2 + b_cur) - img.rows / 2;
				{
					std::lock_guard<std::mutex> lockGuard(my_mutex2);
					*light_state = 'F';
				}
				cout << "found" << endl;
				In_line = true;
			}
			else
			{
				light_touch_num++;
				cout << "touch:" << light_touch_num << endl;
			}

		}

		double spinCmd = -pid(atan(k_cur), 0.0f);
		//double transCmd = pidTrans((int( b_cur) - img.rows / 2), 0.0f);
		double transCmd = pidTrans((int(k_cur * img.cols / 2 + b_cur) - img.rows / 2 - b_trans), 0.0f);

		{
			std::lock_guard<std::mutex> lockGuard(my_mutex);
			*angle = spinCmd + transCmd;  //*spinSpeed = spinCmd + transCmd;
		}
		//cout << *angle << endl;//测试用
	}
	return 0;
}

bool Line_detector::image_processing(cv::Mat imgOri)
{
	cv::Mat imgCan;
	cv::Mat filter2D_img;
	//imshow("imgOri", imgOri);

	GaussianBlur(imgOri, imgOri, cv::Size(k_size, k_size), 0);

	//imshow("gaussin", imgOri);

	Canny(imgOri, imgCan, canny_low_threshold, canny_high_threshold);
	imshow("Canny", imgCan);

	if (first_check)
	{
		Is_longline = line_filter(imgCan);
		first_check = false;
	}

	if (Is_longline)
	{

		extract_single_line(imgOri, imgCan);
		return true;
	}
	vector<vector<cv::Point>> select_contours;
	vector<vector<cv::Point>> select_contours2;
	cv::Mat imageContours = cv::Mat::zeros(imgCan.size(), CV_8UC1);
	cv::Mat imageContours2 = cv::Mat::zeros(imgCan.size(), CV_8UC1);

	enhance_line(imgCan, select_contours);

	//按最小长度筛选
	for (auto p : select_contours)
	{
		cv::RotatedRect rect = minAreaRect(p);
		double loneSide = (rect.size.height >= rect.size.width) ? rect.size.height : rect.size.width;
		double shortSide = (rect.size.height <= rect.size.width) ? rect.size.height : rect.size.width;
		double aspectRatio = shortSide / loneSide;
		//cout << aspectRatio << endl;
		if (shortSide < 30 && rect.size.area() < 3200 && rect.size.area() > 600 && aspectRatio>0.05 && aspectRatio < 0.5)
		{
			// cout << rect.size.area()<< endl;
			select_contours2.push_back(p);
		}
	}


	drawContours(imageContours, select_contours, -1, cv::Scalar(255), -1);
	imshow("Contours Image", imageContours);

	//drawContours(imageContours2, select_contours2, -1, Scalar(255), -1);
	//Mat kernel = getStructuringElement(MORPH_RECT, Size(27, 27), Point(-1, -1));  //开运算kernel
	//morphologyEx(imageContours2, imageContours2, MORPH_CLOSE, kernel, Point(-1, -1), 3);//闭运算
	//imshow("Contours Image2", imageContours2);

   // extract_single_line(imgOri, imageContours2);
	extract_single_line2(imgOri, imageContours);
	return true;
}

bool cmp(vector<int> a, vector<int> b)
{
	if (a[0] != b[0]) return a[0] > b[0];
	if (a[1] != b[1]) return a[1] > b[1];
	if (a[2] != b[2]) return a[2] > b[2];
	if (a[3] != b[3]) return a[3] > b[3];
}

vector<vector<int>> Line_detector::mad(vector<cv::Vec4i> vec, float s)
{
	int row = vec.size();

	//构建二维数组
	vector<vector<int>> vec2(row);
	for (int i = 0; i < vec2.size(); i++)
		vec2[i].resize(4);


	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			vec2[i][j] = int(vec[i][j]);
		}
	}
	//test
	/*
	for (auto p : vec2)
		cout << p[0] << ' ' << p[1] << ' ' << p[2] << ' ' << p[3] << endl;
	*/

	if (row < 2)
	{
		return vec2;
	}
	std::sort(vec2.begin(), vec2.end(), cmp);

	float median = (row % 2 == 1) ? vec2[row / 2][1] :
		(vec2[row / 2][1] + vec2[row / 2 - 1][1]) / 2;//中位数
	std::vector<float> deviations;     //偏差值
	for (int i = 0; i < row; i++)
	{
		deviations.push_back(abs(vec2[i][1] - median));
	}
	std::sort(deviations.begin(), deviations.end());
	float mad = (row % 2 == 1) ? deviations[row / 2] :
		(deviations[row / 2] + deviations[row / 2 - 1]) / 2;

	vector<vector<int>> res;
	for (int i = 0; i < row; i++)
	{
		vector<int> temp;
		if (abs(vec2[i][1] - median) <= s * mad)
		{
			for (int j = 0; j < 4; ++j) {
				temp.push_back(vec2[i][j]);
			}
		}
		else continue;
		res.push_back(temp);
	}

	//角度筛选
	std::vector<float> angle;  //角度
	for (int k = 0; k < res.size(); k++)
	{
		if ((res[k][2] - res[k][0]) == 0)  continue;
		angle.push_back((res[k][3] - res[k][1]) / (res[k][2] - res[k][0]));
	}
	if (angle.size() < 2)
	{
		return res;
	}
	std::sort(angle.begin(), angle.end());
	float angle_median = (angle.size() % 2 == 1) ? angle[int(angle.size() / 2)] :
		(angle[int(angle.size() / 2)] + angle[int(angle.size() / 2) - 1]) / 2;//角度中位数
	std::vector<float> angle_deviations;     //角度偏差值
	for (int i = 0; i < angle.size(); i++)
	{
		angle_deviations.push_back(abs(angle[i] - angle_median));
	}
	std::sort(angle_deviations.begin(), angle_deviations.end());
	float angle_mad = (angle.size() % 2 == 1) ? angle_deviations[int(angle.size() / 2)] :
		(angle_deviations[int(angle.size() / 2)] + angle_deviations[int(angle.size() / 2) - 1]) / 2;


	vector<vector<int>> res2;
	for (int i = 0; i < angle.size(); i++)
	{
		vector<int> temp;
		if ((res[i][2] - res[i][0]) != 0)
		{
			if (abs(((res[i][3] - res[i][1]) / (res[i][2] - res[i][0])) - angle_median) <= s * mad)
			{
				for (int j = 0; j < 4; ++j) {
					temp.push_back(res[i][j]);
				}
			}
			else continue;
			res2.push_back(temp);
		}

	}

	return res2;
}

bool cmp2(cv::Point a, cv::Point b)
{
	return a.y > b.y;
}

vector<cv::Point> Line_detector::mad2(vector<cv::Point> vec, float s)
{
	int n = vec.size();
	if (n < 2)
	{
		return vec;
	}

	sort(vec.begin(), vec.end(), cmp2);
	float median = (n % 2 == 1) ? vec[n / 2].y :
		(vec[n / 2].y + vec[n / 2 - 1].y) / 2;//中位数
	vector<float> deviations;
	vector<cv::Point> new_nums;     //偏差值
	for (int i = 0; i < n; i++)
	{
		deviations.push_back(abs(vec[i].y - median));
	}
	std::sort(deviations.begin(), deviations.end());
	float mad = (n % 2 == 1) ? deviations[n / 2] :
		(deviations[n / 2] + deviations[n / 2 - 1]) / 2;
	for (size_t i = 0; i < n; i++)
	{
		if (abs(vec[i].y - median) < s * mad)
			new_nums.push_back(vec[i]);
	}
	return new_nums;
}

void Line_detector::LSM(vector<vector<int>>& res)
{
	vector<float> x;
	vector<float> y;
	for (auto& line : res)
	{
		x.push_back(line[0]);
		y.push_back(line[1]);
		x.push_back(line[2]);
		y.push_back(line[3]);
	}
	float t1 = 0, t2 = 0, t3 = 0, t4 = 0;
	for (unsigned int i = 0; i < x.size(); i++)
	{
		t1 += x[i] * x[i];
		t2 += x[i];
		t3 += x[i] * y[i];
		t4 += y[i];
	}
	//限制震荡
	k_cur = (t3 * x.size() - t2 * t4) / (t1 * x.size() - t2 * t2);
	b_cur = (t1 * t4 - t2 * t3) / (t1 * x.size() - t2 * t2) + min_;
}

void Line_detector::enhance_line(const cv::Mat& img_color, vector<vector<cv::Point>>& select_contours)
{
	// Mat kernel = getStructuringElement(MORPH_RECT, Size(7, 7), Point(-1, -1));  //开运算kernel   //
	cv::Mat kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5), cv::Point(-1, -1));  //开运算kernel   //
	morphologyEx(img_color, img_color, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 3);//闭运算
	vector<vector<cv::Point>> contours;
	vector<cv::Vec4i> hierarchy;
	findContours(img_color, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point());

	//去除过小点
	for (int i = 0; i < contours.size(); i++)
	{
		if (contours[i].size() < 50) continue;
		select_contours.push_back(contours[i]);
	}
}

bool Line_detector::extract_single_line(cv::Mat& imgOri, cv::Mat& imgCan)
{
	cv::Mat imgHoughP = imgOri.clone();
	vector<cv::Vec4i> Plines;
	HoughLinesP(imgCan, Plines, 1, CV_PI / 180, 80, h_min, h_gap);

	if (Plines.size() == 0)
	{
		scale += 0.1;
		if (scale >= 0.2)
		{
			scale = 0.2;
		}
		return true;
	}
	else scale = initial_scale;

	//剔除异常点前

	cv::Mat imgHoughPb = imgOri.clone();
	draw_HoughLinesP(imgHoughPb, Plines);

	//剔除异常点后
	vector<vector<int>> res = mad(Plines, thres);   //好像没什么用？
	//vector<Vec4i> res = Plines;

	//最小二乘法前
	draw_HoughLinesP(imgHoughP, res);

	cv::Mat result;
	imgHoughPb.push_back(imgHoughP);
	vconcat(imgHoughPb, result);
	resize(result, result, cv::Size(imgOri.cols * 2, imgOri.rows * 2), 0, 0, cv::INTER_LINEAR);

	imshow("result", result);

	//最小二乘法后
	LSM(res);
	// vibrate_suppression();

	Is_find = true;
}

void Line_detector::vibrate_suppression()
{
	float angle_dif = 0;
	int b_dif = 0;
	angle_dif = (atan(k_cur) - atan(k_pre)) * 180 / CV_PI;
	b_dif = b_cur - b_pre;

	if (Is_find && ((fabs(angle_dif) < line_angle_min) || (fabs(b_dif) < line_b_min) || (fabs(angle_dif) > line_angle_max) || (fabs(b_dif) > line_b_max)))
	{
		k_cur = k_pre;
		b_cur = b_pre;
	}
	k_pre = k_cur;
	b_pre = b_cur;
}

void Line_detector::select_roi(const cv::Mat& img_color, cv::Mat& img_roi, float k, int b, float scale)
{
	int image_width = (img_color.cols - roi_limit);
	int image_height = (img_color.rows - roi_limit);

	/*
	if (b == 0) b = image_height / 2;// +70;
	//else b_scale = b * image_height / (max_ - min_);
	//else b_scale = b + min_;

	if (k > 0)
	{
		min_ = b;
		max_ = image_width * k + b;
	}
	else
	{
		min_ = image_width * k + b;
		max_ = b;
	}
	*/
	min_ = image_height / 2 + b_trans;
	max_ = image_height / 2 + b_trans;

	min_ -= image_height * scale / 2;
	max_ += image_height * scale / 2;
	if (min_ < roi_limit) min_ = roi_limit;
	if (max_ > image_height) max_ = image_height;


	cv::Rect rect_roi(left_edge, min_, image_width - 2 * left_edge, max_ - min_);
	img_roi = img_color(rect_roi);
}

bool Line_detector::line_filter(const cv::Mat& img_color)
{
	vector<vector<cv::Point>> select_contours;
	enhance_line(img_color, select_contours);

	//按最小长度筛选
	for (auto p : select_contours)
	{
		cv::RotatedRect rect = minAreaRect(p);
		double loneSide = (rect.size.height >= rect.size.width) ? rect.size.height : rect.size.width;
		if (loneSide > 800)
		{
			initial_scale = 0.2;
			return true;
		}
	}
	return false;
}

void Line_detector::draw_HoughLinesP(cv::Mat& imgHoughP, vector<cv::Vec4i>& Plines)
{
	for (size_t i = 0; i < Plines.size(); i++)
	{
		line(imgHoughP, cv::Point(Plines[i][0], Plines[i][1]),
			cv::Point(Plines[i][2], Plines[i][3]), cv::Scalar(0, 255, 0), 1, 8);
	}
}

void Line_detector::draw_HoughLinesP(cv::Mat& imgHoughP, vector<vector<int>>& Plines)
{
	for (size_t i = 0; i < Plines.size(); i++)
	{
		line(imgHoughP, cv::Point(Plines[i][0], Plines[i][1]),
			cv::Point(Plines[i][2], Plines[i][3]), cv::Scalar(0, 255, 0), 1, 8);
	}
}

void Line_detector::init_parama()
{
	k_pre = 0;
	b_pre = 0;
	k_cur = 0;
	b_cur = 0;

	Is_find = false;
	line_angle_min = 0.5;
	line_b_min = 10;
	line_angle_max = 10;
	line_b_max = 40;

	roi_limit = 0;
	min_ = 0, max_ = 0;
	left_edge = 0;//30
	initial_scale = 0.15;
	scale = initial_scale;

	first_check = true;
	Is_longline = false;

	In_line = false;
	b_trans = 0;
}

bool Line_detector::extract_single_line2(cv::Mat& imgOri, cv::Mat& imgCan)
{
	vector<cv::Point> idx;
	findNonZero(imgCan, idx);
	if (idx.empty()) return false;

	idx = mad2(idx, 2);
	//cout <<"idx:--------" << idx.size() << endl;
	if (idx.size() < 1000) return false;
	cv::Mat imageContours2 = cv::Mat::zeros(imgCan.size(), CV_8UC1);
	for (int i = 0; i < idx.size(); i++)
	{
		circle(imageContours2, idx[i], 1, (0, 0, 255), 0);
	}
	imshow("Contours Image2", imageContours2);

	/*
	//筛选
	Mat imageContours3 = Mat::zeros(imgCan.size(), CV_8UC1);
	vector<vector<Point>> select_contours3;
	vector<vector<Point>> select_contours4;
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(imageContours2, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point());

	//去除过小点
	for (int i = 0; i < contours.size(); i++)
	{
		if (contours[i].size() < 100) continue;
		select_contours3.push_back(contours[i]);
	}

	for (auto p : select_contours3)
	{
		RotatedRect rect = minAreaRect(p);
		double loneSide = (rect.size.height >= rect.size.width) ? rect.size.height : rect.size.width;
		double shortSide = (rect.size.height <= rect.size.width) ? rect.size.height : rect.size.width;
		double aspectRatio = shortSide / loneSide;
		//cout << aspectRatio << endl;
		//cout << "shortside" << shortSide << endl;
		if (shortSide < 30 && shortSide >15 )
		{
			select_contours4.push_back(p);
		}
	}
	cout << select_contours4.size() << endl;
	if (select_contours4.size() < 3)
	{
		k_cur = 0;
		b_cur = imgCan.rows / 2 + min_;
		return false;
	}
	drawContours(imageContours3, select_contours4, -1, Scalar(255), -1);
	imshow("Contours Image3", imageContours3);
	findNonZero(imageContours3, idx);
	if (idx.empty()) return false;
	//筛选结束

	*/
	cv::Vec4f line_para;
	fitLine(idx, line_para, cv::DIST_L2, 0, 1e-2, 1e-2);

	//获取点斜式的点和斜率
	cv::Point point0;
	point0.x = line_para[2];
	point0.y = line_para[3];
	k_cur = line_para[1] / line_para[0];
	b_cur = k_cur * (0 - point0.x) + point0.y + min_;
	Is_find = true;
	//vibrate_suppression();
   // cout << fabs(atan(k_cur) * 180 / CV_PI) << endl;
	return true;
}