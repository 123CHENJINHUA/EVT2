#include<iostream>
#include"Class_launch.h"
#include<opencv2/opencv.hpp>
using namespace std;
using namespace cv;

char b_camera = '0';
char light_state = '0';
char start_state = '0';
double angle = 0;

int main()
{
	Class_launch start;
	thread plc_contact(&Class_launch::plc_contact, start, std::ref(b_camera), std::ref(light_state), std::ref(start_state));
	thread line_detector(&Class_launch::line_detector, start, std::ref(b_camera), std::ref(light_state), std::ref(angle));
	//thread controller(&Class_launch::controller_Kinco, start, std::ref(angle));
	thread controller(&Class_launch::controller_Benmo, start, std::ref(angle), std::ref(start_state));
	plc_contact.join();
	line_detector.join();
	controller.join();
	return 0;
}