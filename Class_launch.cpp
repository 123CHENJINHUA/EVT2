#include "Class_launch.h"
#include"Plc_contact.h"
#include"Line_detector.h"
#include"Controller_Kinco.h"
#include"Controller_Benmo.h"
#include<iostream>
using namespace std;

Class_launch::Class_launch()
{

}

Class_launch::Class_launch(Class_launch& c)
{

}

Class_launch::~Class_launch()
{

}

void Class_launch::plc_contact(char& b_camera,char& light_state, char& b_start)
{
	cout << "plc_connecting..." << endl;
	Plc_contact plc(plc_addr,plc_port,b_camera,light_state,b_start);
	plc.main_process();
}

void Class_launch::line_detector(char& b_camera, char& light_state, double& angle)
{
	cout << "Line_detector connecting..."<<endl;
	Line_detector detector(b_camera, light_state, angle);
	detector.main_process();
}

void Class_launch::controller_Kinco(double& angle, char& b_start)
{
	cout << "Controller_Kinco connecting..." << endl;
	Controller_Kinco controller("1", 38400);

	controller.Mode(1, 1);
	controller.Enable(1);
	controller.Speed(1, 100);
	controller.Position(1, 0);
	controller.Start(1);
	Sleep(1000);
	controller.Continuous_operation(1);

	while (1)
	{
		{
			std::lock_guard<std::mutex> lockGuard(my_mutex);
			controller.angle_data = angle;
		}
		controller.main_process();
	}

	//controller.Stop(1);
}

void Class_launch::controller_Benmo(double& angle, char& b_start)
{
	cout << "Controller_Benmo connecting..." << endl;
	Controller_Benmo controller;

	controller.Controller_Initial();
	Sleep(1000);

	while (1)
	{
		{
			std::lock_guard<std::mutex> lockGuard(my_mutex3);
			Is_start = b_start;
		}

		if (Is_start == '0') 
		{ 
			controller.Disable(); 
			continue; 
		}
		else
		{
			if(!Is_enable) 	controller.Enable();
			{
				std::lock_guard<std::mutex> lockGuard(my_mutex);
				controller.angle_data = angle;
			}
			controller.main_process();
		}
	}

	//controller.Stop(1);
}
