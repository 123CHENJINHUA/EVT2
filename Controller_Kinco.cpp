#include "Controller_Kinco.h"
#include<iostream>
using namespace std;

bool Controller_Kinco::Mode(int num, int mode)
{
	this->mode[0] = num;
	this->mode[5] = mode;
	lrc(this->mode, 10);
	WriteChar(this->mode, 10);
	return true;
}
bool Controller_Kinco::Enable(int num)
{
	this->enable[0] = num;
	lrc(this->enable, 10);
	WriteChar(this->enable, 10);
	return true;
}
bool Controller_Kinco::Position(int num, double position)
{
	this->position[0] = num;

	long positon_trans;
	int low, high;

	positon_trans = long(position);
	low = positon_trans % (256 * 256);
	high = positon_trans / (256 * 256);

	this->position[5] = low % 256;
	this->position[6] = low / 256;
	this->position[7] = high % 256;
	this->position[8] = high / 256;

	if (position < 0)
	{

		this->position[5] -= 1;
		this->position[6] -= 1;
		this->position[7] -= 1;
		this->position[8] -= 1;
	}

	lrc(this->position, 10);
	WriteChar(this->position, 10);
	return true;
}
bool Controller_Kinco::Speed(int num, float speed)
{
	this->speed[0] = num;

	long speed_trans;
	int low, high;

	speed_trans = (time_t)speed * 512 * 65536 / 1875;
	low = speed_trans % (256 * 256);
	high = speed_trans / (256 * 256);

	this->speed[5] = low % 256;
	this->speed[6] = low / 256;
	this->speed[7] = high % 256;
	this->speed[8] = high / 256;

	if (speed < 0)
	{

		this->speed[5] -= 1;
		this->speed[6] -= 1;
		this->speed[7] -= 1;
		this->speed[8] -= 1;
	}

	lrc(this->speed, 10);
	WriteChar(this->speed, 10);
	return true;
}
bool Controller_Kinco::Stop(int num)
{
	this->stop[0] = num;
	lrc(this->stop, 10);
	WriteChar(this->stop, 10);
	return true;
}
bool Controller_Kinco::Start(int num)
{
	this->start_1[0] = num;
	lrc(this->start_1, 10);
	this->start_2[0] = num;
	lrc(this->start_2, 10);
	WriteChar(this->start_1, 10);
	WriteChar(this->start_2, 10);
	return true;
}

bool Controller_Kinco::Continuous_operation(int num)
{
	this->continuous_operation[0] = num;
	lrc(this->continuous_operation, 10);
	WriteChar(this->continuous_operation, 10);
	return true;
}

void Controller_Kinco::main_process()
{
	long angle_trans;
	angle_trans = angle_data*20000;
	Position(1, angle_trans);
}