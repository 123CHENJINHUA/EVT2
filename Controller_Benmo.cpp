#include "Controller_Benmo.h"
using namespace std;

bool Controller_Benmo::Controller_Initial()
{
	WriteChar(0x105, disable, 8);
	WriteChar(0x10A, set_acc, 8);
	WriteChar(0x10A, set_v, 8);
	WriteChar(0x105, set_mode, 8);
	WriteChar(0x105, set_release, 8);
	return true;
}
bool Controller_Benmo::Enable()
{
	WriteChar(0x105, enable, 8);
	return true;
}
bool Controller_Benmo::Disable()
{
	WriteChar(0x105, disable, 8);
	return true;
}

bool  Controller_Benmo::Position(double position)
{

	long positon_trans;
	positon_trans = long(position + 0.5);  //ËÄÉáÎåÈë
	this->position[0] = positon_trans / 256;
	this->position[1] = positon_trans % 256;
	WriteChar(0x32, this->position, 8);
	return true;
}
bool Controller_Benmo::main_process()
{
	double angle_trans;
	angle_trans = (angle_data*45 + 90) * 2.78;
	//cout << angle_trans << endl;
	Position(angle_trans);
	return true;
}