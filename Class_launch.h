//多线程启动程序

#include<iostream>
using namespace std;

class Class_launch
{
private:
	string plc_addr = "192.168.2.1";
	int plc_port = 100;

	char Is_start = '0';
	bool Is_enable = false;
public:

	Class_launch();
	Class_launch(Class_launch& c);
	virtual ~Class_launch();

	void line_detector(char &b_camera, char& light_state, double& angle);
	void controller_Kinco(double& angle, char& b_start);
	void controller_Benmo(double& angle, char& b_start);
	void plc_contact(char& b_camera, char& light_state, char& b_start);
};

