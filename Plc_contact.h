#include<iostream>
#include <string>
using namespace std;

class Plc_contact
{
private:
	char buff[1024];
	char recv_buff[4] = { 0 };
	char recv_to_camera = '0';
	char recv_to_start = '0';
	char send_buff[1] = { 0 };
	char light;

	char* b_camera = NULL;
	char* light_state = NULL;
	char* b_start = NULL;

	string plc_addr = "192.168.2.1";
	int plc_port = 100;
public:

	Plc_contact(string& plc_addr, int& plc_port, char& b_camera, char& light_state, char& b_start);
	virtual ~Plc_contact() {};

	int main_process();
};

