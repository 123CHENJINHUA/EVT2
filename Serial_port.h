#pragma once
#include <iostream>
#include <Windows.h>
#include<atlstr.h>
using namespace std;


class Serial_port
{
private:
	OVERLAPPED m_ov;
	COMSTAT comstat;
	DWORD m_dwCommEvent;
	HANDLE hComm;
	string port = "com";
	int Baud_rate = 19200;
public:
	Serial_port(string& com_num, int& Baud_rate);
	virtual ~Serial_port();
	bool openport(TCHAR* portname);
	bool setupdcb(int rate_arg);
	bool setuptimeout(DWORD ReadInterval, DWORD ReadTotalMultiplier, DWORD ReadTotalconstant, DWORD WriteTotalMultiplier, DWORD WriteTotalconstant);
	void ReciveChar();
	bool WriteChar(const BYTE* m_szWriteBuffer, DWORD m_nTosend);
	void setup();

	unsigned short do_lrc(unsigned char* data, int len);
	void lrc(BYTE* output, int size);
};

