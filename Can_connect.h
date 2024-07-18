#include"ECanVci.h"
#include<iostream>
#include<Windows.h>
#include<string.h>
using namespace std;

//接口卡类型定义
#define USBCAN1 3
#define USBCAN2 4
//函数调用返回状态值
#define STATUS_OK 1
#define STATUS_ERR 0
//通道号
#define CAN1 0
#define CAN2 1

class Can_connect
{
private:
	bool m_connect = false; /*设备启动标致符 false：表示设备未启动或者已经关闭 true：表示设备已
	经启动可以正常收发数据*/
	DWORD DeviceType = USBCAN2; //设备类型 USBCAN2 双通道
	DWORD DeviceInd = 0; //设备索引 0，只有一个 USBCAN


public:
	bool Is_connect = false;
	Can_connect() {};
	virtual ~Can_connect() {};
	bool Initial();
	bool WriteChar(int ID, BYTE* DATA, DWORD LEN);
};

