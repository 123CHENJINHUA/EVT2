#include "Can_connect.h"

using namespace std;

bool Can_connect::Initial()
{
	if (OpenDevice(DeviceType, DeviceInd, 0) != STATUS_OK)
	{
		cout << "打开设备失败" << endl;
		return false;
	}
	INIT_CONFIG init_config;
	memset(&init_config, 0, sizeof(init_config));
	init_config.AccCode = 0;
	init_config.AccMask = 0xffffff; //不滤波
	init_config.Filter = 0;
	init_config.Timing0 = 0x00;
	init_config.Timing1 = 0x1c; //波特率 500k
	init_config.Mode = 0; //正常模式
	if (InitCAN(DeviceType, DeviceInd, CAN1, &init_config) != STATUS_OK)
	{
		cout << "初始化通道失败" << endl;
		CloseDevice(DeviceType, DeviceInd);
		return false;
	}
	if (StartCAN(DeviceType, DeviceInd, CAN1) != STATUS_OK)
	{
		cout << "打开通道失败" << endl;
		CloseDevice(DeviceType, DeviceInd);
		return false;
	}
	Sleep(1000);
	return true;
}

bool Can_connect::WriteChar(int ID, BYTE* DATA, DWORD LEN)
{
	if (Is_connect == false) { Sleep(1000); Is_connect = Initial(); }
	if (Is_connect == false) { Initial(); return false; }
	else
	{
		CAN_OBJ frame;
		memset(&frame, 0, sizeof(frame));
		frame.ID = ID;
		frame.DataLen = LEN;
		frame.SendType = 0;
		frame.RemoteFlag = 0;
		frame.ExternFlag = 0;
		memcpy(frame.Data, DATA, frame.DataLen);
		if (Transmit(DeviceType, DeviceInd, CAN1, &frame, 1) != STATUS_OK)
		{
			cout << "发送数据失败" << endl;
			Sleep(1000);
			return false;
		}
	}
	return true;
}
