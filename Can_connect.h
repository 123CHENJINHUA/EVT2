#include"ECanVci.h"
#include<iostream>
#include<Windows.h>
#include<string.h>
using namespace std;

//�ӿڿ����Ͷ���
#define USBCAN1 3
#define USBCAN2 4
//�������÷���״ֵ̬
#define STATUS_OK 1
#define STATUS_ERR 0
//ͨ����
#define CAN1 0
#define CAN2 1

class Can_connect
{
private:
	bool m_connect = false; /*�豸�������·� false����ʾ�豸δ���������Ѿ��ر� true����ʾ�豸��
	���������������շ�����*/
	DWORD DeviceType = USBCAN2; //�豸���� USBCAN2 ˫ͨ��
	DWORD DeviceInd = 0; //�豸���� 0��ֻ��һ�� USBCAN


public:
	bool Is_connect = false;
	Can_connect() {};
	virtual ~Can_connect() {};
	bool Initial();
	bool WriteChar(int ID, BYTE* DATA, DWORD LEN);
};

