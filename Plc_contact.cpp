#include "Plc_contact.h"
#include<iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#define _WINSOCK_DEPRECATED_NO_WARNINGS 1
#include "winsock2.h"
#include "Line_detector.h"
#pragma comment(lib,"ws2_32.lib")//���ÿ��ļ�
using namespace std;

std::mutex my_mutex3;

Plc_contact::Plc_contact(string& plc_addr, int& plc_port, char& b_camera,char& light_state, char& b_start)
{
	this->plc_addr = plc_addr;
	this->plc_port = plc_port;
    this->b_camera = &b_camera;
    this->light_state = &light_state;
    this->b_start = &b_start;
}

int Plc_contact::main_process()
{
    //�����׽���
    WSADATA wsaData;
    memset(buff, 0, sizeof(buff));
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
    {
        printf("��ʼ��Winsockʧ��");
        return 0;
    }

    SOCKADDR_IN addrSrv;
    addrSrv.sin_family = AF_INET;
    addrSrv.sin_port = htons(plc_port);//�˿ں�
    addrSrv.sin_addr.S_un.S_addr = inet_addr(plc_addr.c_str());//IP��ַ

    //�����׽���
    SOCKET sockClient = socket(AF_INET, SOCK_STREAM, 0);
    if (SOCKET_ERROR == sockClient) {
        printf("Socket() error:%d", WSAGetLastError());
        return 0;
    }
    //�������������������
    if (connect(sockClient, (struct  sockaddr*)&addrSrv, sizeof(addrSrv)) == INVALID_SOCKET) {
        printf("����ʧ��:%d", WSAGetLastError());
        return 0;
    }
    else
    {
        while (1)
        {
            //��������
            recv(sockClient, recv_buff, sizeof(recv_buff), 0);

            
            if (recv_buff[0] == '\x0') recv_to_camera = '0';
            else if (recv_buff[0] == '\x1') recv_to_camera = 'L';
            else if (recv_buff[0] == '\x2') recv_to_camera = 'R';
            else recv_to_camera = '0';

            if (recv_buff[1] == '\x1') recv_to_start = '1';
            else recv_to_start = '0';
            
            //cout << recv_to_camera[0] << "  " << recv_to_camera[1] << endl;

            {
                std::lock_guard<std::mutex> lockGuard(my_mutex);
                *b_camera = recv_to_camera; 
            }

            {
                std::lock_guard<std::mutex> lockGuard(my_mutex2);
                light = *light_state;
            }

            {
                std::lock_guard<std::mutex> lockGuard(my_mutex3);
                *b_start = recv_to_start;
            }

            //��������
            if (light == 'W') send_buff[0] = 1;
            else if (light == 'F') send_buff[0] = 2;
            else if (light == 'S') send_buff[0] = 3;
            else send_buff[0] = 0;
            send(sockClient, send_buff, sizeof(send_buff), 0);
        }
    }

    //�ر��׽���
    closesocket(sockClient);
    WSACleanup();//�ͷų�ʼ��Ws2_32.dll���������Դ��
    //system("pause");//����Ļ����
    return 0;
}
