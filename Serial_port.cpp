#include "Serial_port.h"
#include <iostream>
#include<atlstr.h>
using namespace	std;


Serial_port::Serial_port(string& com_num, int& Baud_rate)
{
	this->port += com_num;
	this->Baud_rate = Baud_rate;
}
Serial_port:: ~Serial_port()
{

}

void Serial_port::setup()
{
	bool open = openport(CA2T(port.c_str()));
	if (open)
	{
		//cout << port.c_str() << " open scuessful!!!" << endl << endl << "information: " << endl;
		if (setupdcb(Baud_rate))
		{
			//cout << "setupDCB scuessful!!!" << endl;
		}
		if (setuptimeout(0, 0, 0, 0, 0))
		{
			//cout << "setupTimeout scuessful!!!" << endl;
		}
		PurgeComm(hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

		PurgeComm(hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
	}
	else
	{
		cout << port.c_str() << " open failed" << endl;
	}
}

bool Serial_port::openport(TCHAR* portname)

{
	hComm = CreateFile(portname,
		GENERIC_READ | GENERIC_WRITE,
		0,
		0,
		OPEN_EXISTING,
		FILE_FLAG_OVERLAPPED,
		0);
	if (hComm == INVALID_HANDLE_VALUE)
	{
		CloseHandle(hComm);
		return false;
	}
	else
	{
		return true;
	}

}



bool Serial_port::setupdcb(int rate_arg)
{
	DCB  dcb;
	int rate = rate_arg;
	memset(&dcb, 0, sizeof(dcb));

	if (!GetCommState(hComm, &dcb))
		return FALSE;
	// set DCB to configure the serial port
	dcb.DCBlength = sizeof(dcb);
	/* ---------- Serial Port Config ------- */
	dcb.BaudRate = rate;
	dcb.Parity = NOPARITY;
	dcb.fParity = 0;
	dcb.StopBits = ONESTOPBIT;
	dcb.ByteSize = 8;
	dcb.fOutxCtsFlow = 0;
	dcb.fOutxDsrFlow = 0;
	dcb.fDtrControl = DTR_CONTROL_DISABLE;
	dcb.fDsrSensitivity = 0;
	dcb.fRtsControl = RTS_CONTROL_DISABLE;
	dcb.fOutX = 0;
	dcb.fInX = 0;
	/* ----------------- misc parameters ----- */
	dcb.fErrorChar = 0;
	dcb.fBinary = 1;
	dcb.fNull = 0;
	dcb.fAbortOnError = 0;
	dcb.wReserved = 0;
	dcb.XonLim = 2;
	dcb.XoffLim = 4;
	dcb.XonChar = 0x13;
	dcb.XoffChar = 0x19;
	dcb.EvtChar = 0;    //reserved ;do not use
	// set DCB
	if (!SetCommState(hComm, &dcb))
		return false;
	else
		return true;
}
bool Serial_port::setuptimeout(DWORD ReadInterval, DWORD ReadTotalMultiplier, DWORD ReadTotalconstant, DWORD WriteTotalMultiplier, DWORD WriteTotalconstant)

{
	COMMTIMEOUTS timeouts;
	timeouts.ReadIntervalTimeout = ReadInterval;
	timeouts.ReadTotalTimeoutConstant = ReadTotalconstant;
	timeouts.ReadTotalTimeoutMultiplier = ReadTotalMultiplier;
	timeouts.WriteTotalTimeoutConstant = WriteTotalconstant;
	timeouts.WriteTotalTimeoutMultiplier = WriteTotalMultiplier;

	if (!SetCommTimeouts(hComm, &timeouts))
	{
		return false;
	}
	else
		return true;
}

void Serial_port::ReciveChar()
{
	this->setup();
	BOOL bRead = TRUE;
	BOOL bResult = TRUE;
	DWORD dwError = 0;
	DWORD BytesRead = 0;
	char RXBuff;

	bool _case = 1;
	char sentenses[100] = { 0 };

	int i = 0;

	for (;;)
	{
		bResult = ClearCommError(hComm, &dwError, &comstat);


		if (comstat.cbInQue == 0)

		{
			continue;
		}



		if (bRead)
		{
			bRead = ReadFile(hComm, &RXBuff, 1, &BytesRead, &m_ov);


			if (RXBuff != '\n' || i > 100)
			{
				sentenses[i] = RXBuff;
				i++;
				if (!bResult)
				{
					switch (dwError = GetLastError())
					{
					case ERROR_IO_PENDING:
					{
						bRead = FALSE;
						break;
					}
					default:
						break;
					}
				}
				else
				{
					bRead = TRUE;
				}  //close if(bRead)
				if (!bRead)
				{
					bRead = TRUE;
					bResult = GetOverlappedResult(hComm, &m_ov, &BytesRead, TRUE);
				}
			}
			else
			{
				_case = false;
			}


		}
		if (_case == false)
		{
			break;
		}
	}
	/*int len = strlen(sentenses);
	int len_ = sizeof(sentenses);*/

	cout << "recive data: " << sentenses << endl;
	string msg = "";
	for (int j = 0; j < i; j++)
	{
		msg += sentenses[j];
	}
	if (CloseHandle(hComm) == 0)
		cout << "close failed" << endl;
}
bool Serial_port::WriteChar(const BYTE* m_szWriteBuffer, DWORD m_nToSend)
{
	this->setup();
	BOOL bWrite = TRUE;
	BOOL bResult = TRUE;
	DWORD BytesSent = 0;
	HANDLE m_hWriteEven = 0;
	ResetEvent(m_hWriteEven);
	if (bWrite)
	{
		m_ov.Offset = 0;
		m_ov.OffsetHigh = 0;
	}
	// Clear buffer
	bResult = WriteFile(hComm, m_szWriteBuffer, m_nToSend, &BytesSent, &m_ov);

	if (!bResult)
	{
		DWORD dwError = GetLastError();
		switch (dwError)
		{
		case ERROR_IO_PENDING:
		{
			BytesSent = 0;
			bWrite = FALSE;
			break;
		}
		default:
			break;
		}
	}
	if (!bWrite)
	{
		bWrite = TRUE;
		bResult = GetOverlappedResult
		(
			hComm,
			&m_ov,
			&BytesSent,
			TRUE
		);
		//deal with the error code
		if (!bResult)
		{
			cout << "GetOverlappedResults() in WriteFile()";
		}
		if (BytesSent != m_nToSend)
		{
			cout << "WARNING: WriteFile() error.. Bytes Sent: " << BytesSent
				<< "; Message Length: " << strlen((char*)m_szWriteBuffer) << endl;
		}
		if (CloseHandle(hComm) == 0)
			cout << "close failed" << endl;
		return TRUE;
	}
	if (CloseHandle(hComm) == 0)
		cout << "close failed" << endl;

}

unsigned short Serial_port::do_lrc(unsigned char* data, int len)
{
	unsigned int lrc = 0;
	for (int i = 0; i < len-1; i++)
	{
		lrc += data[i]; //求和
	}
	lrc = lrc % 256;//取模
	lrc = 256 - lrc;//作差
	return lrc;
}

void Serial_port::lrc(BYTE* output, int size)
{
	output[size - 1] = do_lrc(output, size);
}
