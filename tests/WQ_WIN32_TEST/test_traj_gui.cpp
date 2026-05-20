#include <iostream>
#include "opencv2/opencv.hpp"
#pragma comment (lib, "ws2_32")
#include <stdio.h>
#include <Winsock2.h> //Socket的函数调用　
#include <windows.h>

#define SERVER_ADDR "192.168.10.13"
#define SERVER_PORT 7890

std::vector<cv::Point2f> planned_tra;
int moved_tra_cnt = 0;
std::vector<cv::Point2f> moved_points;


int updateGUI(bool bInit)
{
	static cv::Mat img;
	static int last_moved_tra_cnt = 0;
	static const cv::Point2f offset(150, 150);
	if (bInit)
	{
		img = cv::Mat(1000, 1000, CV_8UC3, cv::Scalar(0));
		for (int i = 0; i < planned_tra.size(); i++)
		{
			if (i >= 1)
			{
				cv::Point p = planned_tra[i - 1];
				cv::line(img, planned_tra[i - 1] + offset, planned_tra[i] + offset, cv::Scalar(0, 0, 255), 1);
			}
			cv::circle(img, planned_tra[i] + offset, 1, cv::Scalar(0, 255, 0), -1);
		}
		last_moved_tra_cnt = 0;
	}
	if (moved_points.size() > 0)
	{
		for (int i = 0; i < moved_points.size(); i++)
		{
			img.at<cv::Vec3b>(moved_points[i] + offset) = cv::Vec3b(255, 255, 255);
		}
		moved_points.clear();
	}
	if (moved_tra_cnt != last_moved_tra_cnt)
	{
		if (last_moved_tra_cnt > moved_tra_cnt)
			last_moved_tra_cnt = 0;

		for (int i = last_moved_tra_cnt; i < moved_tra_cnt; i++)
		{
			cv::circle(img, offset + planned_tra[i], 1, cv::Scalar(255, 0, 0), -1);
		}
		last_moved_tra_cnt = 0;
	}
	cv::imshow("img", img);
	cv::waitKey(10);

	return 0;
}

int main(int argc, char** argv)
{
	std::string ip;
	if (argc < 2)
	{
		printf("输入参数不足,,使用默认IP地址：%s\n", SERVER_ADDR);
		ip = std::string(SERVER_ADDR);
	}
	else
	{
		ip = std::string(argv[1]);
	}

	WSADATA wsaData;
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
		printf("Winsock load faild!\n");
		return 1;
	}

	while (1)
	{
		//  服务器套接字
		SOCKET sHost = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (sHost == INVALID_SOCKET) {
			printf("socket faild!\n");
			WSACleanup();
			return -1;
		}

		SOCKADDR_IN servAddr;
		servAddr.sin_family = AF_INET;
		//  注意   当把客户端程序发到别人的电脑时 此处IP需改为服务器所在电脑的IP
		servAddr.sin_addr.S_un.S_addr = inet_addr(ip.c_str());
		servAddr.sin_port = htons(SERVER_PORT);

		//  连接服务器
		if (connect(sHost, (LPSOCKADDR)&servAddr, sizeof(servAddr)) == SOCKET_ERROR)
		{
			printf("connect faild!\n");
			closesocket(sHost);
			Sleep(1000);
			continue;
		}
		printf("连接到服务器 IP:[%s],port:[%d]\n", inet_ntoa(servAddr.sin_addr), ntohs(servAddr.sin_port));



		char bufSend[1];
		uint8_t header[6];
		int retVal;


		bool bLostConnection = false;
		bool bInited = false;
		while (1)
		{
			if (!bInited)
			{
				bInited = true;
				// 1. get plan tra
				bufSend[0] = 1;
				retVal = send(sHost, bufSend, 1, 0);
				if (retVal == SOCKET_ERROR)
				{
					printf("send faild!\n");
					bLostConnection = true;
					break;
				}
				Sleep(200);

				if (recv(sHost, (char*)(header), 6, 0) != 6)
				{
					printf("receive header faild!\n");
					bLostConnection = true;
					break;
				}
				uint16_t type = header[0];
				type <<=8;
				type += header[1];
				uint32_t length = header[2];
				length <<= 8;
				length += header[3];
				length <<= 8;
				length += header[4];
				length <<= 8;
				length += header[5];


				if (type != 1)
				{
					printf("type mismatch!\n");
					bLostConnection = true;
					break;
				}
				uint8_t* buf = new uint8_t[length + 1];
				int lReceivedSize = 0;
				while (lReceivedSize < length)
				{
					retVal = recv(sHost, (char*)(buf + lReceivedSize), length - lReceivedSize, 0);
					if (retVal < 0)
					{
						printf("recive faild!\n");
						bLostConnection = true;
						delete[] buf;
						break;
					}
					else
					{
						lReceivedSize += retVal;
					}
				}
				planned_tra.clear();
				for (int i = 0; i < lReceivedSize; i = i + sizeof(cv::Point2f))
				{
					cv::Point2f p = *((cv::Point2f*)(buf + i));
					planned_tra.push_back(p);
				}
				delete[] buf;
				moved_points.clear();
				moved_tra_cnt = 0;
				updateGUI(true);
				
			}

			// 2. get points
			bufSend[0] = 2;
			retVal = send(sHost, bufSend, 1, 0);
			if (retVal == SOCKET_ERROR)
			{
				printf("send faild!\n");
				bLostConnection = true;
				break;
			}
			Sleep(50);
			
			if (recv(sHost, (char*)(&header), 6, 0) != 6)
			{
				printf("receive length faild!\n");
				bLostConnection = true;
				break;
			}
			uint16_t type = header[0];
			type <<= 8;
			type += header[1];
			uint32_t length = header[2];
			length <<= 8;
			length += header[3];
			length <<= 8;
			length += header[4];
			length <<= 8;
			length += header[5];
			if (type != 2)
			{
				printf("type mismatch!\n");
				bLostConnection = true;
				break;
			}
			uint8_t* buf = new uint8_t[length + 1];
			int lReceivedSize = 0;
			while (lReceivedSize < length)
			{
				retVal = recv(sHost, (char*)(buf + lReceivedSize), length - lReceivedSize, 0);
				if (retVal < 0)
				{
					printf("recive faild!\n");
					bLostConnection = true;
					delete[] buf;
					break;
				}
				else
				{
					lReceivedSize += retVal;
				}
			}
			moved_tra_cnt = *((uint32_t*)buf);
			if (lReceivedSize > 4)
			{
				for (int i = 4; i < lReceivedSize; i = i + sizeof(cv::Point2f))
				{
					cv::Point2f p = *((cv::Point2f*)(buf + i));
					moved_points.push_back(p);
				}
				updateGUI(false);
			}
		}
		if (bLostConnection)
		{
			printf("connection closed!\n");
			closesocket(sHost);
			continue;
		}
	}
}



