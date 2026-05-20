#include <stdio.h>
#include <fcntl.h>
#include "stdint.h"
#include <sys/stat.h>
#include "errno.h"
#include "time.h"
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>

#endif
#include <string>
#include <iostream>
#include "string.h"
#define XD1Y_ADDR  0x18

static int fd;

int xt_i2c_open(void)
{
#ifndef _WIN32
	fd = open("/dev/i2c-1", O_RDWR);
	if (fd < 0)
		return -1;

	ioctl(fd, I2C_TIMEOUT, 10);
	ioctl(fd, I2C_RETRIES, 2);
#endif
	return 0;
}

int xt_i2c_write_reg(uint8_t addr, uint8_t data)
{
#ifndef _WIN32
	uint8_t write_data[2];
	write_data[0] = addr;
	write_data[1] = data;

	struct i2c_msg msgs[1] =
	{
		{XD1Y_ADDR, 0, 2, write_data},
	};

	struct i2c_rdwr_ioctl_data idata = {
		.msgs = msgs,
		.nmsgs = 1,
	};

	if (ioctl(fd, I2C_RDWR, &idata) < 0)
	{
		perror("i2c_write_reg::ioctl");
		return -1;
	}
#endif
	return 0;
}

int xt_i2c_read_reg(uint8_t addr, uint8_t* data)
{
#ifndef _WIN32
	struct i2c_msg msgs[2] = {
		{XD1Y_ADDR, 0, 1, (uint8_t*)&addr},
		{XD1Y_ADDR, I2C_M_RD, 1, data},
	};

	struct i2c_rdwr_ioctl_data idata = {
		.msgs = msgs,
		.nmsgs = 2,
	};

	if (ioctl(fd, I2C_RDWR, &idata) < 0)
	{
		perror("i2c_read_reg::ioctl");
		return -1;
	}
#endif
	return 0;
}

int xt_i2c_write_reg_bulk(uint8_t addr, uint8_t* data, uint16_t len)
{
	uint8_t* transfer_data = new uint8_t[1 + len];
	transfer_data[0] = addr;
	memcpy(transfer_data + 1, data, len);
	struct i2c_msg msgs[1] = {
		{XD1Y_ADDR, 0, 1 + len, transfer_data},
	};

	struct i2c_rdwr_ioctl_data idata = {
		.msgs = msgs,
		.nmsgs = 1,
	};

	if (ioctl(fd, I2C_RDWR, &idata) < 0)
	{
		perror("i2c_read_reg_bulk::ioctl");
		delete[] transfer_data;
		return -1;
	}
	delete[] transfer_data;
	return 0;
}

int xt_i2c_read_reg_bulk(uint8_t addr, uint8_t* data, uint16_t len)
{
#ifndef _WIN32
	struct i2c_msg msgs[2] = {
		{XD1Y_ADDR, 0, 1, (uint8_t*)&addr},
		{XD1Y_ADDR, I2C_M_RD, len, data},
	};

	struct i2c_rdwr_ioctl_data idata = {
		.msgs = msgs,
		.nmsgs = 2,
	};

	if (ioctl(fd, I2C_RDWR, &idata) < 0)
	{
		perror("i2c_read_reg_bulk::ioctl");
		return -1;
	}
#endif
	return 0;
}

int xt_i2c_close(void)
{
#ifndef _WIN32
	if (fd > 0)
		close(fd);
#endif
	return 0;
}




uint64_t xt_getTimeStap_us()
{
#ifdef _WIN32
	static LARGE_INTEGER freq, start;
	LARGE_INTEGER count;
	if (!QueryPerformanceCounter(&count))
		printf("query performance count failed");
	if (!freq.QuadPart) { // one time initialization
		if (!QueryPerformanceFrequency(&freq))
			printf("Query performance frequency failed");
		start = count;
	}
	return ((double)(count.QuadPart - start.QuadPart) / freq.QuadPart) * 1000000;
#else
	struct timespec tp;
	clock_gettime(CLOCK_MONOTONIC, &tp);
	return (uint64_t)((uint64_t)(tp.tv_sec) * (uint64_t)(1000000) + tp.tv_nsec / 1000);
#endif

}


int main(int argc, char** argv)
{
	if (xt_i2c_open() < 0)
	{
		printf("i2c open failed!\n");
		return -1;
	}

	uint32_t SN;
	if (xt_i2c_read_reg_bulk(0x55, (uint8_t*)&SN, 4) < 0)
	{
		printf("i2c read sn failed!\n");
		xt_i2c_close();
		return -1;
	}
	else
		printf("xd1y sn is %d\n", SN);

	int ret, i;
	uint8_t write_buffer[8] = { 0 };
	uint8_t	read_buffer[256] = { 0 };
	read_buffer[0] = 0x11;
	uint8_t startAppCmd = 0x01;
	while (true)
	{
		std::cout << ">>>>  Please input test command  <<<<<" << std::endl;
		std::cout << "\t 0: read version" << std::endl;
		std::cout << "\t 1: set device address" << std::endl;
		std::cout << "\t 2: read 1 byte from register address 0x51" << std::endl;
		std::cout << "\t 3: read 320 bytes from register address 0x60" << std::endl;
		std::cout << "\t 4: send .bin to update firmware" << std::endl;
		std::cout << "\t 5: write 0 to register 0x01" << std::endl;
		std::cout << "\t 6: start app" << std::endl;
		std::cout << "\t 7: start bootloader" << std::endl;
		std::cout << "\t 8: read register" << std::endl;
		std::cout << "\t 9: write register" << std::endl;
		std::cout << ">>";
		int cmd;
		std::cin >> cmd;
		printf("select cmd is %d\n", cmd);
		int devAddr = 0x30;
		uint8_t tmp = 0;
		switch (cmd)
		{
		case 0:
		{
			// exit boot mode, start application
			ret = xt_i2c_write_reg(0x01, 1);
			if (ret < 0)
			{
				printf("Write data error!\n");
				return;
			}
			do
			{
				uint8_t readData = 0;
				usleep(100000);
				xt_i2c_read_reg(0x22, &readData);
				if (0x00 == readData)
				{
					break;
				}
			} while (true);

			uint8_t major = 0;
			uint8_t minor = 0;
			xt_i2c_read_reg(0x53, &major);
			xt_i2c_read_reg(0x54, &minor);
			printf("fw version is %d.%d\n", (int)major, (int)minor);
			break;
		}
		case 1:
			std::cout << "please input device address: ";
			std::cin >> devAddr;
			std::cout << "device address is set as " << devAddr << std::endl;
			break;
		case 2:
			xt_i2c_read_reg(0x51, read_buffer);
			printf("Read 1 byte from (devAddr::regAddr) %02x::51 : %02x\n", devAddr, read_buffer[0]);
			break;
		case 3:
			xt_i2c_read_reg_bulk(0x60, read_buffer, 320);
			printf("read 320 bytes from (devAddr::regAddr) %02x::60 \n", devAddr);
			for (int i = 0; i < 320; i++)
			{
				printf("%02x ", read_buffer[i]);
			}
			printf("\n");
			break;
		case 4:
			void iap_test();
			iap_test();
			break;
		case 5:
			tmp = 0;
			ret = xt_i2c_write_reg(0x01, tmp);
			if (ret < 0)
			{
				printf("Write data error!\n");

			}
			break;
		case 6:	// start app

			// stop boot mode
			ret = xt_i2c_write_reg(0x01, 1);
			if (ret < 0)
			{
				printf("Write data error!\n");
				break;
			}
			break;
		case 7:	// start bootloader
			printf("you should re-power the xd1y to enter bootloader mode");
			break;
		case 8:
		{
			std::cout << "pls. enter register address:";
			int addr = 0;
			std::cin >> addr;
			xt_i2c_read_reg(addr, read_buffer);
			printf("read register 0x%x value is 0x%x(%d)\n", addr, (int)read_buffer[0], (int)read_buffer[0]);
			break;
		}
			
		case 9:
		{
			std::cout << "pls. enter register address and value:";
			int addr = 0;
			int value = 0;
			std::cin >> addr >> value;
			xt_i2c_write_reg(addr, value);
			printf("write register 0x%x with value 0x%x(%d)\n", addr, value, value);
			break;
		}
		default:
			std::cout << " no command matched!!!! " << std::endl;
			break;
		}

	}
	
	return 0;
}


int get_file_size(FILE* fileHandle)
{

	unsigned int current_read_position = ftell(fileHandle);
	int file_size;
	fseek(fileHandle, 0, SEEK_END);
	//��ȡ�ļ��Ĵ�С
	file_size = ftell(fileHandle);
	//�ָ��ļ�ԭ����ȡ��λ��
	fseek(fileHandle, current_read_position, SEEK_SET);

	return file_size;
}

int  read_file(uint8_t* &content)
{
	std::string filename;
	printf("input bin file name: ");
	std::cin >> filename;

	int FWSize = 0;
	FILE* fp = NULL;
	fp = fopen(filename.c_str(), "rb");

	FWSize = get_file_size(fp) - 4;
	printf("Load Firmware file %d bytes!\r\n", FWSize);

	int pageSize = (FWSize + 2 + 1023) / 1024; // 2 is for crc
	int memorySize = pageSize * 1024;
	content = (uint8_t*)malloc(memorySize);

	printf("Load Firmware file %d bytes, PageSize=%d Malloc memgory %d bytes!\r\n",
		FWSize, pageSize, memorySize);

	int index = 0;
	char ch;
	ch = (uint8_t)fgetc(fp);
	ch = (uint8_t)fgetc(fp);
	ch = (uint8_t)fgetc(fp);
	ch = (uint8_t)fgetc(fp); // jump 4 bytes
	while (true)
	{
		if (index < memorySize)
		{
			if (index < (FWSize))
				content[index++] = (uint8_t)fgetc(fp);
			else
				content[index++] = 255;
		}
		else
		{
			printf("read file %d bytes!\r\n", index);
			break;
		}

	}
	fclose(fp);
	return index;
}

void iap_test()
{
	uint8_t* dataPt;
	uint8_t testData[1024];
	for (int i = 0; i < 1024; i++)
	{
		testData[i] = i % 256;
	}

	int len = read_file(dataPt);
	int pageSize = len / 1024;
	uint16_t crc16(unsigned char *updata, unsigned int len);
	uint16_t crc = crc16((unsigned char*)dataPt, 1024 * pageSize - 2);
	//crc = 0;
	*(uint16_t*)(dataPt + 1024 * pageSize - 2) = crc;  // setup crc
	// step-1
	int ret = 0;

	uint8_t readData = 0;


	// read pcb version
	ret = xt_i2c_read_reg(0x23, &readData);
	if (ret < 0)
	{
		printf("read pcb version error!\n");
		return;
	}
	if (1) //(readData == (filename.c_str()[5] - '0'))
	{
		printf("PCB version is %d\n", (int)readData);
		ret = xt_i2c_write_reg(0x25, readData);
		if (ret < 0)
		{
			printf("write pcb version error!\n");
			return;
		}
	}
	else
	{
		printf("PCB version mismatch %d %d\n", (int)readData, (int)readData);
		return;
	}


	// write Pages
	for (int i = 0; i < pageSize; i++)
	{
		printf("write %d/%d page\n",i + 1, pageSize);
		xt_i2c_write_reg_bulk(0x21, (uint8_t*)(dataPt + 1024 * i), 1024);

		// check if mcu finish flash
		do
		{
			usleep(100000);
			xt_i2c_read_reg(0x22, &readData);
			if (0x03 == readData)
			{
				break;
			}
			else
			{
				printf("wait %th page finisehed\n", i+1);
			}
		} while (true);
		usleep(100000);
	}

	printf("write %d pages, enter app mode\n", pageSize);
#if 1
	// exit boot mode, start application
	ret = xt_i2c_write_reg(0x01, 1);
	if (ret < 0)
	{
		printf("Write data error!\n");
		return;
	}
	do
	{
		usleep(100000);
		xt_i2c_read_reg(0x22, &readData);
		if (0x00 == readData)
		{
			break;
		}
	} while (true);
#endif
	printf("write %d pages to device, finish bootloader\r\n", pageSize);

	free(dataPt);

	uint8_t major = 0;
	uint8_t minor = 0;
	xt_i2c_read_reg(0x53, &major);
	xt_i2c_read_reg(0x54, &minor);
	printf("fw version is %d.%d\n", (int)major, (int)minor);
}


















static unsigned char auchCRCHi[] =
{
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
	0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40
};

static unsigned char auchCRCLo[] =
{
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
	0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
	0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
	0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
	0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
	0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
	0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
	0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
	0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
	0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
	0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
	0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
	0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
	0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
	0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
	0x40
};

// little endian, platform independent
uint16_t crc16(unsigned char *updata, unsigned int len)
{
	unsigned char uchCRCHi = 0xff;
	unsigned char uchCRCLo = 0xff;
	unsigned int  uindex;
	while (len--)
	{
		uindex = uchCRCLo ^ *updata++;
		uchCRCLo = uchCRCHi ^ auchCRCHi[uindex];
		uchCRCHi = auchCRCLo[uindex];
	}

	uint16_t ret;
	uint8_t* pData = (uint8_t*)(&ret);
	pData[0] = uchCRCLo;
	pData[1] = uchCRCHi;
	return ret;
}