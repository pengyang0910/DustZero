#include "XD1Y/xd1y.h"
#include "stdio.h"
#include "xutils.h"
#include <iostream>
#include <fstream>
#include <sstream>

static uint8_t xd1y_raw_data[XD1Y_RAW_POINT_CNT];
static uint16_t xd1y_data[XD1Y_POINT_CNT];
static uint16_t xd1y_intensity[XD1Y_POINT_CNT];
int main()
{
	openXD1Y();
#ifdef RK3566_BUILD
	xd1y_load_intrinsic_file("/userdata/laserConfig/XD1Y_Calibration.bin");
#else 
	xd1y_load_intrinsic_file("/root/XD1Y_Calibration.bin");
#endif
	


	float xd1y_height_mm = 67;
	float xd1y_pitch_angle_degree = 17;

	std::ifstream ifs("../Config/xbase.cfg");
	std::string line;
	while (std::getline(ifs, line))
	{
		std::stringstream ss(line);
		std::string s;
		ss >> s;
		if (s == "XD1Y_HEIGHT_MM")
		{
			ss >> xd1y_height_mm;
		}
		else if (s == "XD1Y_PITCH_ANGLE_DEGREE")
		{
			ss >> xd1y_pitch_angle_degree;
		}
	}

	printf("xd1y install height is %f mm, pitch angle is %f degree", xd1y_height_mm, xd1y_pitch_angle_degree);

	while (1)
	{
		if (updateXD1Y(xd1y_raw_data) > 0)
		{			
			xd1y_transform(xd1y_raw_data, xd1y_data, xd1y_intensity, NULL);
			/*for (int i = 0; i < XD1Y_RAW_POINT_CNT; i++)
			{
				printf("%d ", xd1y_raw_data[i]);
			}
			printf("\n\n");*/	
			int result = xd1yPointCloudProcess(xd1y_data, xd1y_pitch_angle_degree, xd1y_height_mm);
			printf("[%d]%d\n", getTimeStap_ms(), result);
		}

		sleep_us(20000);
	}
	closeXD1Y();

	return 0;
}


