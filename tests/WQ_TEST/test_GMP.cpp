#include "XUtils/Gmp/GlobalMatchP.h"
#include "xutils.h"
#include <fstream>
#include "opencv2/opencv.hpp"
#include <fstream>

float raw_laser_scan[2400] = { 0 };
int16_t laser_scan_mm[360] = { 0 };

#if 1
int main(int argc, char** argv)
{
	if (argc < 4)
	{
		printf("invalid parameter, usuage: ./test_GMP datasets_dir start_test_case test_cnt \n");
		return -1;
	}
	std::string datasets_dir(argv[1]);
	int start_test_idx = atoi(argv[2]);
	int test_cnt = atoi(argv[3]);
	printf("DataSets dir is %s, start test case %d, test cnt is %d\n", datasets_dir.c_str(), start_test_idx, test_cnt);

	std::ofstream ofs_results("gmp_results.csv");
	for (int test_idx = start_test_idx; test_idx < start_test_idx + test_cnt; test_idx++)
	{
		// 1. load data
		std::stringstream ss;
		ss << datasets_dir << "GMP_" << test_idx << ".bin";
		FILE* fp = fopen(ss.str().c_str(), "rb");

		int map_size_x, map_size_y;
		double map_scale, map_origin_x, map_origin_y;
		memset(raw_laser_scan, 0, 2400 * sizeof(float));
		memset(laser_scan_mm, 0, 360 * sizeof(int16_t));
		

		fread(&map_size_x, sizeof(map_size_x),1,fp);
		fread(&map_size_y, sizeof(map_size_y), 1, fp);
		fread(&map_scale, sizeof(map_scale), 1, fp);
		fread(&map_origin_x, sizeof(map_origin_x), 1, fp);
		fread(&map_origin_y, sizeof(map_origin_y), 1, fp);

		uint8_t* pRawMapData = new uint8_t[map_size_x * map_size_y];
		ON_SCOPE_EXIT([=]() {delete[] pRawMapData; });

		fread(pRawMapData, 1, map_size_x * map_size_y, fp);
		fread(raw_laser_scan, sizeof(float), 2400, fp);
		fclose(fp);

		for (int i = 0; i < 360; i++)
		{
			laser_scan_mm[i] = raw_laser_scan[(int)(i / 0.15)] * 1000;
		}

		printf("**************************TEST SCENE %d *************************\n", test_idx);
		// 2. update map
		updateGMPMap(pRawMapData, map_size_x, map_size_y, 0.05f, map_origin_x, map_origin_y);

		// 3. global match
		std::vector<GMP_results_t> results;
		uint64_t t0 = getTimeStap_us();
		GMPMatch(raw_laser_scan, 2400, 0.15, results);
		uint64_t t1 = getTimeStap_us();
		if (results.size() > 0)
		{
			printf("TEST %d results: %.3f %.3f %.3f %.3f \n", test_idx, results[0].score, results[0].x, results[0].y, results[0].theta_deg);
			std::stringstream ss;
			ss << test_idx << "_bg";
			GMPDrawDebugImage((char*)ss.str().c_str(), raw_laser_scan, 0, 0.15, results[0]);
			ss.str("");
			ss << test_idx;
			GMPDrawDebugImage((char*)ss.str().c_str(), raw_laser_scan, 2400, 0.15, results[0]);
		}

		ofs_results << test_idx << "," << results[0].score << "," << results[0].x << "," << results[0].y << "," << results[0].theta_deg << "," << (t1 - t0) / 1000000.0 << std::endl;
	}
	ofs_results.close();

	return 0;
}
#endif



#if 0
int main(int argc, char** argv)
{
	std::ifstream infile1;
	infile1.open(argv[1]); //amclTest.log

	int max_test_cnt = 9;
	for (int test_cnt =1; test_cnt <= max_test_cnt; test_cnt++)
	{
		if (infile1.eof())
		{
			break;
		}

		// 1. load data
		float odomData_px, odomData_py, odomData_pa;
		int laserConcatenationResultSize = 0;
		infile1 >> odomData_px;
		infile1 >> odomData_py;
		infile1 >> odomData_pa;
		infile1 >> laserConcatenationResultSize;

		int map_size_x, map_size_y;
		double map_scale, map_origin_x, map_origin_y, map_max_occ_dist;
		infile1 >> map_size_x;
		infile1 >> map_size_y;
		infile1 >> map_scale;
		infile1 >> map_origin_x;
		infile1 >> map_origin_y;
		infile1 >> map_max_occ_dist;

		memset(raw_laser_scan, 0, 2400 * sizeof(float));
		memset(laser_scan_mm, 0, 360 * sizeof(int16_t));
		uint8_t* pRawMapData = new uint8_t[map_size_x * map_size_y];
		ON_SCOPE_EXIT([=]() {delete[] pRawMapData; });
		for (int i = 0; i < map_size_x * map_size_y; i++)
		{
			int occ_state;
			float occ_dist;
			infile1 >> occ_state;
			infile1 >> occ_dist;
			pRawMapData[i] = occ_state;

		}
		for (int i = 0; i < laserConcatenationResultSize; i++)
		{
			float x, y, theta;
			infile1 >> x >> y >> theta;
			int idx = (int)std::round((theta + CV_PI) / CV_PI * 180 / 0.15);
			while (idx < 0)
				idx += 2400;
			while (idx > 2400)
				idx -= 2400;
			raw_laser_scan[idx] = std::sqrt(x*x + y * y);
		}
		for (int i = 0; i < 360; i++)
		{
			laser_scan_mm[i] = raw_laser_scan[(int)(i / 0.15)] * 1000;
		}

		std::string end;
		infile1 >> end;

#if 1
		{
			std::stringstream ss;
			ss << "GMP_"<< test_cnt << ".bin";
			FILE* fp = fopen(ss.str().c_str(), "wb");
			fwrite(&map_size_x, sizeof(map_size_x), 1, fp);
			fwrite(&map_size_y, sizeof(map_size_y), 1, fp);
			fwrite(&map_scale, sizeof(map_scale), 1, fp);
			fwrite(&map_origin_x, sizeof(map_origin_x), 1, fp);
			fwrite(&map_origin_y, sizeof(map_origin_y), 1, fp);
			fwrite(pRawMapData, sizeof(uint8_t), map_size_x * map_size_y, fp);
			fwrite(raw_laser_scan, sizeof(float), 2400, fp);
			fclose(fp);
		}
#endif

		printf("**************************TEST SCENE %d *************************\n", test_cnt);
		// 2. update map
		updateGMPMap(pRawMapData, map_size_x, map_size_y, 0.05f, map_origin_x, map_origin_y);

		// 3. global match
		std::vector<GMP_results_t> results;
		GMPMatch(raw_laser_scan, 2400, 0.15, results);
		if (results.size() > 0)
		{
			printf("TEST %d results: %.3f %.3f %.3f %.3f \n", test_cnt, results[0].score, results[0].x, results[0].y, results[0].theta_deg);
			std::stringstream ss;
			ss << test_cnt << "_bg";
			GMPDrawDebugImage((char*)ss.str().c_str(), raw_laser_scan, 0, 0.15, results[0]);
			ss.str("");
			ss << test_cnt;
			GMPDrawDebugImage((char*)ss.str().c_str(), raw_laser_scan, 2400, 0.15, results[0]);
		}
	}
	infile1.close();

	return 0;
}
#endif

#if 0

// load bin file
int main(int argc, char** argv)
{
	int map_size_x, map_size_y;
	double map_scale, map_origin_x, map_origin_y;
	map_size_x = 448;
	map_size_y = 448;
	map_scale = 0.05;
	map_origin_x = -6;
	map_origin_y = 3.55;
	
	

	memset(raw_laser_scan, 0, 2400 * sizeof(float));
	memset(laser_scan_mm, 0, 360 * sizeof(int16_t));
	uint8_t* pRawMapData = new uint8_t[map_size_x * map_size_y];
	ON_SCOPE_EXIT([=]() {delete[] pRawMapData; });
	FILE* fp = fopen("tmp.bin", "rb"); //amclTest.log
	fread(pRawMapData, 1, 17, fp);
	fread(pRawMapData, 1, map_size_x * map_size_y, fp);
	fclose(fp);

	std::fstream ifs;
	ifs.open("wq_laser.txt");
	if (!ifs.is_open())
		return -1;
	for (int i = 0; i < 2400; i++)
	{
		double tmp;
		ifs >> tmp;
		raw_laser_scan[i] = tmp;
	}
	for (int i = 0; i < 360; i++)
	{
		laser_scan_mm[i] = raw_laser_scan[(int)(i / 0.15)] * 1000;
	}
	ifs.close();

#if 1
	{
		FILE* fp = fopen("123.bin", "wb");
		fwrite(&map_size_x, sizeof(map_size_x), 1, fp);
		fwrite(&map_size_y, sizeof(map_size_y), 1, fp);
		fwrite(&map_scale, sizeof(map_scale), 1, fp);
		fwrite(&map_origin_x, sizeof(map_origin_x), 1, fp);
		fwrite(&map_origin_y, sizeof(map_origin_y), 1, fp);
		fwrite(pRawMapData, sizeof(uint8_t), map_size_x * map_size_y, fp);
		fwrite(raw_laser_scan, sizeof(float), 2400, fp);
		fclose(fp);
	}
#endif


	// 2. update map
	updateGMPMap(pRawMapData, map_size_x, map_size_y, 0.05f, map_origin_x, map_origin_y);

	// 3. global match
	int test_cnt = 0;
	std::vector<GMP_results_t> results;
	GMPMatch(raw_laser_scan, 2400, 0.15, results);
	if (results.size() > 0)
	{
		printf("TEST %d results: %.3f %.3f %.3f %.3f \n", test_cnt, results[0].score, results[0].x, results[0].y, results[0].theta_deg);
		std::stringstream ss;
		ss << test_cnt << "_bg";
		GMPDrawDebugImage((char*)ss.str().c_str(), raw_laser_scan, 0, 0.15, results[0]);
		ss.str("");
		ss << test_cnt;
		GMPDrawDebugImage((char*)ss.str().c_str(), raw_laser_scan, 2400, 0.15, results[0]);
	}



	return 0;
}

#endif