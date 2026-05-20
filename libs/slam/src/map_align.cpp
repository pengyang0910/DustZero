
#include "slam/map_align.h"
#include "gmapping/scanmatcher/gridlinetraversal.h"
#include "xutils/xutils.h"

bool DStaMapLoad(const char* filepath,cv::Mat& img)
{
	FILE* fp = fopen(filepath, "rb");
	if (fp == NULL) return false;

	int width, height;
	fread(&width, sizeof(unsigned int), 1, fp);
	fread(&height, sizeof(unsigned int), 1, fp);

	img = cv::Mat(cv::Size(height, width), CV_8UC1, cv::Scalar(0));
	fread(img.data, 1, width * height, fp);
	fclose(fp);

	return true;
}

void Rotate(const cv::Mat& srcImg, cv::Mat& dstImg, double angle)
{
	cv::Point2f center(srcImg.cols / 2, srcImg.rows / 2);
	cv::Mat M = cv::getRotationMatrix2D(center, angle, 1);
	cv::warpAffine(srcImg, dstImg, M, cv::Size(srcImg.cols, srcImg.rows));
}

void Rotate(const cv::Mat& srcImg, cv::Mat& dstImg, double angle, cv::Point2f center)
{
	//cv::Point2f center(srcImg.cols / 2, srcImg.rows / 2);
	cv::Mat M = cv::getRotationMatrix2D(center, angle, 1);
	cv::warpAffine(srcImg, dstImg, M, cv::Size(srcImg.cols, srcImg.rows));
}

float Map_align::calImgScore(cv::Mat img)
{
	float cost = 0;
	cv::Mat rowSum(1, img.rows, CV_8UC1, cv::Scalar(0));
	cv::Mat colSum(1, img.cols, CV_8UC1, cv::Scalar(0));

	for (int i = 0;i < img.rows;i++)
	{
		int sum = 0;
		for (int j = 0;j < img.cols;j++)
		{
			if (img.at<unsigned char>(i, j) == 100)
				sum++;
		}
		rowSum.at<unsigned char>(0, i) = sum;
	}

	for (int i = 0;i < img.cols;i++)
	{
		int sum = 0;
		for (int j = 0;j < img.rows;j++)
		{
			if (img.at<unsigned char>(j, i) == 100)
				sum++;
		}
		colSum.at<unsigned char>(0, i) = sum;
	}

	cv::Scalar     mean;
	cv::Scalar     dev;

	cv::meanStdDev(colSum, mean, dev);
	float s1 = dev.val[0];

	cv::meanStdDev(rowSum, mean, dev);
	float s2 = dev.val[0];

	cost = sqrt(s1 * s1 + s2 * s2);

	return cost;
}


cv::Mat rotationImg;

bool Map_align::mapAlign(cv::Mat img,float &align_theta,float &center_x,float &center_y)
{
	// if (!mapIsRotation(img))
	// {
	// 	return false;
	// }
	
	cv::Mat tmpImg;
	img.copyTo(tmpImg);

	cv::Mat newImg = cv::Mat(cv::Size(img.rows, img.cols), CV_8UC1, cv::Scalar(0));

	std::vector<cv::Point2i> mapPoints;
	//std::vector<int> mapPointsIndex;

	cv::Point2f center0(img.cols / 2, img.rows / 2);
	cv::Point2f center(img.cols / 2, img.rows / 2);

	for (int i = 0;i < img.rows;i++)
	{
		for (int j = 0;j < img.cols;j++)
		{
			if (img.at<unsigned char>(i, j) == 100)
			{
				cv::Point2i mp;
				mp.x = i;
				mp.y = j;
				mapPoints.push_back(cv::Point2i(j, i));
				newImg.at<unsigned char>(i, j) = 100;
			}
		}
	}
	
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(newImg, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	
	std::vector<cv::Point> contour_points;
	for (int i = 0; i < contours.size();i++)
	{
		//cv::drawContours(tmpImg, contours, i, cv::Scalar(30));
		for (int j = 0; j < contours[i].size();j++)
		{
			contour_points.push_back(contours[i][j]);
		}
	}
	float radius;
	cv::minEnclosingCircle(contour_points, center, radius);
	//cv::circle(tmpImg, center, radius, cv::Scalar(0, 0, 255));
    //center_x =center.x;
	//center_y =center.y;

	float maxScore = 0;
	int bestAngle = 0;

	for (size_t j = 0; j < mapPoints.size(); j++)
	{
		mapPoints[j].y = mapPoints[j].y - center.y;//+(center0.y- center.y);
		mapPoints[j].x = mapPoints[j].x - center.x;//+(center0.x - center.x);
	}
	
	for (int i = -90;i < 90;i += 2)
	{
		//cv::Mat img3 = cv::Mat(cv::Size(img.rows, img.cols), CV_8UC1, cv::Scalar(255));
		cv::Mat rowSum(1, img.rows, CV_8UC1, cv::Scalar(0));
		cv::Mat colSum(1, img.cols, CV_8UC1, cv::Scalar(0));
		float score = 0;
		float theta = i * 3.14159 / 180.0;

		float r1 = cos(theta);
		float r2 = sin(theta);
		for (size_t j = 0; j < mapPoints.size(); j++)
		{
			float y0 = mapPoints[j].y;
			float x0 = mapPoints[j].x;
			int x = (x0 * r1 - y0 * r2+0.5) + center.x+(center0.x- center.x);
			int y = (x0 * r2 + y0 * r1+0.5) + center.y+(center0.y- center.y);

			if (x >= 0 && x < img.rows)
			{
				rowSum.at<unsigned char>(0, x)++;
			}

			if (y >= 0 && y < img.cols)
			{
				colSum.at<unsigned char>(0, y)++;
			}
		}

		cv::Scalar     mean;
		cv::Scalar     dev;

		cv::meanStdDev(colSum, mean, dev);
		float s1 = dev.val[0];

		cv::meanStdDev(rowSum, mean, dev);
		float s2 = dev.val[0];

		score = sqrt(s1 * s1 + s2 * s2);

		//uint64_t st2 = getTimeStap_us();
		//printf("%d score is %f   %f \n", i, score, diff_t1);
		if (score > maxScore)
		{
			maxScore = score;
			bestAngle = i;
		}
	}

	float finalAngle = 0;
	maxScore = 0;
	for (int i = 0;i < 10;i++)
	{
		float angle = bestAngle - 2.5 + 0.5 * i;
		cv::Mat rowSum(1, img.rows, CV_8UC1, cv::Scalar(0));
		cv::Mat colSum(1, img.cols, CV_8UC1, cv::Scalar(0));
		float score = 0;//calImgScore(tmpImg1, maxObs);
		float theta = angle * 3.14159 / 180.0;
	
		float r1 = cos(theta);
		float r2 = sin(theta);
		//uint64_t st2 = getTimeStap_us();
		
		cv::Mat pos = (cv::Mat_<float>(3, 1) << 0, 0, 0);
		for (size_t j = 0; j < mapPoints.size(); j++)
		{
			float y0 = mapPoints[j].y;
			float x0 = mapPoints[j].x;

			int x = (x0 * r1 - y0 * r2 + 0.5) + center.x+(center0.x- center.x);
			int y = (x0 * r2 + y0 * r1 + 0.5) + center.y+(center0.y- center.y);

			if (x >= 0 && x < img.rows)
			{
				rowSum.at<unsigned char>(0, x)++;
			}

			if (y >= 0 && y < img.cols)
			{
				colSum.at<unsigned char>(0, y)++;
			}
		}

		cv::Scalar     mean;
		cv::Scalar     dev;

		//uint64_t st3 = getTimeStap_us();
		
		cv::meanStdDev(colSum, mean, dev);
		float s1 = dev.val[0];

		cv::meanStdDev(rowSum, mean, dev);
		float s2 = dev.val[0];

		score = sqrt(s1 * s1 + s2 * s2);

		//printf("%f score is %f  \n", angle, score);
		if (score > maxScore)
		{
			maxScore = score;
			finalAngle = angle;
		}
	}

	printf("bestAngle is %f  %f\r\n", finalAngle, maxScore);
	align_theta= (finalAngle)* M_PI / 180.0;

	return  true;
}


bool  Map_align::mapAlign(std::vector<int>occPts,int width,int height,float &theta,cv::Point2f center)
{
	std::vector<cv::Point2i> mapPoints;
	cv::Point2f center0(height/2, width / 2);

	for (int i = 0;i < occPts.size();i++)
	{
		cv::Point2i mp;
		mp.x = occPts[i]%width - center.x;
		mp.y = occPts[i]/width - center.y;
		mapPoints.push_back(mp);
	}

	float maxScore = 0;
	int bestAngle = 0;
	
	std::vector<int> candidateAngles;
	for (size_t i = 0; i < 90; i+=2)
	{
		candidateAngles.push_back(i);
		if(i>0)
		{
			candidateAngles.push_back(-i);
		}
	}

	//for (int i = -90;i < 90;i += 2)
	for (int i = 0;i < candidateAngles.size();i ++)
	{
		//cv::Mat img3 = cv::Mat(cv::Size(img.rows, img.cols), CV_8UC1, cv::Scalar(255));
		cv::Mat rowSum(1, height, CV_8UC1, cv::Scalar(0));
		cv::Mat colSum(1, width, CV_8UC1, cv::Scalar(0));
		float score = 0;
		float theta = candidateAngles[i] * 3.14159 / 180.0;

		float r1 = cos(theta);
		float r2 = sin(theta);
		for (size_t j = 0; j < mapPoints.size(); j++)
		{
			float y0 = mapPoints[j].y;
			float x0 = mapPoints[j].x;
			int x = (x0 * r1 - y0 * r2+0.5) + center.x+(center0.x- center.x);
			int y = (x0 * r2 + y0 * r1+0.5) + center.y+(center0.y- center.y);

			if (x >= 0 && x < height)
			{
				rowSum.at<unsigned char>(0, x)++;
			}

			if (y >= 0 && y < width)
			{
				colSum.at<unsigned char>(0, y)++;
			}
		}

		cv::Scalar     mean;
		cv::Scalar     dev;

		cv::meanStdDev(colSum, mean, dev);
		float s1 = dev.val[0];

		cv::meanStdDev(rowSum, mean, dev);
		float s2 = dev.val[0];

		score = sqrt(s1 * s1 + s2 * s2);

		//uint64_t st2 = getTimeStap_us();
		printf("%d score is %f  \n", candidateAngles[i], score);
		if (score > maxScore)
		{
			maxScore = score;
			bestAngle = candidateAngles[i];
		}
	}

	float finalAngle = 0;
	maxScore = 0;
	for (int i = 0;i < 10;i++)
	{
		float angle = bestAngle - 2.5 + 0.5 * i;
		cv::Mat rowSum(1, height, CV_8UC1, cv::Scalar(0));
		cv::Mat colSum(1, width, CV_8UC1, cv::Scalar(0));
		float score = 0;//calImgScore(tmpImg1, maxObs);
		float theta = angle * 3.14159 / 180.0;
	
		float r1 = cos(theta);
		float r2 = sin(theta);
		//uint64_t st2 = getTimeStap_us();
		
		cv::Mat pos = (cv::Mat_<float>(3, 1) << 0, 0, 0);
		for (size_t j = 0; j < mapPoints.size(); j++)
		{
			float y0 = mapPoints[j].y;
			float x0 = mapPoints[j].x;

			int x = (x0 * r1 - y0 * r2 + 0.5) + center.x+(center0.x- center.x);
			int y = (x0 * r2 + y0 * r1 + 0.5) + center.y+(center0.y- center.y);

			if (x >= 0 && x < height)
			{
				rowSum.at<unsigned char>(0, x)++;
			}

			if (y >= 0 && y < width)
			{
				colSum.at<unsigned char>(0, y)++;
			}
		}

		cv::Scalar     mean;
		cv::Scalar     dev;

		//uint64_t st3 = getTimeStap_us();
		
		cv::meanStdDev(colSum, mean, dev);
		float s1 = dev.val[0];

		cv::meanStdDev(rowSum, mean, dev);
		float s2 = dev.val[0];

		score = sqrt(s1 * s1 + s2 * s2);

		//printf("%f score is %f  \n", angle, score);
		if (score > maxScore)
		{
			maxScore = score;
			finalAngle = angle;
		}
	}

	//printf("bestAngle is %f  %f\r\n", finalAngle, maxScore);
	theta= (finalAngle)* 3.14159 / 180.0;

	return  true;
}

bool Map_align::mapIsRotation(cv::Mat img)
{
	cv::Mat tmpImg;
	img.copyTo(tmpImg);
	// 直线提取
	cv::Mat dstImg, midImage;
	cv::Canny(tmpImg, dstImg, 1, 80, 3);
	//cv::imshow("轮廓图", dstImg);
	std::vector<cv::Vec2f> lines;
	cv::HoughLines(dstImg, lines, 1, 1.5*CV_PI / 180, 50, 0, 0);
	if (lines.size()==0)
	{
		return false;
	}
	
	return true;

}

float checkIsWall(std::vector<cv::Point2f> mapPoints, cv::Point3f pose1, cv::Point3f pose2)
{
	int validPts = 0;
	for (size_t j = 0; j < mapPoints.size(); j++)
	{
		float y = mapPoints[j].y;
		float x = mapPoints[j].x;

		if (fabs(pose1.x - pose2.x) > fabs(pose1.y - pose2.y))  // shui ping 
		{
			if (fabs(y - (pose1.y+ pose2.y)/2) <= 0.25)
				validPts++;
		}
		else   // shuzhi 
		{
			if (fabs(x - (pose1.x + pose2.x) / 2) <= 0.25)
				validPts++;
		}
	}
	float percent = 0;
	if (fabs(pose1.x - pose2.x) > fabs(pose1.y - pose2.y)) percent = 0.05*validPts / fabs(pose1.x - pose2.x);
	else  percent = 0.05*validPts / fabs(pose1.y - pose2.y);
	
	return percent;
	
	// if (percent>0.4)
	// {
	// 	return true;
	// }
	// return false;
}



void Map_align::checkDir(std::vector<cv::Point2i> mapPoints,int width,int height, float &pa)
{
	pa=0;
	cv::Mat rowSum(1, height, CV_8UC1, cv::Scalar(0));
	cv::Mat colSum(1, width, CV_8UC1, cv::Scalar(0));
	for (size_t j = 0; j < mapPoints.size(); j++)
	{
		int y = mapPoints[j].y;
		int x = mapPoints[j].x;

		if (x >= 0 && x < height)
		{
			rowSum.at<unsigned char>(0, x)++;
		}

		if (y >= 0 && y < width)
		{
			colSum.at<unsigned char>(0, y)++;
		}
	}

	int row_max=0;
	int col_max=0;
	for (size_t j = 0; j < width; j++)
	{
		if(colSum.at<unsigned char>(0, j)>col_max)
			col_max= colSum.at<unsigned char>(0, j);
	}
	for (size_t j = 0; j < height; j++)
	{
		if(rowSum.at<unsigned char>(0, j)>row_max)
			row_max= rowSum.at<unsigned char>(0, j);
	}

	if (row_max>col_max)
	{
		pa=M_PI_2;
	}
	
}

float calDistTline(cv::Point3f robot_pos,cv::Point3f p1,cv::Point3f p2)
{
  float c= sqrt((robot_pos.x-p1.x)*(robot_pos.x-p1.x)+(robot_pos.y-p1.y)*(robot_pos.y-p1.y));
  float b= sqrt((robot_pos.x-p2.x)*(robot_pos.x-p2.x)+(robot_pos.y-p2.y)*(robot_pos.y-p2.y));
  float a= sqrt((p2.x-p1.x)*(p2.x-p1.x)+(p2.y-p1.y)*(p2.y-p1.y));

  float p= (a+b+c)/2;
  float h= 2*sqrt(p*(p-a)*(p-b)*(p-c))/a;

  return h;

}

std::vector<cv::Point3f> Map_align::constructBlockFromMap(std::vector<cv::Point2f> mapPoints,cv::Point3f robot_pos,cv::Point3f &wallPose,float block_len,float Pa)
{
	std::vector<float> xVect;
	std::vector<float> yVect;
	std::vector<cv::Point3f> ret_poses;

	for (size_t j = 0; j < mapPoints.size(); j++)
	{
		xVect.push_back(mapPoints[j].x);
		yVect.push_back(mapPoints[j].y);	
	}

    float off_set=0.07;
	float minY = *std::min_element(yVect.begin(), yVect.end());
	float maxY = *std::max_element(yVect.begin(), yVect.end());
	float minX = *std::min_element(xVect.begin(), xVect.end());
	float maxX = *std::max_element(xVect.begin(), xVect.end());
	
	cv::Point3f pose;
	std::vector<cv::Point3f> tmpRectPoses;
	//printf("minX: %.4f, minY: %.4f maxX: %.4f maxY: %.4f", minX, minY, maxX, maxY);
	/*  (minX, minY)-->(maxX, minY)-->(maxX, maxY)-->(minX, maxY) */
	pose.x = minX;
	pose.y = minY;
	pose.z = atan2(pose.y, pose.x);
	tmpRectPoses.push_back(pose);

	pose.x = maxX;
	pose.y = minY;
	pose.z = atan2(pose.y, pose.x);
	tmpRectPoses.push_back(pose);

	pose.x = maxX;
	pose.y = maxY;
	pose.z = atan2(pose.y, pose.x);
	tmpRectPoses.push_back(pose);

	pose.x = minX;
	pose.y = maxY;
	pose.z = atan2(pose.y, pose.x);
	tmpRectPoses.push_back(pose);

	// rotate back to map coordinator
	float minDis = 1000;
	int minIndex=-1;
	for (int i = 0; i < tmpRectPoses.size(); i++)
	{
		float tmpDis= sqrt((tmpRectPoses[i].y - robot_pos.y) * (tmpRectPoses[i].y - robot_pos.y) + (tmpRectPoses[i].x - robot_pos.x) * (tmpRectPoses[i].x - robot_pos.x));
		if(tmpDis<minDis)
		{
			minDis = tmpDis;
			minIndex = i;
		}
	}

	//calc the wallPose, find the baseLine 
	cv::Point3f tmpWallPose;
    float minDis1 = 1000.0;
	int minIndex2= -1;

	for (int i = 0; i < tmpRectPoses.size(); i++)
	{
		if (abs(i - minIndex)!=1&&abs(i - minIndex)!=3)
			continue;
        if (fabs(tmpRectPoses[i].x - tmpRectPoses[minIndex].x)<0.05) 
        {
            tmpWallPose.x = tmpRectPoses[i].x;
            tmpWallPose.y = (tmpRectPoses[i].y + tmpRectPoses[minIndex].y) / 2;
        }
        else
        {
            double a = -(tmpRectPoses[i].y - tmpRectPoses[minIndex].y) / (tmpRectPoses[i].x - tmpRectPoses[minIndex].x);
            double b = tmpRectPoses[i].y + tmpRectPoses[i].x * a;
            double m = -robot_pos.x + a * robot_pos.y;
            tmpWallPose.x = (-m + a * b) / (a*a+1);
            tmpWallPose.y = -a * tmpWallPose.x + b;
        }
        
        float dist= sqrt((tmpWallPose.y - robot_pos.y) * (tmpWallPose.y - robot_pos.y) + (tmpWallPose.x - robot_pos.x) * (tmpWallPose.x - robot_pos.x));
        if (dist < minDis1)
        {
            minDis1 = dist;
            minIndex2 = i;
			wallPose.x=tmpWallPose.x;
			wallPose.y=tmpWallPose.y;
        }
        //printf("dist id %f\n",dist);
	}

	/* find base line */
	float tmpPa;
	float minDistAngle = 1000;
	int baseIndex = -1;
	for (int i = 0; i < tmpRectPoses.size(); i++)
	{	
		tmpPa = atan2(tmpRectPoses[(i + 1) % 4].y - tmpRectPoses[i].y, tmpRectPoses[(i + 1) % 4].x - tmpRectPoses[i].x);
		float distAngle = fabs(ClipAngle(tmpPa - Pa));
	//	printf("distangle is %f \n",distAngle);
		if (distAngle < (5) * 3.14159 / 180.0)
		{			
			if (distAngle < minDistAngle)
			{
				minDistAngle = distAngle;
				baseIndex = i;
			}
		}
	}

	minDistAngle = 1000;
	int baseIndex2 = -1;
	for (int i = 0; i < tmpRectPoses.size(); i++)
	{	
		tmpPa = atan2(tmpRectPoses[(i + 1) % 4].y - tmpRectPoses[i].y, tmpRectPoses[(i + 1) % 4].x - tmpRectPoses[i].x);
		float distAngle = fabs(ClipAngle(tmpPa - Pa-M_PI));
	//	printf("distangle is %f \n",distAngle);
		if (distAngle < (5) * 3.14159 / 180.0)
		{			
			if (distAngle < minDistAngle)
			{
				minDistAngle = distAngle;
				baseIndex2 = i;
			}
		}
	}
	float dist1=calDistTline(robot_pos,tmpRectPoses[baseIndex],tmpRectPoses[(baseIndex + 1) % 4]);
	float dist2=calDistTline(robot_pos,tmpRectPoses[baseIndex2],tmpRectPoses[(baseIndex2 + 1) % 4]);
	
	if (baseIndex!=-1||baseIndex2!=-1)
	{
		int i = baseIndex;
		if(dist1>dist2)  
		{
			i=baseIndex2; 
			Pa=Pa+M_PI;
		}
		//printf("find baseLine Point i=%d: (%.4f, %.4f) (%.4f, %.4f)",
		//	baseIndex, tmpRectPoses[i].x, tmpRectPoses[i].y, tmpRectPoses[(i + 1) % 4].x, tmpRectPoses[(i + 1) % 4].y);
		// sort rect poses
		tmpRectPoses.push_back(tmpRectPoses[i]);        // new sort rect
		tmpRectPoses.push_back(tmpRectPoses[(i + 1) % 4]);  // new sort rect
		tmpRectPoses.push_back(tmpRectPoses[(i + 2) % 4]);  // new sort rect
		tmpRectPoses.push_back(tmpRectPoses[(i + 3) % 4]);  // new sort rect

		for (int j = 0; j < 4; j++)
		{
			/* code */
			tmpRectPoses.erase(tmpRectPoses.begin()); // del old rect
		}
	}

	//float maxLen = 80;
	cv::Point3f leftBtmPose = tmpRectPoses[0];
	cv::Point3f rightBtmPose = tmpRectPoses[1];
	cv::Point3f leftTopPose = tmpRectPoses[2];
	cv::Point3f rightTopPose = tmpRectPoses[3];
	float basePa = atan2(rightBtmPose.y - leftBtmPose.y, rightBtmPose.x - leftBtmPose.x);

	float distBase = sqrt((rightBtmPose.y - leftBtmPose.y) * (rightBtmPose.y - leftBtmPose.y) + (rightBtmPose.x - leftBtmPose.x) * (rightBtmPose.x - leftBtmPose.x));
	float iswallFlag = checkIsWall(mapPoints, tmpRectPoses[1], tmpRectPoses[2]);
	float iswallFlag2 = checkIsWall(mapPoints, tmpRectPoses[3], tmpRectPoses[0]);
	float tmpDis1 = sqrt((leftBtmPose.y - robot_pos.y) * (leftBtmPose.y - robot_pos.y) + (leftBtmPose.x - robot_pos.x) * (leftBtmPose.x - robot_pos.x));
	float tmpDis2 = sqrt((rightBtmPose.y - robot_pos.y) * (rightBtmPose.y - robot_pos.y) + (rightBtmPose.x - robot_pos.x) * (rightBtmPose.x - robot_pos.x));
	float baseLen = block_len;
	wallPose.z= Pa;
	
	if (tmpDis1 < tmpDis2)
	{
		rightBtmPose.x = leftBtmPose.x + baseLen * cos(basePa);
		rightBtmPose.y = leftBtmPose.y + baseLen * sin(basePa);
	}
	else 
	{
		leftBtmPose.x = rightBtmPose.x + baseLen * cos(basePa+ M_PI);
		leftBtmPose.y = rightBtmPose.y + baseLen * sin(basePa+ M_PI);
	}

	// update top pose
	rightTopPose.x = rightBtmPose.x + baseLen * cos(basePa + M_PI / 2);
	rightTopPose.y = rightBtmPose.y + baseLen * sin(basePa + M_PI / 2);
	leftTopPose.x = leftBtmPose.x + baseLen * cos(basePa + M_PI / 2);
	leftTopPose.y = leftBtmPose.y + baseLen * sin(basePa + M_PI / 2);


	rightBtmPose.x = rightBtmPose.x - off_set * cos(basePa+ M_PI / 2);
	rightBtmPose.y = rightBtmPose.y - off_set * sin(basePa+ M_PI / 2);
	leftBtmPose.x = leftBtmPose.x - off_set * cos(basePa+ M_PI / 2);
	leftBtmPose.y = leftBtmPose.y - off_set * sin(basePa+ M_PI / 2);
	rightTopPose.x = rightTopPose.x - off_set * cos(basePa + M_PI / 2);
	rightTopPose.y = rightTopPose.y - off_set * sin(basePa + M_PI / 2);
	leftTopPose.x = leftTopPose.x - off_set * cos(basePa + M_PI / 2);
	leftTopPose.y = leftTopPose.y - off_set * sin(basePa + M_PI / 2);

	//ret_poses.push_back(wallPose);
	ret_poses.push_back(leftBtmPose);	
	ret_poses.push_back(rightBtmPose);	
	ret_poses.push_back(rightTopPose);	
	ret_poses.push_back(leftTopPose);	

	//printf("minX: %.4f, minY: %.4f maxX: %.4f maxY: %.4f  minX: %.4f, minY: %.4f maxX: %.4f maxY: %.4f", leftBtmPose.x,leftBtmPose.y, rightBtmPose.x,rightBtmPose.y, rightTopPose.x, rightTopPose.y,leftTopPose.x,leftTopPose.y);

	return ret_poses;
}

std::vector<cv::Point2i> linePoints;
struct Line_Info {
	int start_index;
	int end_index;
	std::vector<cv::Point2i> line_pts;
};

struct wallPos_Info {
	int point_index;
	int lenth;
	float dist2Robot;
	float dist2Block;
	float weight;
};

void cal_avera(std::vector<float>& X, std::vector<float>& Y, float& av_x,float& av_y)
{
	double L_xx, L_yy, L_xy;
	//变量初始化
	av_x = 0; //X的平均值
	av_y = 0; //Y的平均值
	L_xx = 0; //Lxx
	L_yy = 0; //Lyy
	L_xy = 0; //Lxy

	for (size_t i = 0; i < X.size(); i++)
	{
		av_x += X[i];
		av_y += Y[i];
	}
	av_x = av_x / X.size();
	av_y = av_y / X.size();
}

float disToBlock(std::vector<cv::Point2i> ret_poses,cv::Point2i curr_pos)
{
	float minDis = 1000;
	for (int i = 0; i < ret_poses.size(); i++)
	{
		float dist = 0;
		if (fabs(ret_poses[i].x - ret_poses[(i + 1) % 4].x)<1) 
        {
			dist= abs(ret_poses[i].x-curr_pos.x);
		}
		else dist= abs(ret_poses[i].y-curr_pos.y);
			
		if (dist<minDis)
		{
			minDis = dist;
		}	
	}

	return minDis;
}

bool Map_align::getWallPose(std::vector<cv::Point2i> mapPoints,std::vector<cv::Point2i> freePoints,int width,int height,cv::Point2i robot_pos,cv::Point2i& wallPose)
{
	cv::Mat newImg = cv::Mat(cv::Size(width,height), CV_8UC1, cv::Scalar(0));

	
	std::vector<int> xVect;
	std::vector<int> yVect;
	std::vector<cv::Point2i> ret_poses;
	
	//printf("robot_pos is %d  %d \n",robot_pos.x,robot_pos.y);

	for (size_t i = 0; i < mapPoints.size(); i++)
	{
		int y = mapPoints[i].y;
		int x = mapPoints[i].x;
		newImg.at<unsigned char>(x, y) = 100;
		xVect.push_back(y);
		yVect.push_back(x);	
	}

	for (size_t i = 0; i < freePoints.size(); i++)
	{
		int y = freePoints[i].y;
		int x = freePoints[i].x;
		//newImg.at<unsigned char>(x, y) = 150;
	}

	int minY = *std::min_element(yVect.begin(), yVect.end());
	int maxY = *std::max_element(yVect.begin(), yVect.end());
	int minX = *std::min_element(xVect.begin(), xVect.end());
	int maxX = *std::max_element(xVect.begin(), xVect.end());
	cv::Point2i pose;
	/*  (minX, minY)-->(maxX, minY)-->(maxX, maxY)-->(minX, maxY) */
	pose.x = minX;
	pose.y = minY;
	ret_poses.push_back(pose);

	pose.x = maxX;
	pose.y = minY;
	ret_poses.push_back(pose);

	pose.x = maxX;
	pose.y = maxY;
	ret_poses.push_back(pose);

	pose.x = minX;
	pose.y = maxY;
	ret_poses.push_back(pose);

	cv::Mat img;
	newImg.copyTo(img);

	for (int i = 0; i < ret_poses.size(); i++)
	{
		//printf("retpose is %d %d\n",ret_poses[i].x,ret_poses[i].y);
	}

	cv::imwrite("/tmp/map.png",newImg);
	
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(newImg, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	std::vector<cv::Point> contour_points;
	for (int i = 0; i < contours.size();i++)
	{
		if (contours[i].size() < 40)
			continue;
		//cv::drawContours(tmpImg, contours, i, cv::Scalar(30));
		for (int j = 0; j < contours[i].size();j++)
		{
			contour_points.push_back(contours[i][j]);
		}
	}


	Line_Info line_info;
	std::vector<Line_Info> all_lines;
	int start_index = 0;
	int end_index = 0;
	int cnt = 0;
	int cnt1 = 0;
	bool start_flag = false;
	bool end_flag = false;
	//printf("contour_points size is %d  \n",contour_points.size());
	if(contour_points.size()==0)
		return false;
	for (int i = 4; i < contour_points.size()-4;i++)
	{
		if (abs(img.at<unsigned char>(contour_points[i].y, contour_points[i].x)-100)>15)
		{
			if (linePoints.size() > 8)
			{
				end_index=i-1;
				line_info.line_pts = linePoints;
				line_info.end_index = end_index;
				line_info.start_index = start_index;
				all_lines.push_back(line_info);

			}
			cnt = 0;
			cnt1 = 0;
			start_flag = false;
			end_flag = false;
			line_info.line_pts.clear();
			linePoints.clear();
			continue;
		}
		std::vector<float> X2;
		std::vector<float> Y2;
		std::vector<float> X1;
		std::vector<float> Y1;
		for (int j = i - 4; j <= i + 4; j++)
		{
			X2.push_back(contour_points[j].x);
			Y2.push_back(contour_points[j].y);
		}
		for (int j = i - 2; j <= i + 2; j++)
		{
			X1.push_back(contour_points[j].x);
			Y1.push_back(contour_points[j].y);
		}

		float avery1 = 0;
		float averx1 = 0;
		cal_avera(X1, Y1, averx1,avery1);
		float avery2 = 0;
		float averx2 = 0;
		cal_avera(X2, Y2, averx2,avery2);
		float dis_err = sqrt((averx2 - averx1) * (averx2 - averx1) + (avery2 - avery1) * (avery2 - avery1));

		if (dis_err < 0.3)
			cnt++;

		if (cnt>=3)
		{
			start_flag = true;
			if (cnt==3)
			{
				start_index = i;
			}
		}

		if (start_flag&&dis_err >= 0.3)
			cnt1++;

		if (cnt1 >= 2)
		{
			end_flag = true;
			if (cnt1 == 2)
			{
				end_index = i;
			}
		}

		if (end_flag == true)
		{
			if (linePoints.size()>=8)
			{
				line_info.line_pts = linePoints;
				line_info.end_index = end_index;
				line_info.start_index = start_index;
				all_lines.push_back(line_info);

			}
			cnt = 0; cnt1 = 0;
			start_flag = false; end_flag = false;
			linePoints.clear();
			line_info.line_pts.clear();
	
			continue;
		}

		if (start_flag==true)
		{
			linePoints.push_back(contour_points[i]);
		}    
	}

	cv::Mat saveImg = cv::Mat(cv::Size(width,height), CV_8UC1, cv::Scalar(0));
    int validSize= contour_points.size();
	std::vector<wallPos_Info> all_wall_info;
	//printf("all_lines.size() is %d  %d  %d \n",all_lines.size(),robot_pos.x,robot_pos.y);
	for (int j = 0;j < all_lines.size();j++)
	{
		int start_i = all_lines[j].start_index;
		int end_i = all_lines[j].end_index;
		// int unkownCnt = 0;
		// for (int i = start_i-10;i <= start_i-4;i++)
		// {
		// 	if (i>=0&&i<validSize&&abs(img.at<unsigned char>(contour_points[i].y, contour_points[i].x) - 100) > 15)
		// 	{
		// 		unkownCnt++;
		// 	}
		// }

		// for (int i = start_i - 10;i <= start_i - 4;i++)
		// {
		// 	if (i>=0&&i<validSize&&abs(img.at<unsigned char>(contour_points[i].y, contour_points[i].x) - 100) > 15)
		// 	{
		// 		unkownCnt++;
		// 	}
		// }

		// if (unkownCnt>=5)
		// {
		// 	continue;
		// }
		wallPos_Info wall_info;
        float minDis=10000;
		int minIndex=-1;
		for (int i = start_i;i <= end_i;i++)
		{
			saveImg.at<unsigned char>(contour_points[i].y, contour_points[i].x)=180;//

			float dist2Robot = sqrt((contour_points[i].x - robot_pos.x) * (contour_points[i].x - robot_pos.x) + (contour_points[i].y - robot_pos.y) * (contour_points[i].y - robot_pos.y));
			if (dist2Robot<minDis)
			{
				minDis= dist2Robot;
				minIndex= i;
			}
		}
		//printf("111 min %f %d \n",minDis,minIndex);

		GMapping::IntPoint* m_linePoints;
		m_linePoints = new GMapping::IntPoint[20000];
		GMapping::IntPoint p0(robot_pos.x,robot_pos.y);
		GMapping::IntPoint p1(contour_points[minIndex].x,contour_points[minIndex].y);
		GMapping::GridLineTraversalLine line;
		line.points=m_linePoints;
		GMapping::GridLineTraversal::gridLine(p0, p1, &line);
		bool isreachable=true;
		for (int i=0; i<line.num_points-1; i++)
		{
			if (abs(img.at<unsigned char>(m_linePoints[i].y, m_linePoints[i].x)-100)<10)
			{
				isreachable=false;
				break;
			}
				
		}
        float w1=0.1;
		float w2=1;
		float w3=30;
		//if (isreachable==true)
		{
			wall_info.dist2Robot=minDis;
			wall_info.point_index= minIndex;
			wall_info.dist2Block= disToBlock(ret_poses,cv::Point2i(p1.x,p1.y));
			wall_info.lenth= end_i - start_i;
			wall_info.weight= w1*wall_info.dist2Robot+w2*wall_info.dist2Block+w3/wall_info.lenth;
			all_wall_info.push_back(wall_info);
		//	printf("wall_info  %d %d %f  %f  %d  %f \n",p1.x,p1.y,minDis,wall_info.dist2Block,wall_info.lenth,wall_info.weight);
		}	
	}

	int bestIndex=-1;
	int minWeight=1000;

	for (size_t i = 0; i < all_wall_info.size(); i++)
	{
		if (all_wall_info[i].weight<minWeight)
		{
			minWeight=all_wall_info[i].weight;
			bestIndex= all_wall_info[i].point_index;
		}
	}
	
	if(bestIndex!=-1)
	{
		wallPose.x= contour_points[bestIndex].y;
		wallPose.y= contour_points[bestIndex].x;
		//printf("wall_pos is %d  %d \n",wallPose.x,wallPose.y);
		return true;
	}
	
	//cv::imwrite("/tmp/savemap.jpg",saveImg);

	return false;
	
}