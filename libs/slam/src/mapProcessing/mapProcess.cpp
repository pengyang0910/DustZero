#include "mapProcess.h"
#include <chrono>

bool checkIsHorizenStart(cv::Mat img, int x, int y,int wallLen,int jump, std::pair<cv::Point, cv::Point> &wall)
{
	//int lenth
	bool isStart = true;
	for (int i=x- jump;i<x;i++)
	{
		if (img.at<uchar>(y, i)== img.at<uchar>(y, x))
		{
			isStart = false;
			break;
		}
	}

	if (isStart==true)
	{
		int leftIndex = x;
		int rightIndex = x;
		for (int i = x; i < img.cols; i++)
		{
			if (img.at<uchar>(y, i) == img.at<uchar>(y, x))
			{
				leftIndex = i;
			}
			else if (img.at<uchar>(y, i) != img.at<uchar>(y, x))
			{
				if ((i - leftIndex) > jump)
				{
					break;
				}
			}
		}
		int lenth = leftIndex - x;
		if (lenth> wallLen)
		{
			wall.first = cv::Point(x,y);
			wall.second = cv::Point(leftIndex, y);
			return true;
		}
	}

	return false;
}


bool checkIsVerticalStart(cv::Mat img, int x, int y, int wallLen, int jump, std::pair<cv::Point, cv::Point>& wall)
{
	//int lenth
	bool isStart = true;
	for (int j = y - jump; j < y; j++)
	{
		if (img.at<uchar>(j, x) == img.at<uchar>(y, x))
		{
			isStart = false;
			break;
		}
	}

	if (isStart == true)
	{
		int topIndex = y;
		int bottomIndex = y;
		for (int j = y; j < img.rows; j++)
		{
			if (img.at<uchar>(j, x) == img.at<uchar>(y, x))
			{
				topIndex = j;
			}
			else if (img.at<uchar>(j, x) != img.at<uchar>(y, x))
			{
				if ((j - topIndex) > jump)
				{
					break;
				}
			}
		}
		int lenth = topIndex - y;
		if (lenth > wallLen)
		{
			wall.first = cv::Point(x, y);
			wall.second = cv::Point(x, topIndex);
			return true;
		}
	}

	return false;
}

bool checkisValid(cv::Mat img, cv::Point u)
{
	float radius = 2;
	for (int yy = -radius; yy <= radius; yy++)
	{
		for (int xx = -radius; xx <= radius; xx++)
		{
			cv::Point2i newS;
			newS.x = u.x + xx;
			newS.y = u.y + yy;

			if (xx == 0 && yy == 0)
			{
				continue;
			}

			if (newS.x<0||newS.x>=img.cols||newS.y<0||newS.y>=img.rows)
			{
				continue;
			}
			

			if (img.at<uchar>(newS.y, newS.x) == 100)
			{
				return false;
			}
		}
	}

	return true;
}




struct  walls
{
	int dir;
	std::vector<std::pair<cv::Point, cv::Point>> candidateWall;
	std::pair<cv::Point, cv::Point> realWall;
};

void findNextWall(cv::Mat img,std::pair<cv::Point, cv::Point> wall, std::vector<std::pair<cv::Point, cv::Point>> &neiborWalls,std::pair<int,int> &range)
{
	int minX = wall.first.x;
	int maxX= wall.second.x;
	int y = wall.first.y+1;

	neiborWalls.clear();
	neiborWalls.push_back(wall);
	for (size_t j = 0; j < 5; j++)
	{
		//checkIsHorizenStart();
		std::pair<cv::Point, cv::Point> newWall;
		bool iscandidateFlag = false;
		for (size_t i = 0; i < img.cols; i++)
		{
			if (img.at<uchar>(y + j, i) == 100&&i<= maxX)
			{
				bool issucces = checkIsHorizenStart(img, i, y+j, 8, 3, newWall);
				if (issucces&&(newWall.first.x<maxX&& newWall.second.x >minX))
				{
					neiborWalls.push_back(newWall);

					if (newWall.first.x < minX)
						minX = newWall.first.x;

					if (newWall.second.x > maxX)
						maxX = newWall.second.x;
					
					iscandidateFlag = true;
					break;

				}
			}
		}
		if (iscandidateFlag==false)
		{
			break;
		}
	}
	range.first = minX;
	range.second = maxX;
}


void getBestWall(cv::Mat img,std::pair<cv::Point, cv::Point>& bestWall, std::vector<std::pair<cv::Point, cv::Point>> neiborWalls)
{
	int bestIndex = 0;
	int maxPts=0;
	for (size_t i = 0; i < neiborWalls.size(); i++)
	{
		int cnt = 0;
		for (size_t j = neiborWalls[i].first.x; j < neiborWalls[i].second.x; j++)
		{
			if (img.at<uchar>(neiborWalls[i].first.y, j) == 100)
				cnt++;
		}

		if (cnt>maxPts)
		{
			maxPts = cnt;
			bestIndex = i;
		}

	}
	bestWall = neiborWalls[bestIndex];
}

void resetWallPts(cv::Mat &maskImg, cv::Mat & img, std::pair<cv::Point, cv::Point>& bestWall)
{
	
	int clearDir = 1;
	int freeCnt = 0;
	int unkownCnt = 0;
	int freeCnt1 = 0;
	int unkownCnt1 = 0;
	for (int x= bestWall.first.x;x<bestWall.second.x;x++)
	{
		for (int y = bestWall.first.y-2; y < bestWall.second.y; y++)
		{
			if (y<0||y>=img.rows)
			{
				continue;
			}
			
			if (img.at<uchar>(y, x) == 0)
				unkownCnt++;
			if (img.at<uchar>(y, x) == 255)
				freeCnt++;
		}
		for (int y = bestWall.first.y+1; y < bestWall.second.y+3; y++)
		{
			if (img.at<uchar>(y, x) == 0)
				unkownCnt1++;
			if (img.at<uchar>(y, x) == 255)
				freeCnt1++;
		}

		
	}

	if (unkownCnt > unkownCnt1 || freeCnt < freeCnt1)
		clearDir = 1;
	else clearDir = 2;

	for (int x = bestWall.first.x; x <= bestWall.second.x; x++)
	{
		int freeCnt = 0;
		int unkownCnt = 0;
		int freeCnt1 = 0;
		int unkownCnt1 = 0;

		img.at<uchar>(bestWall.first.y, x) = 100;
		maskImg.at<uchar>(bestWall.first.y, x) = 1;

		int obsCnt = 0;
		int obsCntUp = 0;
		int obsCntDown = 0; 
		for (int y = bestWall.first.y - 5; y <= bestWall.second.y + 5; y++)
		{
			if (y<0||y>=img.rows)
			{
				continue;
			}
			
			if (img.at<uchar>(y, x) == 100)
			{
				obsCnt++;
				if (y< bestWall.first.y)
				{
					obsCntUp++;
				}

				if (y > bestWall.first.y)
				{
					obsCntDown++;
				}
			}
				
		}
		if (obsCnt >= 7|| obsCntUp>=5|| obsCntDown>=5)
		{
			//continue;
		}


		for (int y = bestWall.first.y - 2; y < bestWall.second.y; y++)
		{	
			if (y<0||y>=img.rows)
			{
				continue;
			}
			
			if (obsCntUp>=4)
			{
				break;
			}

			
			if (img.at<uchar>(y, x) == 100 && maskImg.at<uchar>(y, x) == 0)
			{
				if (clearDir == 1)
				{
					img.at<uchar>(y, x) = 0;
				}
				else img.at<uchar>(y, x) = 255;
			}

			maskImg.at<uchar>(y, x) = 1;
		}
		for (int y = bestWall.first.y + 1; y < bestWall.second.y + 3; y++)
		{
			if (y<0||y>=img.rows)
			{
				continue;
			}
			
			if (obsCntDown >= 4)
			{
				break;
			}
			
			if (img.at<uchar>(y, x) == 100 && maskImg.at<uchar>(y, x) == 0)
			{
				if (clearDir == 1)
				{
					img.at<uchar>(y, x) = 255;
				}
				else img.at<uchar>(y, x) = 0;
			}

			maskImg.at<uchar>(y, x) = 1;

		}

		img.at<uchar>(bestWall.first.y, x) = 100;
	}

}


void findNextWallVertical(cv::Mat img, std::pair<cv::Point, cv::Point> wall, std::vector<std::pair<cv::Point, cv::Point>>& neiborWalls, std::pair<int, int>& range)
{
	int minY = wall.first.y;
	int maxY = wall.second.y;
	int x = wall.first.x + 1;

	neiborWalls.clear();
	neiborWalls.push_back(wall);
	for (size_t i = 0; i < 4; i++)
	{
		//checkIsHorizenStart();
		std::pair<cv::Point, cv::Point> newWall;
		bool iscandidateFlag = false;
		for (size_t j = 0; j < img.rows; j++)
		{
			if (img.at<uchar>(j, i+x) == 100&&j<=maxY)
			{
				bool issucces = checkIsVerticalStart(img, i+x, j, 10, 3, newWall);
				if (issucces && (newWall.first.y<maxY && newWall.second.y >minY))
				{
					neiborWalls.push_back(newWall);

					if (true)
					{
						if (newWall.first.y < minY)
							minY = newWall.first.y;

						if (newWall.second.y > maxY)
							maxY = newWall.second.y;

					}
					iscandidateFlag = true;

					break;

				}
			}
		}
		if (iscandidateFlag == false)
		{
			break;
		}
	}

	range.first = minY;
	range.second = maxY;
}


void getBestWallVertical(cv::Mat img, std::pair<cv::Point, cv::Point>& bestWall, std::vector<std::pair<cv::Point, cv::Point>> neiborWalls)
{
	int bestIndex = 0;
	int maxPts = 0;
	for (size_t i = 0; i < neiborWalls.size(); i++)
	{
		int cnt = 0;
		for (size_t j = neiborWalls[i].first.y; j < neiborWalls[i].second.y; j++)
		{
			if (img.at<uchar>(j,neiborWalls[i].first.x) == 100)
				cnt++;
		}

		if (cnt > maxPts)
		{
			maxPts = cnt;
			bestIndex = i;
		}

	}
	bestWall = neiborWalls[bestIndex];
}

void resetWallPtsVertical(cv::Mat& maskImg, cv::Mat& img, std::pair<cv::Point, cv::Point>& bestWall)
{
	int clearDir = 1;
	int freeCnt = 0;
	int unkownCnt = 0;
	int freeCnt1 = 0;
	int unkownCnt1 = 0;
	for (int y = bestWall.first.y; y <= bestWall.second.y; y++)
	{

		for (int x = bestWall.first.x - 2; x < bestWall.second.x; x++)
		{
			if (x<0||x>=img.cols)
			{
				continue;
			}

			if (img.at<uchar>(y, x) == 0)
				unkownCnt++;
			if (img.at<uchar>(y, x) == 255)
				freeCnt++;
		}
		for (int x = bestWall.first.x + 1; x < bestWall.second.x + 3; x++)
		{
			if (x<0||x>=img.cols)
			{
				continue;
			}
			
			if (img.at<uchar>(y, x) == 0)
				unkownCnt1++;
			if (img.at<uchar>(y, x) == 255)
				freeCnt1++;
		}


	}

	if (unkownCnt > unkownCnt1 || freeCnt < freeCnt1)
		clearDir = 1;
	else clearDir = 2;

	for (int y = bestWall.first.y; y < bestWall.second.y; y++)
	{
		int freeCnt = 0;
		int unkownCnt = 0;
		int freeCnt1 = 0;
		int unkownCnt1 = 0;

		img.at<uchar>(y,bestWall.first.x) = 100;
		maskImg.at<uchar>(y, bestWall.first.x) = 1;
		int obsCnt = 0;
		int obsCntUp = 0;
		int obsCntDown = 0;
		for (int x = bestWall.first.x - 5; x <= bestWall.second.x+5; x++)
		{
			if (x<0||x>=img.cols)
			{
				continue;
			}
			
			if (img.at<uchar>(y, x) == 100)
			{
				obsCnt++;
				if (x < bestWall.first.x)
				{
					obsCntUp++;
				}

				if (x > bestWall.first.x)
				{
					obsCntDown++;
				}
			}
		}

		if (obsCnt >= 7 || obsCntUp >= 5 || obsCntDown >= 5)
		{
			//continue;
		}

		/*int freeCnt = 0;
		int unkwownCnt = 0;
		for (int x = bestWall.first.x - 1; x > bestWall.first.x-5; x--)
		{
			if (img.at<uchar>(y, x) == 100)
				obsCnt++;
			else if (img.at<uchar>(y, x) == 0)
			{
				unkwownCnt++;
			}
			else if (img.at<uchar>(y, x) == 255)
			{
				freeCnt++;
			}
		}

		for (int x = bestWall.first.x+1; x <= bestWall.second.x + 5; x++)
		{
			if (img.at<uchar>(y, x) == 100)
				obsCnt++;
		}*/

		/*if (obsCnt >= 7)
		{
			continue;

		}*/


		for (int x = bestWall.first.x - 2; x < bestWall.second.x; x++)
		{
			if (x<0||x>=img.cols)
			{
				continue;
			}
			
			if (obsCntUp >= 4)
			{
				break;
			}
			
			if (img.at<uchar>(y, x) == 100&& maskImg.at<uchar>(y, x) ==0)
			{
				if (clearDir == 1)
				{
					img.at<uchar>(y, x) = 0;
				}
				else img.at<uchar>(y, x) = 255;
			}

			maskImg.at<uchar>(y, x) = 1;
		}
		for (int x = bestWall.first.x + 1; x < bestWall.second.x + 3; x++)
		{
			if (x<0||x>=img.cols)
			{
				continue;
			}

			if (obsCntDown >= 4)
			{
				break;
			}
			if (img.at<uchar>(y, x) == 100 && maskImg.at<uchar>(y, x) == 0)
			{
				if (clearDir == 1)
				{
					img.at<uchar>(y, x) = 255;
				}
				else img.at<uchar>(y, x) = 0;
			}

			maskImg.at<uchar>(y, x) = 1;

		}
	}

}


bool check_img(cv::Mat img,int x,int y,uchar value)
{
	if (x<0||x>=img.cols||y<0||y>=img.rows)
	{
		return false;
	}
	
	if (img.at<uchar>(y, x) == value)
		return true;
	else return false;
}


void MapProcess::map_optimize(cv::Mat original_img,cv::Mat &seg_img,cv::Mat &disp_img)
{

	cv::Mat original_img1 = cv::Mat(cv::Size(original_img.cols, original_img.rows), CV_8UC1, cv::Scalar(0));
	for (int y = 0;y < original_img.rows;y++)
	{
		for (int x = 0;x < original_img.cols;x++)
		{
			original_img.at<uchar>(y, x) = original_img.at<uchar>(y, x) & 7;
			if (original_img.at<uchar>(y, x) == 1)
			{
				original_img1.at<uchar>(y, x) = 100;
			}
			else if (original_img.at<uchar>(y, x) == 7)
			{
				original_img1.at<uchar>(y, x) = 255;
			}
			else
			{
				original_img1.at<uchar>(y, x) = 0;
			}
		}
	}

	std::vector<cv::Vec4i> hierarcy;
	for (int y = 0;y < original_img.rows;y++)
		for (int x = 0;x < original_img.cols;x++)
		{
			if (abs(original_img1.at<uchar>(y, x)) ==100)
			{
				if (checkisValid(original_img1, cv::Point(x, y)))
					original_img1.at<uchar>(y, x) = 255;
			}
		}
	
	cv::imwrite("/app/fj212/original_img1.png",original_img1);
	cv::Point anchor(-1, -1); //needed for opencv erode
	int g_nStructElementSize = 1; //结构元素(内核矩阵)的尺寸
    //获取自定义核
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
		cv::Size(2 * g_nStructElementSize + 1, 2 * g_nStructElementSize + 1),
		cv::Point(g_nStructElementSize, g_nStructElementSize));

	cv::Mat after_dilate;
	cv::Mat after_erode;

	// img close 
	erode(original_img1, after_erode, element, cv::Point(-1, -1), 2, cv::BORDER_CONSTANT);
	dilate(after_erode, after_dilate, element, cv::Point(-1, -1), 2, cv::BORDER_CONSTANT);

	cv::Mat contour_map = after_dilate.clone();
	for (int y = 0;y < contour_map.rows;y++)
		for (int x = 0;x < contour_map.cols;x++)
		{
			if (abs(contour_map.at<uchar>(y, x)) == 100)
			{
				contour_map.at<uchar>(y, x) = 0;
			}
		}

	std::vector<std::vector<cv::Point>> contours1;
	//cv::findContours(cv::Mat(contours), contours1, hierarcy, 0, CV_CHAIN_APPROX_NONE);
	cv::findContours(contour_map, contours1, hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	int maxId = 0;
	for (int i = 0;i < contours1.size();i++)
	{
		if (contours1[i].size() > maxId)  maxId = i;
	}

	cv::Mat newRoom = cv::Mat(cv::Size(original_img.cols, original_img.rows), CV_8UC1, cv::Scalar(0));
	for (int i = 0;i < contours1[maxId].size();i++)
	{
		newRoom.at<uchar>(contours1[maxId][i].y, contours1[maxId][i].x) = 255;
	}

	

	cv::drawContours(newRoom, contours1, maxId, cv::Scalar(255), cv::FILLED, 8);
	
	for (int y = 0; y < after_dilate.rows; y++)
	{
		for (int x = 0; x < after_dilate.cols; x++)
		{
			if (after_dilate.at<uchar>(y, x) == 100)
			{
				int ret = cv::pointPolygonTest(contours1[maxId], cv::Point2f(x, y), true);

				if (ret >= -2)
				{
					newRoom.at<uchar>(y, x) = 100;
				}
				else if (after_dilate.at<uchar>(y, x) == 100)
				{
					int ret = cv::pointPolygonTest(contours1[maxId], cv::Point2f(x, y), false);

					if (ret >=0)
					{
						newRoom.at<uchar>(y, x) = 255;
					}
				}
			}
		}
	}


	cv::Mat maskImg = cv::Mat(cv::Size(original_img.cols, original_img.rows), CV_8UC1, cv::Scalar(0));

	int minLen = 8;  //40cm
	int scanRange = 1;
	int countRange = 2;
	int wallLenth = 4;
	int jumpRange = 3;

	std::vector<std::pair<cv::Point, cv::Point>> horizenWalls;

	after_dilate = newRoom.clone();
	// find the wall  
	for (int y = 0;y < original_img.rows;y++)
	{
		for (int x = 0;x < original_img.cols;x++)
		{
			if (after_dilate.at<uchar>(y, x) == 100&& maskImg.at<uchar>(y, x) == 0)
			{
				std::pair<cv::Point, cv::Point> wall;
				bool issucces=checkIsHorizenStart(after_dilate, x, y, minLen, jumpRange,wall);
				
				if (issucces)
				{
					std::vector<std::pair<cv::Point, cv::Point>> neiborWalls;
					std::pair<int, int> range;
					findNextWall(after_dilate,wall, neiborWalls, range);
					//if (neiborWalls.size()>=4)
					{
						std::pair<cv::Point, cv::Point> realWall;
						getBestWall(after_dilate,realWall, neiborWalls);
						horizenWalls.push_back(realWall);
						resetWallPts(maskImg, after_dilate, realWall);
					}
				}
			}
		}
	}

	std::vector<std::pair<cv::Point, cv::Point>> verticalWalls;
	for (int y = 0; y < original_img.rows; y++)
	{
		for (int x = 0; x < original_img.cols; x++)
		{
			if (after_dilate.at<uchar>(y, x) == 100 && maskImg.at<uchar>(y, x) == 0)
			{
				std::pair<cv::Point, cv::Point> wall;
				bool issucces = checkIsVerticalStart(after_dilate, x, y, minLen, jumpRange, wall);

				if (issucces)
				{
					std::vector<std::pair<cv::Point, cv::Point>> neiborWalls;
					//verticalWalls.push_back(wall);
					std::pair<int, int> range;
					findNextWallVertical(after_dilate, wall, neiborWalls,range);
					//if (neiborWalls.size() >= 4)
					{
						std::pair<cv::Point, cv::Point> realWall;
						getBestWallVertical(after_dilate, realWall, neiborWalls);
						verticalWalls.push_back(realWall);
						resetWallPtsVertical(maskImg, after_dilate, realWall);
					}
				}
			}
		}
	}


	cv::Mat labelImg = cv::Mat(cv::Size(original_img.cols, original_img.rows), CV_8UC1, cv::Scalar(0));

	for (size_t i = 0; i < horizenWalls.size(); i++)
	{
		int y = horizenWalls[i].first.y;
		std::vector<cv::Point>  labelPts;
		for (size_t j = horizenWalls[i].first.x-1; j >= horizenWalls[i].first.x-4; j--)
		{
			labelImg.at<uchar>(y, j) = 1;
			if (after_dilate.at<uchar>(y, j) == 100)
			{
				for (size_t k = 0; k < labelPts.size(); k++)
				{
					after_dilate.at<uchar>(labelPts[k].y, labelPts[k].x) = 100;
				}
				break;
			}

			labelPts.push_back(cv::Point(j,y));
		}

		labelPts.clear();
		for (size_t j = horizenWalls[i].second.x + 1; j <= horizenWalls[i].second.x + 4; j++)
		{
			labelImg.at<uchar>(y, j) = 1;
			if (after_dilate.at<uchar>(y, j) == 100)
			{
				for (size_t k = 0; k < labelPts.size(); k++)
				{
					after_dilate.at<uchar>(labelPts[k].y, labelPts[k].x) = 100;
				}
				break;
			}

			labelPts.push_back(cv::Point(j, y));
		}
	}

	for (size_t i = 0; i < verticalWalls.size(); i++)
	{
		int x = verticalWalls[i].first.x;
		std::vector<cv::Point>  labelPts;
		for (size_t j = verticalWalls[i].first.y - 1; j >= verticalWalls[i].first.y - 4; j--)
		{
			labelImg.at<uchar>(j, x) = 1;
			if (after_dilate.at<uchar>(j, x) == 100)
			{
				for (size_t k = 0; k < labelPts.size(); k++)
				{
					after_dilate.at<uchar>(labelPts[k].y, labelPts[k].x) = 100;
				}
				break;
			}

			labelPts.push_back(cv::Point(x, j));
		}

		labelPts.clear();
		for (size_t j = verticalWalls[i].second.y + 1; j <= verticalWalls[i].second.y + 4; j++)
		{
			labelImg.at<uchar>(j, x) = 1;
			if (after_dilate.at<uchar>(j, x) == 100)
			{
				for (size_t k = 0; k < labelPts.size(); k++)
				{
					after_dilate.at<uchar>(labelPts[k].y, labelPts[k].x) = 100;
				}
				break;
			}

			labelPts.push_back(cv::Point(x, j));
		}
	}

	for (size_t i = 0; i < horizenWalls.size(); i++)
	{
		int x = horizenWalls[i].first.x;
		std::vector<cv::Point>  labelPts;
		for (size_t j = horizenWalls[i].first.y - 1; j >= horizenWalls[i].first.y - 4; j--)
		{
			if (labelImg.at<uchar>(j, x) == 1)
			{
				for (size_t k = 0; k < labelPts.size(); k++)
				{
					after_dilate.at<uchar>(labelPts[k].y, labelPts[k].x) = 100;
				}
				break;
			}

			labelPts.push_back(cv::Point(x, j));
		}

		labelPts.clear();
		for (size_t j = horizenWalls[i].second.y + 1; j <= horizenWalls[i].second.y + 4; j++)
		{
			if (labelImg.at<uchar>(j, x) == 1)
			{
				for (size_t k = 0; k < labelPts.size(); k++)
				{
					after_dilate.at<uchar>(labelPts[k].y, labelPts[k].x) = 100;
				}
				break;
			}

			labelPts.push_back(cv::Point(x, j));
		}
	}

	for (size_t i = 0; i < verticalWalls.size(); i++)
	{
		int y = verticalWalls[i].first.y;
		std::vector<cv::Point>  labelPts;
		for (size_t j = verticalWalls[i].first.x - 1; j >= verticalWalls[i].first.x - 3; j--)
		{
			if (labelImg.at<uchar>(y, j) == 1)
			{
				for (size_t k = 0; k < labelPts.size(); k++)
				{
					after_dilate.at<uchar>(labelPts[k].y, labelPts[k].x) = 100;
				}
				break;
			}

			labelPts.push_back(cv::Point(j, y));
		}

		labelPts.clear();
		for (size_t j = verticalWalls[i].second.x + 1; j <= verticalWalls[i].second.x + 4; j++)
		{
			if (labelImg.at<uchar>(y, j) == 1)
			{
				for (size_t k = 0; k < labelPts.size(); k++)
				{
					after_dilate.at<uchar>(labelPts[k].y, labelPts[k].x) = 100;
				}
				break;
			}

			labelPts.push_back(cv::Point(j, y));
		}
	}

	cv::Mat contour_map1 = after_dilate.clone();
	for (int y = 0; y < contour_map1.rows; y++)
		for (int x = 0; x < contour_map1.cols; x++)
		{
			if (abs(contour_map1.at<uchar>(y, x)) == 100)
			{
				contour_map1.at<uchar>(y, x) = 0;
			}
		}


	//cv::findContours(cv::Mat(contours), contours1, hierarcy, 0, CV_CHAIN_APPROX_NONE);
	std::vector<std::vector<cv::Point>> contours2;
	cv::findContours(contour_map1, contours2, hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	maxId = 0;
	for (int i = 0; i < contours2.size(); i++)
	{
		if (contours2[i].size() > maxId)  maxId = i;
	}

	cv::Mat newRoom1 = cv::Mat(cv::Size(original_img.cols, original_img.rows), CV_8UC1, cv::Scalar(0));
	for (int i = 0;i < contours2[maxId].size();i++)
	{
		//newRoom1.at<uchar>(contours2[maxId][i].y, contours2[maxId][i].x) = 255;
	}
	
	cv::drawContours(newRoom1, contours2, maxId, cv::Scalar(255), cv::FILLED, 8);

	seg_img = newRoom1.clone();
	disp_img= after_dilate.clone();

	cv::imwrite("/app/fj212/seg_img.png",seg_img);

	printf("optimize done \n");

}


void MapProcess::map_optimize_new(cv::Mat original_img, cv::Mat& seg_img, cv::Mat& filter_img,cv::Mat& disp_img)
{

	std::chrono::high_resolution_clock::time_point st00 = std::chrono::high_resolution_clock::now();
	cv::Mat original_img1 = cv::Mat(cv::Size(original_img.cols, original_img.rows), CV_8UC1, cv::Scalar(0));
	cv::Mat original_img2 = cv::Mat(cv::Size(original_img.cols, original_img.rows), CV_8UC1, cv::Scalar(0));
	for (int y = 0; y < original_img.rows; y++)
	{
		for (int x = 0; x < original_img.cols; x++)
		{
			original_img.at<uchar>(y, x) = original_img.at<uchar>(y, x) & 7;
			if (original_img.at<uchar>(y, x) == 1)
			{
				original_img1.at<uchar>(y, x) = 100;
			}
			else if (original_img.at<uchar>(y, x) == 7)
			{
				original_img1.at<uchar>(y, x) = 255;
				original_img2.at<uchar>(y, x) = 255;
			}
		}
	}

	std::vector<std::vector<cv::Point>> contours0;
	std::vector<cv::Vec4i> hierarcy;
	cv::findContours(original_img2, contours0, hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	int maxId = 0;
	int maxSize=0;
	for (int i = 0; i < contours0.size(); i++)
	{
		if (contours0[i].size() > maxSize)  
		{
			maxId = i;
			maxSize = contours0[i].size();
		}
	}

	std::vector<int> xVect;
	std::vector<int> yVect;

    for (size_t i = 0; i < contours0[maxId].size(); i++)
    {
        xVect.push_back(contours0[maxId][i].x);
        yVect.push_back(contours0[maxId][i].y);
	}

	int minY = *std::min_element(yVect.begin(), yVect.end());
    int maxY = *std::max_element(yVect.begin(), yVect.end());
    int minX = *std::min_element(xVect.begin(), xVect.end());
    int maxX = *std::max_element(xVect.begin(), xVect.end());

	cv::Mat img3 = cv::Mat(cv::Size(original_img.cols, original_img.rows), CV_8UC1, cv::Scalar(0));
	cv::drawContours(img3, contours0, maxId, cv::Scalar(100), cv::FILLED);   

	for (int y = minY; y < maxY; y++)
	{
		for (int x = minX; x < maxX; x++)
		{
			if (original_img1.at<uchar>(y, x) == 0)
			{
				/*int ret = cv::pointPolygonTest(contours0[maxId], cv::Point2f(x, y), false);

				if (ret > 0)
				{
					original_img1.at<uchar>(y, x) = 255;
				} */

				if (img3.at<uchar>(y, x)==100)
				{
			//		original_img1.at<uchar>(y, x) = 255;
				}
			}
		}
	}

	filter_img = original_img1.clone();
	
	for (int y = 0; y < original_img.rows; y++)
		for (int x = 0; x < original_img.cols; x++)
		{
			if (abs(original_img1.at<uchar>(y, x)) == 100)
			{
				if (checkisValid(original_img1, cv::Point(x, y)))
					original_img1.at<uchar>(y, x) = 255;
			}
		}

	cv::imwrite("/app/fj212/original_img1.png", original_img1);
	std::chrono::high_resolution_clock::time_point st001 = std::chrono::high_resolution_clock::now();
	cv::Point anchor(-1, -1); //needed for opencv erode
	int g_nStructElementSize = 1; //结构元素(内核矩阵)的尺寸
	//获取自定义核
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
		cv::Size(2 * g_nStructElementSize + 1, 2 * g_nStructElementSize + 1),
		cv::Point(g_nStructElementSize, g_nStructElementSize));

	cv::Mat after_dilate;
	cv::Mat after_erode;

	// img close 
	erode(original_img1, after_erode, element, cv::Point(-1, -1), 2, cv::BORDER_CONSTANT);
	dilate(after_erode, after_dilate, element, cv::Point(-1, -1), 2, cv::BORDER_CONSTANT);
	
	cv::Mat contour_map = after_dilate.clone();
	for (int y = 0; y < contour_map.rows; y++)
		for (int x = 0; x < contour_map.cols; x++)
		{
			if (abs(contour_map.at<uchar>(y, x)) == 100)
			{
				contour_map.at<uchar>(y, x) = 0;
			}
		}

	std::vector<std::vector<cv::Point>> contours1;
	//cv::imwrite("contour_map.jpg", contour_map);
	//cv::findContours(cv::Mat(contours), contours1, hierarcy, 0, CV_CHAIN_APPROX_NONE);
	cv::findContours(contour_map, contours1, hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	maxId = 0;
	maxSize=0;
	for (int i = 0; i < contours1.size(); i++)
	{
		if (contours1[i].size() > maxSize)  
		{
			maxId = i;
			maxSize = contours1[i].size();
		}
	}
	//printf("maxid size is %d  %d \n",maxId,contours1[maxId].size());

	cv::Mat newRoom = cv::Mat(cv::Size(original_img.cols, original_img.rows), CV_8UC1, cv::Scalar(0));
	for (int i = 0; i < contours1[maxId].size(); i++)
	{
		newRoom.at<uchar>(contours1[maxId][i].y, contours1[maxId][i].x) = 255;
	}
	
	cv::drawContours(newRoom, contours1, maxId, cv::Scalar(255), cv::FILLED, 8);
	//cv::drawContours(img3, contours1, maxId, cv::Scalar(200), cv::FILLED);   
	//cv::imwrite("newRoom.jpg", newRoom);

	for (int y = 0; y < after_dilate.rows; y++)
	{
		for (int x = 0; x < after_dilate.cols; x++)
		{
			if (after_dilate.at<uchar>(y, x) == 100)
			{
				int ret = cv::pointPolygonTest(contours1[maxId], cv::Point2f(x, y), true);

				if (ret >= -2)
				{
					newRoom.at<uchar>(y, x) = 100;
				}
			}
		}
	}

	std::chrono::high_resolution_clock::time_point st01 = std::chrono::high_resolution_clock::now();


	cv::Mat maskImg = cv::Mat(cv::Size(original_img.cols, original_img.rows), CV_8UC1, cv::Scalar(0));
	cv::Mat maskImgH = cv::Mat(cv::Size(original_img.cols, original_img.rows), CV_8UC1, cv::Scalar(0));
	cv::Mat maskImgV = cv::Mat(cv::Size(original_img.cols, original_img.rows), CV_8UC1, cv::Scalar(0));

	int minLen = 8;  //40cm
	int scanRange = 1;
	int countRange = 2;
	int wallLenth = 4;
	int jumpRange = 3;

	std::vector<std::pair<cv::Point, cv::Point>> horizenWalls;

	
	after_dilate = newRoom.clone();
	// find the wall  
	for (int y = 0; y < original_img.rows; y++)
	{
		for (int x = 0; x < original_img.cols; x++)
		{
			if (after_dilate.at<uchar>(y, x) == 100 && maskImgH.at<uchar>(y, x) == 0)
			{
				std::pair<cv::Point, cv::Point> wall;
				bool issucces = checkIsHorizenStart(after_dilate, x, y, minLen, jumpRange, wall);

				if (issucces)
				{
					std::vector<std::pair<cv::Point, cv::Point>> neiborWalls;
					std::pair<int, int> range;
					findNextWall(after_dilate, wall, neiborWalls,range);
					//if (neiborWalls.size()>=4)
					{
						std::pair<cv::Point, cv::Point> realWall;
						getBestWall(after_dilate, realWall, neiborWalls);
						realWall.first.x= range.first;
						realWall.second.x= range.second;
						horizenWalls.push_back(realWall);
						for (size_t i = 0; i < neiborWalls.size(); i++)
						{
							for (size_t j = range.first; j < range.second; j++)
							{
								maskImgH.at<uchar>(neiborWalls[i].first.y, j) = 1;
							}
						}
						resetWallPts(maskImg, after_dilate, realWall);
					}
				}
			}
		}
	}

	std::vector<std::pair<cv::Point, cv::Point>> verticalWalls;
	/* for (int y = 0; y < original_img.rows; y++)
	{
		for (int x = 0; x < original_img.cols; x++)
		{
			if (after_dilate.at<uchar>(y, x) == 100 && maskImgV.at<uchar>(y, x) == 0)
			{
				std::pair<cv::Point, cv::Point> wall;
				bool issucces = checkIsVerticalStart(after_dilate, x, y, minLen, jumpRange, wall);

				if (issucces)
				{
					std::vector<std::pair<cv::Point, cv::Point>> neiborWalls;
					//verticalWalls.push_back(wall);
					std::pair<int, int> range;
					findNextWallVertical(after_dilate, wall, neiborWalls,range);
					//if (neiborWalls.size() >= 4)
					{
						std::pair<cv::Point, cv::Point> realWall;
						getBestWallVertical(after_dilate, realWall, neiborWalls);
						for (size_t i = 0; i < neiborWalls.size(); i++)
						{
							for (size_t j = range.first; j < range.second; j++)
							{
								maskImgV.at<uchar>(j,neiborWalls[i].first.x) = 1;
							}
						}
						realWall.first.y= range.first;
						realWall.second.y= range.second;
						verticalWalls.push_back(realWall);
						resetWallPtsVertical(maskImg, after_dilate, realWall);
					}
				}
			}
		}
	} */

	std::chrono::high_resolution_clock::time_point st02 = std::chrono::high_resolution_clock::now();
	//cv::imwrite("newRoom.jpg", after_dilate);
	//  connect process
	for (size_t i = 0; i < verticalWalls.size(); i++)
	{
		int x = verticalWalls[i].first.x;
		int minY=verticalWalls[i].second.y;
		int maxY=0; 
		std::vector<cv::Point>  labelPts;
		for (size_t j = verticalWalls[i].first.y; j <= verticalWalls[i].second.y; j++)
		{
			if (after_dilate.at<uchar>(j, x) == 100)
			{
				if (j<minY)
				{
					minY=j;
				}

				if (j>maxY)
				{
					maxY=j;
				}
			}
		}
		verticalWalls[i].first.y= minY;
		verticalWalls[i].second.y= maxY;
	}
	for (size_t i = 0; i < horizenWalls.size(); i++)
	{
		int y = horizenWalls[i].first.y;
		int minX=horizenWalls[i].second.x;
		int maxX=0; 
		std::vector<cv::Point>  labelPts;
		for (size_t j = horizenWalls[i].first.x; j <= horizenWalls[i].second.x; j++)
		{
			if (after_dilate.at<uchar>(y, j) == 100)
			{
				if (j<minX)
				{
					minX=j;
				}

				if (j>maxX)
				{
					maxX=j;
				}
			}
		}
		horizenWalls[i].first.x= minX;
		horizenWalls[i].second.x= maxX;
	}

	cv::Mat labelImg = cv::Mat(cv::Size(original_img.cols, original_img.rows), CV_8UC1, cv::Scalar(0));
	for (size_t i = 0; i < horizenWalls.size(); i++)
	{
		int y = horizenWalls[i].first.y;
		std::vector<cv::Point>  labelPts;
		for (signed int  j = horizenWalls[i].first.x - 1; j >= horizenWalls[i].first.x - 4 && j>=0; j--)
		{
			labelImg.at<uchar>(y, j) = 1;
			if (after_dilate.at<uchar>(y, j) == 100)
			{
				for (size_t k = 0; k < labelPts.size(); k++)
				{
					after_dilate.at<uchar>(labelPts[k].y, labelPts[k].x) = 100;
				}
				break;
			}

			labelPts.push_back(cv::Point(j, y));
		}

		labelPts.clear();
		for (size_t j = horizenWalls[i].second.x + 1; j <= horizenWalls[i].second.x + 4; j++)
		{
			labelImg.at<uchar>(y, j) = 1;
			if (j<0||j>=after_dilate.cols)
			{
				continue;
			}
			if (after_dilate.at<uchar>(y, j) == 100)
			{
				for (size_t k = 0; k < labelPts.size(); k++)
				{
					after_dilate.at<uchar>(labelPts[k].y, labelPts[k].x) = 100;
				}
				break;
			}

			labelPts.push_back(cv::Point(j, y));
		}
	}

	for (size_t i = 0; i < verticalWalls.size(); i++)
	{
		int x = verticalWalls[i].first.x;
		std::vector<cv::Point>  labelPts;
		for (signed int  j = verticalWalls[i].first.y - 1; j >= verticalWalls[i].first.y - 4 && j>=0; j--)
		{
			if (j<0||j>=after_dilate.rows)
			{
				continue;
			}

			labelImg.at<uchar>(j, x) = 1;
			if (after_dilate.at<uchar>(j, x) == 100)
			{
				for (size_t k = 0; k < labelPts.size(); k++)
				{
					after_dilate.at<uchar>(labelPts[k].y, labelPts[k].x) = 100;
				}
				break;
			}

			labelPts.push_back(cv::Point(x, j));
		}

		labelPts.clear();
		for (size_t j = verticalWalls[i].second.y + 1; j <= verticalWalls[i].second.y + 4; j++)
		{
			if (j<0||j>=after_dilate.rows)
			{
				continue;
			}

			labelImg.at<uchar>(j, x) = 1;
			if (after_dilate.at<uchar>(j, x) == 100)
			{
				for (size_t k = 0; k < labelPts.size(); k++)
				{
					after_dilate.at<uchar>(labelPts[k].y, labelPts[k].x) = 100;
				}
				break;
			}

			labelPts.push_back(cv::Point(x, j));
		}
	}

	
	for (size_t i = 0; i < horizenWalls.size(); i++)
	{
		int x = horizenWalls[i].first.x;
		std::vector<cv::Point>  labelPts;
		for (signed int  j = horizenWalls[i].first.y - 1; j >= horizenWalls[i].first.y - 3 && j>=0; j--)
		{
			if (j>=after_dilate.rows)
			{
				continue;
			}

			if (labelImg.at<uchar>(j, x) == 1|| after_dilate.at<uchar>(j, x) == 100)
			{
				for (size_t k = 0; k < labelPts.size(); k++)
				{
					after_dilate.at<uchar>(labelPts[k].y, labelPts[k].x) = 100;
				}
				break;
			}

			labelPts.push_back(cv::Point(x, j));
		}

		labelPts.clear();
		for (size_t j = horizenWalls[i].second.y + 1; j <= horizenWalls[i].second.y + 3; j++)
		{
			if (j>=after_dilate.rows)
			{
				continue;
			}

			if (labelImg.at<uchar>(j, x) == 1||after_dilate.at<uchar>(j, x) == 100)
			{
				for (size_t k = 0; k < labelPts.size(); k++)
				{
					after_dilate.at<uchar>(labelPts[k].y, labelPts[k].x) = 100;
				}
				break;
			}

			labelPts.push_back(cv::Point(x, j));
		}
	}

	for (size_t i = 0; i < verticalWalls.size(); i++)
	{
		int y = verticalWalls[i].first.y;
		std::vector<cv::Point>  labelPts;
		for (signed int  j = verticalWalls[i].first.x - 1; j >= verticalWalls[i].first.x - 3 && j>=0; j--)
		{
			if (j>=after_dilate.cols)
			{
				continue;
			}

			if(j<0)  break;

			if (labelImg.at<uchar>(y, j) == 1||after_dilate.at<uchar>(y, j) == 100)
			{
				after_dilate.at<uchar>(y, j) = 100;
				for (size_t k = 0; k < labelPts.size(); k++)
				{
					after_dilate.at<uchar>(labelPts[k].y, labelPts[k].x) = 100;
				}
				break;
			}

			labelPts.push_back(cv::Point(j, y));
		}

		labelPts.clear();
		for (size_t j = verticalWalls[i].second.x + 1; j <= verticalWalls[i].second.x + 3; j++)
		{
			if (j>=after_dilate.cols)
			{
				continue;
			}
			
			if (labelImg.at<uchar>(y, j) == 1 || after_dilate.at<uchar>(y, j) == 100)
			{
				after_dilate.at<uchar>(y, j) = 100;
				for (size_t k = 0; k < labelPts.size(); k++)
				{
					after_dilate.at<uchar>(labelPts[k].y, labelPts[k].x) = 100;
				}
				break;
			}

			labelPts.push_back(cv::Point(j, y));
		}

		y = verticalWalls[i].second.y;
		labelPts.clear();
		for (signed int j = verticalWalls[i].first.x - 1; j >= verticalWalls[i].first.x - 3 && j>=0; j--)
		{
			if (j>=after_dilate.cols)
			{
				continue;
			}

			if(j<0) break;

			if (labelImg.at<uchar>(y, j) == 1||after_dilate.at<uchar>(y, j) == 100)
			{
				after_dilate.at<uchar>(y, j) = 100;
				for (size_t k = 0; k < labelPts.size(); k++)
				{
					after_dilate.at<uchar>(labelPts[k].y, labelPts[k].x) = 100;
				}
				break;
			}

			labelPts.push_back(cv::Point(j, y));
		}

		labelPts.clear();
		for (size_t j = verticalWalls[i].second.x + 1; j <= verticalWalls[i].second.x + 3; j++)
		{
			if (j>=after_dilate.cols)
			{
				continue;
			}
			
			if (labelImg.at<uchar>(y, j) == 1 || after_dilate.at<uchar>(y, j) == 100)
			{
				after_dilate.at<uchar>(y, j) = 100;
				for (size_t k = 0; k < labelPts.size(); k++)
				{
					after_dilate.at<uchar>(labelPts[k].y, labelPts[k].x) = 100;
				}
				break;
			}

			labelPts.push_back(cv::Point(j, y));
		}

	}
	cv::imwrite("/app/fj212/newRoom1.png", after_dilate);
	//printf("111111111\n");
	std::chrono::high_resolution_clock::time_point st03 = std::chrono::high_resolution_clock::now();

	cv::Mat contour_map1 = after_dilate.clone();
	for (int y = 0; y < contour_map1.rows; y++)
		for (int x = 0; x < contour_map1.cols; x++)
		{
			if (abs(contour_map1.at<uchar>(y, x)) == 100)
			{
				contour_map1.at<uchar>(y, x) = 0;
			}
		}


	//cv::findContours(cv::Mat(contours), contours1, hierarcy, 0, CV_CHAIN_APPROX_NONE);
	std::vector<std::vector<cv::Point>> contours2;
	cv::findContours(contour_map1, contours2, hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	maxId = 0;
	maxSize=0;
	for (int i = 0; i < contours2.size(); i++)
	{
		if (contours2[i].size() > maxSize)  
		{
			maxId = i;
			maxSize = contours2[i].size();
		}
	}

	cv::Mat newRoom1 = cv::Mat(cv::Size(original_img.cols, original_img.rows), CV_8UC1, cv::Scalar(0));
	for (int i = 0; i < contours2[maxId].size(); i++)
	{
		//newRoom1.at<uchar>(contours2[maxId][i].y, contours2[maxId][i].x) = 255;
	}

	cv::drawContours(newRoom1, contours2, maxId, cv::Scalar(255), cv::FILLED, 8);

	std::chrono::high_resolution_clock::time_point st04 = std::chrono::high_resolution_clock::now();

	
	double diff_t0 = std::chrono::duration_cast<std::chrono::duration<double>>(st001 - st00).count() * 1000;
	double diff_t1 = std::chrono::duration_cast<std::chrono::duration<double>>(st01 - st00).count() * 1000;
	double diff_t2 = std::chrono::duration_cast<std::chrono::duration<double>>(st02 - st01).count() * 1000;
	double diff_t3 = std::chrono::duration_cast<std::chrono::duration<double>>(st03 - st02).count() * 1000;
	double diff_t4 = std::chrono::duration_cast<std::chrono::duration<double>>(st04 - st03).count() * 1000;

	printf("diff_t1 is %f  %f %f %f %f \n",diff_t0,diff_t1,diff_t2,diff_t3,diff_t4);

	seg_img = newRoom1.clone();
	disp_img = after_dilate.clone();

	cv::imwrite("/app/fj212/seg_img.png", seg_img);
	cv::imwrite("/app/fj212/disp_img.png", disp_img);

	printf("optimize done \n");

}
