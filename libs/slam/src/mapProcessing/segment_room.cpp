#include "segment_room.h"

#include <iostream>
#include <chrono>
#include "opencv2/opencv.hpp"
//#include "meanshift2d.h"
#include "voronoi_segmentation.h"
#include "distance_segmentation.h"
#include "morphological_segmentation.h"
// include "adaboost_classifier.h"

//#include "Socket/tlv.h"
#include "segment_room.h"
#include "gmapping/scanmatcher/gridlinetraversal.h"

std::vector<cv::Point> segment_map::expand_polygon1(std::vector<cv::Point> trajectoryPoint, const double offset_dist, const double direction)
{
	// return trajectoryPoint;

	const double &L = offset_dist;
	std::vector<cv::Point> newpoint;
	size_t len = trajectoryPoint.size();
	if (len < 3)
	{
		printf("room point < 3\r\n");
		return trajectoryPoint;
	}
	cv::Point2f xim1, xip1;
	for (size_t i = 0; i < len; i++)
	{
		if (i == 0)
		{
			xim1 = trajectoryPoint[len - 1];
		}
		else
		{
			xim1 = trajectoryPoint[i - 1];
		}

		cv::Point2f xi = trajectoryPoint[i];
		if (i == len - 1)
		{
			xip1 = trajectoryPoint[0];
		}
		else
		{
			xip1 = trajectoryPoint[i + 1];
		}

		const cv::Point2f &v1 = xi - xim1;
		const cv::Point2f &v2 = xi - xip1;
		double x1 = v1.x;
		double y1 = v1.y;
		double x2 = v2.x;
		double y2 = v2.y;

		double dot = x1 * x2 + y1 * y2; // dot product between [x1, y1] and [x2, y2]
		double det = x1 * y2 - y1 * x2; // determinant
		double theta = atan2(det, dot); // atan2(y, x) or atan2(sin, cos)
		if (theta == M_PI || theta == 0)
		{
			printf("this point is  discarded becasue it is on the line constructed by last and next point\n");
			continue;
		}

		double nv1 = norm(v1);
		double nv2 = norm(v2);
		cv::Point2f unit_v1(v1.x / nv1, v1.y / nv1);
		cv::Point2f unit_v2(v2.x / nv2, v2.y / nv2);

		double l = L / sin(theta);
		l = fabs(l) * direction;

		if (acos(dot / (norm(v1) * norm(v2))) < 0.15)
		{
			xi = xi + offset_dist * (unit_v1 + unit_v2);
		}
		else
		{
			if (theta > 0)
			{
				l = -l;
				xi = xi + l * unit_v1 + l * unit_v2;
			}
			else
			{
				xi = xi + l * unit_v1 + l * unit_v2;
			}
		}

		newpoint.push_back(xi);
	}
	return newpoint;
}

bool segment_map::PointToline(cv::Point2i const &a, cv::Point2i const &b, cv::Point2i const &p,cv::Point2i& d,float &dist)
{
	float ap_ab = (b.x - a.x) * (p.x - a.x) + (b.y - a.y) * (p.y - a.y); // cross( a , p , b );
	if (ap_ab <= 0)
		return false;

	float d2 = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
	if (ap_ab >= d2)
		return false;

	float r = ap_ab / d2;
	float px = a.x + (b.x - a.x) * r;
	float py = a.y + (b.y - a.y) * r;
	d.x= px;
	d.y= py;

	dist = sqrt((p.x - px) * (p.x - px) + (p.y - py) * (p.y - py));
	return true;
}


bool segment_map::PointTolinef( cv::Point2f const&a,cv::Point2f const&b,cv::Point2f const&p,cv::Point2f &d,float &dist)
{
	float ap_ab = (b.x - a.x) * (p.x - a.x) + (b.y - a.y) * (p.y - a.y); // cross( a , p , b );
	if (ap_ab <= 0)
		return false;

	float d2 = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
	if (ap_ab >= d2)
		return false;

	float r = ap_ab / d2;
	float px = a.x + (b.x - a.x) * r;
	float py = a.y + (b.y - a.y) * r;
	d.x= px;
	d.y= py;

	dist = sqrt((p.x - px) * (p.x - px) + (p.y - py) * (p.y - py));
	return true;
}
bool checkPtValid(cv::Mat img, cv::Point u)
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

			if (newS.x < 0 || newS.y < 0 || newS.y > img.rows || newS.x > img.cols)
			{
				// printf("checkPtValid out of img  %d  %d \n",newS.x,newS.y);
				continue;
			}

			if (img.at<uchar>(newS.y, newS.x) == 0)
			{
				return false;
			}
		}
	}
	return true;
}



bool getNextBlkEntry(int curId, int dstId, NavMsg::BlockArray_t m_blockArrays, NavMsg::Pose_t &pt, std::vector<bool> &visit)
{
	bool ret = false;
	if ((curId > m_blockArrays.blks.size()) || (dstId > m_blockArrays.blks.size()))
		return false;

	visit[curId - 1] = true;
	//printf("entryNum IS %d   %d  %d \n", m_blockArrays.blks[curId - 1].entryNum, curId, dstId);

	for (size_t j = 0; j < m_blockArrays.blks[dstId - 1].entryNum; j++)
	{
		if (m_blockArrays.blks[dstId - 1].nextBlkIdOfEntries[j] == curId)
		{
			pt = m_blockArrays.blks[dstId - 1].entryPoses[j];
			printf("dstId entryPose is %d %f %f \n", dstId,pt.px, pt.py);
			return true;
		}
	}

	for (int i = 0; i < m_blockArrays.blks[curId - 1].entryNum; i++)
	{
		if (visit[m_blockArrays.blks[curId - 1].nextBlkIdOfEntries[i] - 1] == false)
		{
			ret = getNextBlkEntry(m_blockArrays.blks[curId - 1].nextBlkIdOfEntries[i], dstId, m_blockArrays, pt, visit);
			if (ret)
			{
				return true;
			}			
		}
	}

	return ret;
}

segment_map::segment_map()
{
	// if(xlog ==NULL)
	{
		xlog = new XLog(false);
	}
}

segment_map::~segment_map()
{
	if (NULL != xlog)
		delete xlog;
}

void segment_map::check_valid_Point(cv::Mat &img, int x, int y)
{
	int radius = 2;
	int obsCnt = 0;
	for (int yy = -radius; yy <= radius; yy++)
	{
		for (int xx = -radius; xx <= radius; xx++)
		{
			cv::Point2i newS;
			newS.x = x + xx;
			newS.y = y + yy;

			if (xx == 0 && yy == 0)
			{
				continue;
			}

			if (newS.x < 0 || newS.x >= img.cols || newS.y < 0 || newS.y >= img.rows)
			{
				continue;
			}

			if (img.at<uchar>(newS.y, newS.x) == 100)
			{
				obsCnt++;
			}
		}
	}

	// printf("obsCnt is  %d \n",obsCnt);
	if (obsCnt <= 1)
	{
		img.at<uchar>(y, x) = 255;
		xlog->Info("check pt ok %d %d ", y, x);
	}
}


void segment_map::cal_roomBlock(std::vector<int> room_ids,NavMsg::BlockArray_t& blockArrays,cv::Mat indexed_map,cv::Mat forbidden_img,cv::Point3f mapInfo,cv::Point2i robot_pos)
{
	NavMsg::BlockArray_t new_blockArrays = blockArrays;
	cv::Mat prety_img = m_prety_img;
	cv::Mat filter_img  = m_filter_img;

	cv::Mat color_segmented_map = indexed_map.clone();
	color_segmented_map.convertTo(color_segmented_map, CV_8U);
	cv::cvtColor(color_segmented_map, color_segmented_map, cv::COLOR_GRAY2BGR);

	for (size_t i = 0; i < room_ids.size(); ++i)
	{
		int currId = room_ids [i];
	    cv::Mat newImg = cv::Mat(cv::Size(indexed_map.cols, indexed_map.rows), CV_8UC1, cv::Scalar(0));
		// choose random color for each room
		const cv::Vec3b color((rand() % 250) + 1, (rand() % 250) + 1, (rand() % 250) + 1);
		for (size_t v = 0; v < indexed_map.rows; ++v)
		{
			for (size_t u = 0; u < indexed_map.cols; ++u)
			{
				if (indexed_map.at<int>(v, u) == currId /* &&forbidden_img.at<unsigned char>(v,u)==0 */ )
				{
					newImg.at<unsigned char>(v, u) = 100;
					color_segmented_map.at<cv::Vec3b>(v, u) = color;
				}
			}
		}

		cv::Mat newImg_tmp= newImg.clone();

		std::vector<std::vector<cv::Point>> contours1;
		std::vector<std::vector<cv::Point>> contours1_tmp;
		std::vector<cv::Vec4i> hierarcy;

		// cv::findContours(cv::Mat(contours), contours1, hierarcy, 0, CV_CHAIN_APPROX_NONE);
		cv::findContours(newImg, contours1, hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

		int contor_index = -1;
		int max_contor = 0;
		for (size_t u = 0; u < contours1.size(); u++)
		{
			if (contours1[u].size() > max_contor)
			{
				contor_index = u;
				max_contor = contours1[u].size();
			}
		}
	//	contours1_tmp = contours1;
		xlog->Info("max_contor is %d   %d ", contor_index, max_contor);
		if (contor_index == -1)
		{
			xlog->Error("----------error room_contors \n");
			continue;
		}

		for (size_t j = 0; j < contours1[contor_index].size(); j++)
		{
			bool isConnect = false;
			if (blockArrays.blkNum>1)
			{
				for (int ii = -10; ii <= 10; ii++)
				{	
					if(isConnect)
						break;

					for (int jj = -10; jj <= 10; jj++)
					{
						int x = contours1[contor_index][j].x + ii;
						int y = contours1[contor_index][j].y + jj;
						float dist = sqrt(ii * ii + jj * jj);

						if (x<0||x>=indexed_map.cols||y<0||y>=indexed_map.rows)
						{
							continue;
						}
						
						if (indexed_map.at<int>(y, x) != currId&& indexed_map.at<int>(y, x)>0)
						{
							isConnect = true;
							break;
						}
					}
				}
				
				if (!isConnect)
				{
					
					float distUNk = 100;
					float distObs = 100;
					for (int ii = -3; ii <= 3; ii++)
					{
						for (int jj = -3; jj <= 3; jj++)
						{
							int x = contours1[0][j].x + ii;
							int y = contours1[0][j].y + jj;
							
							if (x<0||x>=indexed_map.cols||y<0||y>=indexed_map.rows)
							{
								continue;
							}

							if (filter_img.at<uchar>(y, x) == 100)
							{
								float dist = sqrt(ii * ii + jj * jj);
								if (dist < distObs)  distObs = dist;
							}
							else if (filter_img.at<uchar>(y, x) == 0)
							{
								float dist = sqrt(ii * ii + jj * jj);
								if (dist < distUNk)  distUNk = dist;
							}
						}
					}

					if (distObs <= distUNk && distObs<100)
					{
						for (int ii = -6; ii <= 6; ii++)
						{
							for (int jj = -6; jj <= 6; jj++)
							{
								int x = contours1[0][j].x + ii;
								int y = contours1[0][j].y + jj;
								//float dist = sqrt(ii * ii + jj * jj);

								if (x<0||x>=indexed_map.cols||y<0||y>=indexed_map.rows)
								{
									continue;
								}

								if (prety_img.at<uchar>(y, x) == 0)
								{
									newImg.at<uchar>(y, x) = 100;
								}
							}
						}
					}
				}
				else 
				{
					for (int ii = -2; ii <= 2; ii++)
					{
						for (int jj = -2; jj <= 2; jj++)
						{
							int x = contours1[contor_index][j].x + ii;
							int y = contours1[contor_index][j].y + jj;
							float dist = sqrt(ii * ii + jj * jj);

							if (x<0||x>=indexed_map.cols||y<0||y>=indexed_map.rows)
							{
								continue;
							}
							if (dist <= 2 && prety_img.at<uchar>(y, x) == 0)
							{
								newImg.at<uchar>(y, x) = 100;
								// newImg.at<unsigned char>(v, u) = 100;
							}
						}
					}
				}
			}
			else
			{
				for (int ii = -2; ii <= 2; ii++)
				{
					for (int jj = -2; jj <= 2; jj++)
					{
						int x = contours1[contor_index][j].x + ii;
						int y = contours1[contor_index][j].y + jj;
						float dist = sqrt(ii * ii + jj * jj);

						if (x<0||x>=indexed_map.cols||y<0||y>=indexed_map.rows)
						{
							continue;
						}

						if (dist <= 1.5 && prety_img.at<uchar>(y, x) == 0)
						{
							newImg.at<uchar>(y, x) = 100;
							// newImg.at<unsigned char>(v, u) = 100;
						}
					}
				}
			}
		}

		for (size_t j = 0; j < contours1[contor_index].size(); j++)
		{
			for (int ii = -2; ii <= 2; ii++)
			{
				for (int jj = -2; jj <= 2; jj++)
				{
					int x = contours1[contor_index][j].x + ii;
					int y = contours1[contor_index][j].y + jj;
					float dist = sqrt(ii * ii + jj * jj);

					if (dist <= 1.5 && prety_img.at<uchar>(y, x) == 255)
					{
						newImg_tmp.at<uchar>(y, x) = 0;
					}
				}
			}
		}

		cv::findContours(newImg_tmp, contours1_tmp, hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
		int contor_index1 = -1;
		int max_contor1 = 0;
		for (size_t u = 0; u < contours1_tmp.size(); u++)
		{
			if (contours1_tmp[u].size() > max_contor1)
			{
				contor_index1 = u;
				max_contor1 = contours1_tmp[u].size();
			}
		}

		for (size_t v = 0; v < indexed_map.rows; ++v)
		{
			for (size_t u = 0; u < indexed_map.cols; ++u)
			{
				if (newImg.at<unsigned char>(v, u) == 100&&forbidden_img.at<unsigned char>(v,u)>0)
				{
					newImg.at<unsigned char>(v, u) = 0;
				}
			}
		} 

		std::vector<std::vector<cv::Point>> contours2;
		cv::findContours(newImg, contours2, hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

		char name0[50];
		sprintf(name0, "/app/fj212/roomblock_new_%ld.jpg", currId);
		cv::imwrite(name0, newImg);

		std::vector<cv::Point> contours_tmp;
		//if (label_vector_index_codebook.size() > 1)
		{
			contours1 = contours2;
		}

		contor_index = -1;
		max_contor = 0;
		for (size_t u = 0; u < contours1.size(); u++)
		{
			float ret = cv::pointPolygonTest(contours1[u], cv::Point2f(robot_pos.x, robot_pos.y), true);
			xlog->Info("contours1 robotPos is %d  %f", contours1[u].size(),ret);
			if (ret < -4)
			{
				continue;
			}
			
			if (contours1[u].size() > max_contor)
			{
				contor_index = u;
				max_contor = contours1[u].size();
			}
		}
	
		if (contor_index ==-1)
		{
			for (size_t u = 0; u < contours1.size(); u++)
			{
				NavMsg::Pose_t pt = blockArrays.blks[currId-1].wallPose;
				int px = (int)round((pt.px - mapInfo.x) / mapInfo.z);
				int py = (int)round((pt.py - mapInfo.y) / mapInfo.z);
				float ret = cv::pointPolygonTest(contours1[u], cv::Point2f(px, py), true);
				xlog->Info("contours1 wallPose is %d %f ", contours1[u].size(),ret);
				if (ret < -8)
				{
					continue;
				}
				
				if (contours1[u].size() > max_contor)
				{
					contor_index = u;
					max_contor = contours1[u].size();
				}
			}
		}

		if (contor_index ==-1)
		{
			max_contor = 0;
			for (size_t u = 0; u < contours1.size(); u++)
			{
				if (contours1[u].size() > max_contor)
				{
					contor_index = u;
					max_contor = contours1[u].size();
				}
			}
		}
		
		xlog->Info("contours2 size is %d index is %d ", contours2.size(),contor_index);
		if (contor_index ==-1)
		{	
			continue;
		}

		contours_tmp = contours1[contor_index];

		int tmpCnt = 0;
		for (size_t j = 0; j < contours1[contor_index].size(); j++)
		{
			cv::Point2i pt;
			pt.x = contours1[contor_index][j].x;
			pt.y = contours1[contor_index][j].y;

			bool flag = checkIsWall(pt, 2);

			if (flag == true)
			{
				tmpCnt++;
				if (tmpCnt > 2)
				{
					for (size_t k = 0; k < contours1[contor_index].size(); k++)
					{
						contours_tmp[k] = contours1[contor_index][(k + j) % contours1[contor_index].size()];
					}
					break;
				}
			}
		}
		
		contours1[contor_index] = contours_tmp;

		std::vector<int> virPtTraj;
		std::vector<cv::Point2i> virtualPos;
		std::vector<std::pair<int, int>> virPtIndex;
		for (size_t j = 0; j < contours1[contor_index].size(); j++)
		{
			// contours.push_back(cv::Point2i(contours1[0][j].x, contours1[0][j].y));
			cv::Point2i pt;
			pt.x = contours1[contor_index][j].x;
			pt.y = contours1[contor_index][j].y;

			bool flag = checkIsWall(pt, 2);
			std::vector<int> neibors;
			int neibor_cnt = check_neigbors(pt.x, pt.y, currId, indexed_map, neibors);

			if (flag == false && neibor_cnt < 2)
			{
				virPtTraj.push_back(j);
			}
			else
			{
				if (virPtTraj.size() >= 3)
				{
					int start = virPtTraj[0] - 1;
					if (start < 0)
						start = 0;
					int end = virPtTraj[virPtTraj.size() - 1] + 1;
					if (end > contours1[contor_index].size() - 1)
						end = contours1[contor_index].size() - 1;

					virPtIndex.push_back(std::make_pair(start, end));
				}

				virPtTraj.clear();
			}
		}

		if (virPtTraj.size() >= 3)
		{
			int start = virPtTraj[0] - 1;
			if (start < 0)
				start = 0;
			int end = virPtTraj[virPtTraj.size() - 1] + 1;
			if (end > contours1[contor_index].size() - 1)
				end = contours1[contor_index].size() - 1;

			virPtIndex.push_back(std::make_pair(start, end));
		}

		virPtTraj.clear();

		NavMsg::Polygon_t plys = blockArrays.blks[currId-1];
		plys.isRecognized =true;
		std::vector<NavMsg::Pose_t> entryPoses;
		std::vector<NavMsg::Pose_t> candidateWallPoses;
		std::vector<int> entryWidths;
		std::vector<int32_t> entryIds;
		xlog->Info("virPtIndex.size() is %d ", virPtIndex.size());
		std::vector<cv::Point> final_out;
		if (virPtIndex.size()==0&&blockArrays.blkNum>1)
		{
			xlog->Error("virPtIndex is 0 ");
			for (size_t j = 0; j < contours1[contor_index].size(); j++)
			{
				// contours.push_back(cv::Point2i(contours1[0][j].x, contours1[0][j].y));
				cv::Point2i pt;
				pt.x = contours1[contor_index][j].x;
				pt.y = contours1[contor_index][j].y;

				bool flag = checkIsWall(pt, 2);
				std::vector<int> neibors;
				int neibor_cnt = check_neigbors(pt.x, pt.y, currId, indexed_map, neibors);

				if (flag == false && neibor_cnt < 2)
				{
					virPtTraj.push_back(j);
				}
				else
				{
					if (virPtTraj.size() >= 1)
					{
						int start = virPtTraj[0] - 1;
						if (start < 0)
							start = 0;
						int end = virPtTraj[virPtTraj.size() - 1] + 1;
						if (end > contours1[contor_index].size() - 1)
							end = contours1[contor_index].size() - 1;

						virPtIndex.push_back(std::make_pair(start, end));
					}

					virPtTraj.clear();
				}
			}
		}

		double epsilon =2;
		if (blockArrays.blkNum>1)
		{
			epsilon =4;
		}
		
		
		for (size_t j = 0; j < virPtIndex.size(); j++)
		{
			int start = 0;
			int end = 0;
			if (j == 0)
			{
				start = 0;
				end = virPtIndex[j].first;
			}
			else
			{
				start = virPtIndex[j - 1].second;
				end = virPtIndex[j].first;
			}

			std::vector<cv::Point> pts;

			for (size_t k = start; k < end; k++)
			{
				pts.push_back(contours1[contor_index][k]);
			}

			xlog->Info("pts size start is %d %d %d ", pts.size(), start, end);
			if (pts.size() > 0)
			{
				std::vector<cv::Point> out;
				cv::approxPolyDP(pts, out, 2, false);
				for (size_t k = 0; k < out.size(); k++)
				{
					final_out.push_back(out[k]);
				}
			}

			std::vector<cv::Point> pts1;
			std::vector<cv::Point> out1;
			for (size_t k = virPtIndex[j].first; k < virPtIndex[j].second; k++)
			{
				pts1.push_back(contours1[contor_index][k]);
			}
			if (pts1.size() == 0)
			{
				xlog->Error("pts size  is 0 ! %d ", virPtIndex[j].first);
				continue;
			}

			cv::approxPolyDP(pts1, out1, 2, false);

			for (size_t k = 0; k < out1.size(); k++)
			{
				xlog->Info("out1 %d is  %d  %d", k, out1[k].x, out1[k].y);
				final_out.push_back(out1[k]);
			}

			cv::Point p1 = contours1[contor_index][virPtIndex[j].first];
			cv::Point p2 = contours1[contor_index][virPtIndex[j].second];

			xlog->Info("first pos  is %d  %d  %d  %d", p1.x, p1.y, p2.x, p2.y);

			NavMsg::Pose_t pt;
			int center = (virPtIndex[j].first + virPtIndex[j].second) / 2;
			cv::Point p3 = contours1[contor_index][center];
			pt.px = (p1.x + p2.x) / 2;
			pt.py = (p1.y + p2.y) / 2;
			pt.pa = atan2(p1.y - p2.y, p1.x - p2.x);

			float minDist = INFINITY;
			float newPa = 0;
			cv::Point newPt;
			bool isFindValidPt=false;
			for (int k = 0; k < out1.size(); k++)
			{
				cv::Point2i p00(out1[k].x, out1[k].y);
				cv::Point2i p01(out1[(k + 1)%out1.size()].x, out1[(k + 1)%out1.size()].y);

				float dist=1000;
				cv::Point2i pd;
				bool isvalid = PointToline(p00, p01, p3,pd,dist);
				if (isvalid&&dist < minDist)
				{
					minDist = dist;
					newPa = atan2(p00.y - p01.y, p00.x - p01.x);
					xlog->Info("ploy %d dist %f  pa: %f  %d  %d  %d  %d  %d  %d ", k, dist, newPa, pd.x,  pd.y,p00.x, p00.y, p01.x, p01.y);
					newPt.x =  pd.x;
					newPt.y =  pd.y;
					isFindValidPt=true;
				}
			}
					
			if (isFindValidPt)
			{
				pt.px = newPt.x;
				pt.py = newPt.y;
				pt.pa = newPa;

				xlog->Info("pt %f %f  p3 %d  %d newPt:  %d  %d  ", pt.px, pt.py, p3.x,p3.y,newPt.x, newPt.y);
			}
			else
			{
		 		pt.px = p3.x;
				pt.py = p3.y;
				pt.pa = pt.pa; 
				xlog->Error("isFindValidPt false ");
			}

			int nextEntry = -1;
			bool sucess = check_neigbor_id(pt.px, pt.py, currId, indexed_map, nextEntry, blockArrays.blkNum);
			if (sucess && nextEntry > 0)
			{
				int entry_width = sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
				if (!contains(entryIds, nextEntry))
				{
					entryIds.push_back(nextEntry);
					xlog->Debug("entry  is %f  %f  %f  %d ", pt.px, pt.py, pt.pa, nextEntry);
					entryPoses.push_back(pt);
					entryWidths.push_back(entry_width);
				}
				else
				{
					for (size_t k = 0; k < entryIds.size(); k++)
					{
						if (entryIds[k]==nextEntry)
						{
							if (entryWidths[k] < entry_width)
							{
								entryWidths[k] = entry_width;
								entryPoses[k] = pt;
								xlog->Debug("update entry  is %f  %f  %f  %d ", pt.px, pt.py, pt.pa, nextEntry);
							}
							break;
						}		
					}	
				}
			}
			else
			{
				candidateWallPoses.push_back(pt);
				xlog->Error("entry failed ");
			}
				
		}

		
		xlog->Debug("entry %d  ",entryPoses.size());

		if (entryPoses.size()>0)
		{
			plys.wallPose = entryPoses[0];
		}
		else 
		{
			if (candidateWallPoses.size()>0)
			{
				plys.wallPose= candidateWallPoses[0];
				xlog->Debug("wallPose  is %f  %f  %f ", plys.wallPose.px, plys.wallPose.py, plys.wallPose.pa);
			}
			else 
			{
				xlog->Error("wallPose failed ");
				if(contours1[contor_index].size()>0)
				{
					plys.wallPose.px= contours1[contor_index][0].x;
					plys.wallPose.py= contours1[contor_index][0].y;
				}
				else
				{
					xlog->Error("wallPose contours1 failed ");
				}
			}
		}

		plys.entryNum = entryIds.size();
		plys.nextBlkIdOfEntries = entryIds;
		plys.entryPoses = entryPoses;
		

		u_int16_t start = 0;
		u_int16_t end = 0;
		if (0 == virPtIndex.size())
		{
			start = 0;
			end = contours1[contor_index].size() - 1;
		}
		else
		{
			start = virPtIndex[virPtIndex.size() - 1].second;
			end = contours1[contor_index].size() - 1;
		}
		std::vector<cv::Point> pts;
		for (size_t k = start; k < end; k++)
		{
			pts.push_back(contours1[contor_index][k]);
		}

		if (pts.size() > 0)
		{
			std::vector<cv::Point> out;
			cv::approxPolyDP(pts, out, 2, false);
			for (size_t k = 0; k < out.size(); k++)
			{
				final_out.push_back(out[k]);
			}
		}

		std::vector<cv::Point> newPloy;
		newPloy = final_out;
		std::vector<std::vector<cv::Point>> contours_poly;
		contours_poly.push_back(final_out);

		// printf("expand \n");
		cv::Mat map_new = color_segmented_map.clone();
		char name[50];
		// sprintf(name, "seg_map_room1.jpg");
		// cv::imwrite(name,result);
		// cv::ArrayList<cv::MatOfPoint>  list = new cv::ArrayList<>();
		cv::drawContours(map_new, contours_poly, 0, cv::Scalar(200), 1, 8);
		// cv::imshow("room", color_segmented_map);
		sprintf(name, "/app/fj212/seg_map_room_new_%ld.jpg", i);
		cv::imwrite(name, map_new);
		//xlog->Debug("corners : %d \n", contours_poly[i - 1].size());

		std::vector<NavMsg::Point_t> ployPoints;
		NavMsg::Point_t prePt;
		for (size_t j = newPloy.size(); j > 0; j--)
		{
			NavMsg::Point_t pt;
			pt.x = newPloy[j - 1].x;
			pt.y = newPloy[j - 1].y;
			if (pt.x < 0 || pt.y < 0 || pt.y > indexed_map.rows || pt.x > indexed_map.cols)
			{
				// xlog->Info("out of img  %d  %d ",pt.x,pt.y);
				continue;
			}

			if (j < newPloy.size())
			{
				float dist = sqrt((pt.x - prePt.x) * (pt.x - prePt.x) + (pt.y - prePt.y) * (pt.y - prePt.y));
				if (dist <= 2)
				{
					continue;
				}
			}

			ployPoints.push_back(pt);
			// printf("pt is %f %f \n",pt.x,pt.y);
			prePt = pt;
		}


		std::vector<cv::Point> out;
		cv::approxPolyDP(contours1_tmp[contor_index1], out, 2, false);
		std::vector<NavMsg::Point_t> ployPointsRaw;
		//or (size_t k = 0; k < out.size(); k++)
		for (size_t k = out.size(); k >0; k--)
		{
			NavMsg::Point_t pt;
			pt.x = out[k-1].x;
			pt.y = out[k-1].y;
			ployPointsRaw.push_back(pt);
		}
	

		plys.id = currId;
		plys.points = ployPoints;
		plys.pointNum = ployPoints.size();
		plys.pointsRaw = ployPointsRaw;
		plys.pointRawNum = ployPointsRaw.size();
		blockArrays.blks[currId-1]= plys;
		printf(" order %d \n",plys.cleanOrder);

		//currId++;
	}
}

void segment_map::room_segment(cv::Mat prety_img, NavMsg::BlockArray_t &blockArrays, cv::Mat &filter_img, cv::Point2i robot_pos)
{
	m_prety_img = prety_img;
	m_filter_img = filter_img;
	// segment the given map
	std::chrono::high_resolution_clock::time_point st00 = std::chrono::high_resolution_clock::now();
	cv::Mat segmented_map;
	float map_resolution = 0.05;
	// float room_lower_limit_distance_ = 0.15;//0.35;
	float room_lower_limit_distance_ = 0.35;
	float room_upper_limit_distance_ = 163;
	xlog->Info("start seg ");
	DistanceSegmentation distance_segmentation; // distance segmentation method
	distance_segmentation.segmentMap(prety_img, segmented_map, map_resolution, room_lower_limit_distance_, room_upper_limit_distance_);

	xlog->Info("end seg ");
	std::chrono::high_resolution_clock::time_point st10 = std::chrono::high_resolution_clock::now();
	double diff_t3 = std::chrono::duration_cast<std::chrono::duration<double>>(st10 - st00).count() * 1000;
	xlog->Info("segmap time  is    %f ", diff_t3);

	std::map<int, size_t> label_vector_index_codebook; // maps each room label to a position in the rooms vector
	size_t vector_index = 0;
	for (int v = 0; v < segmented_map.rows; ++v)
	{
		for (int u = 0; u < segmented_map.cols; ++u)
		{
			const int label = segmented_map.at<int>(v, u);
			if (label > 0 && label < 65280) // do not count walls/obstacles or free space as label
			{
				if (label_vector_index_codebook.find(label) == label_vector_index_codebook.end())
				{
					label_vector_index_codebook[label] = vector_index;
					vector_index++;
				}
			}
		}
	}

	// convert the segmented map into an indexed map which labels the segments with consecutive numbers (instead of arbitrary unordered labels in segmented map)
	cv::Mat indexed_map = segmented_map.clone();
	for (int y = 0; y < segmented_map.rows; ++y)
	{
		for (int x = 0; x < segmented_map.cols; ++x)
		{
			const int label = segmented_map.at<int>(y, x);
			if (label > 0 && label < 65280)
				indexed_map.at<int>(y, x) = label_vector_index_codebook[label] + 1; // start value from 1 --> 0 is reserved for obstacles
		}
	}

	int curr_Id = 1;
	if (robot_pos.x >= 0 && robot_pos.x < indexed_map.cols && robot_pos.y >= 0 && robot_pos.y < indexed_map.rows)
	{
		curr_Id = indexed_map.at<int>(robot_pos.y, robot_pos.x);
		if (curr_Id<=0||curr_Id >= 65280)
		{
			bool flag= get_curr_id(robot_pos.x, robot_pos.y,curr_Id,indexed_map);
			if (flag==false)
			{
				xlog->Error("find curr_id failed  %d  %d  ", robot_pos.x, robot_pos.y);
				curr_Id =1;
			}
			
		}
		
		xlog->Info("curr_id is  %d  %d  %d  ", robot_pos.x, robot_pos.y, curr_Id);
	}

	int room_cnt = label_vector_index_codebook.size();
	cv::Mat new_indexed_map = indexed_map.clone();
	//curr_Id =1;
	if (curr_Id > 1)
	{
		xlog->Info("room_cnt is  %d  ",room_cnt );
		for (size_t v = 0; v < indexed_map.rows; ++v)
		{
			for (size_t u = 0; u < indexed_map.cols; ++u)
			{
				if (indexed_map.at<int>(v, u) >= curr_Id && indexed_map.at<int>(v, u) <= room_cnt)
				{
					new_indexed_map.at<int>(v, u) = indexed_map.at<int>(v, u) - curr_Id + 1;
					//printf("1");
				}
				else if (indexed_map.at<int>(v, u) > 0 && indexed_map.at<int>(v, u) < curr_Id)
				{
					new_indexed_map.at<int>(v, u) = room_cnt + indexed_map.at<int>(v, u) - curr_Id + 1;
				}
			}
		}
	}

	indexed_map = new_indexed_map.clone();

	std::vector<std::vector<cv::Point>> contours_poly(label_vector_index_codebook.size()); //用于存放折线点集
	std::chrono::high_resolution_clock::time_point st11 = std::chrono::high_resolution_clock::now();
	double diff_t33 = std::chrono::duration_cast<std::chrono::duration<double>>(st11 - st00).count() * 1000;
	xlog->Info("total seg time  is    %f ", diff_t33);

	blockArrays.blks.clear();

	// colorize the segmented map with the indices of the room_center vector
	cv::Mat color_segmented_map = indexed_map.clone();
	m_indexed_map = indexed_map.clone();
	// printf("m_indexed_map size is %d %d\n",m_indexed_map.cols,m_indexed_map.rows);
	color_segmented_map.convertTo(color_segmented_map, CV_8U);
	cv::cvtColor(color_segmented_map, color_segmented_map, cv::COLOR_GRAY2BGR);
	int currId = 1;
	xlog->Info("roomNum  is    %d ", label_vector_index_codebook);
	
	for (size_t i = 1; i <= label_vector_index_codebook.size(); ++i)
	//for (size_t i = 1; i <= 1; ++i)
	{
		cv::Mat newImg = cv::Mat(cv::Size(indexed_map.cols, indexed_map.rows), CV_8UC1, cv::Scalar(0));
		// choose random color for each room
		std::vector<cv::Point> contours;
		const cv::Vec3b color((rand() % 250) + 1, (rand() % 250) + 1, (rand() % 250) + 1);
		for (size_t v = 0; v < indexed_map.rows; ++v)
		{
			for (size_t u = 0; u < indexed_map.cols; ++u)
			{
				if (indexed_map.at<int>(v, u) == currId)
				{
					color_segmented_map.at<cv::Vec3b>(v, u) = color;
					contours.push_back(cv::Point(u, v));
					newImg.at<unsigned char>(v, u) = 100;
				}
			}
		}

		cv::Mat newImg_tmp= newImg.clone();

		xlog->Debug("room is %d  area is %d ", i, contours.size());

		std::vector<std::vector<cv::Point>> contours1;
		std::vector<std::vector<cv::Point>> contours1_tmp;
		std::vector<cv::Vec4i> hierarcy;

		// cv::findContours(cv::Mat(contours), contours1, hierarcy, 0, CV_CHAIN_APPROX_NONE);
		cv::findContours(newImg, contours1, hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

		int contor_index = -1;
		int max_contor = 0;
		for (size_t u = 0; u < contours1.size(); u++)
		{
			if (contours1[u].size() > max_contor)
			{
				contor_index = u;
				max_contor = contours1[u].size();
			}
		}
	//	contours1_tmp = contours1;
		xlog->Info("max_contor is %d   %d ", contor_index, max_contor);
		if (contor_index == -1)
		{
			xlog->Error("----------error room_contors \n");
			continue;
		}

		for (size_t j = 0; j < contours1[contor_index].size(); j++)
		{
			bool isConnect = false;
			if (label_vector_index_codebook.size()>1)
			{
				for (int ii = -10; ii <= 10; ii++)
				{	
					if(isConnect)
						break;

					for (int jj = -10; jj <= 10; jj++)
					{
						int x = contours1[contor_index][j].x + ii;
						int y = contours1[contor_index][j].y + jj;
						float dist = sqrt(ii * ii + jj * jj);

						if (x<0||x>=new_indexed_map.cols||y<0||y>=new_indexed_map.rows)
						{
							continue;
						}
						
						if (new_indexed_map.at<int>(y, x) != i&& new_indexed_map.at<int>(y, x)>0)
						{
							isConnect = true;
							break;
						}
					}
				}
				
				if (!isConnect)
				{
					float distUNk = 100;
					float distObs = 100;
					for (int ii = -3; ii <= 3; ii++)
					{
						for (int jj = -3; jj <= 3; jj++)
						{
							int x = contours1[0][j].x + ii;
							int y = contours1[0][j].y + jj;
							
							if (x<0||x>=new_indexed_map.cols||y<0||y>=new_indexed_map.rows)
							{
								continue;
							}

							if (filter_img.at<uchar>(y, x) == 100)
							{
								float dist = sqrt(ii * ii + jj * jj);
								if (dist < distObs)  distObs = dist;
							}
							else if (filter_img.at<uchar>(y, x) == 0)
							{
								float dist = sqrt(ii * ii + jj * jj);
								if (dist < distUNk)  distUNk = dist;
							}
						}
					}

					if (distObs <= distUNk && distObs<100)
					{
						for (int ii = -6; ii <= 6; ii++)
						{

							for (int jj = -6; jj <= 6; jj++)
							{
								int x = contours1[0][j].x + ii;
								int y = contours1[0][j].y + jj;
								//float dist = sqrt(ii * ii + jj * jj);

								if (x<0||x>=new_indexed_map.cols||y<0||y>=new_indexed_map.rows)
								{
									continue;
								}

								if (prety_img.at<uchar>(y, x) == 0)
								{
									newImg.at<uchar>(y, x) = 100;
								}
							}
						}
					}
				}
				else 
				{
					for (int ii = -2; ii <= 2; ii++)
					{
						for (int jj = -2; jj <= 2; jj++)
						{
							int x = contours1[contor_index][j].x + ii;
							int y = contours1[contor_index][j].y + jj;
							float dist = sqrt(ii * ii + jj * jj);

							if (x<0||x>=new_indexed_map.cols||y<0||y>=new_indexed_map.rows)
							{
								continue;
							}
							if (dist <= 2 && prety_img.at<uchar>(y, x) == 0)
							{
								newImg.at<uchar>(y, x) = 100;
								// newImg.at<unsigned char>(v, u) = 100;
							}
						}
					}
				}
			}
			else
			{
				for (int ii = -2; ii <= 2; ii++)
				{
					for (int jj = -2; jj <= 2; jj++)
					{
						int x = contours1[contor_index][j].x + ii;
						int y = contours1[contor_index][j].y + jj;
						float dist = sqrt(ii * ii + jj * jj);

						if (x<0||x>=new_indexed_map.cols||y<0||y>=new_indexed_map.rows)
						{
							continue;
						}

						if (dist <= 1.5 && prety_img.at<uchar>(y, x) == 0)
						{
							newImg.at<uchar>(y, x) = 100;
							// newImg.at<unsigned char>(v, u) = 100;
						}
					}
				}
			}
		}

		for (size_t j = 0; j < contours1[contor_index].size(); j++)
		{
			for (int ii = -2; ii <= 2; ii++)
			{
				for (int jj = -2; jj <= 2; jj++)
				{
					int x = contours1[contor_index][j].x + ii;
					int y = contours1[contor_index][j].y + jj;
					float dist = sqrt(ii * ii + jj * jj);

					if (dist <= 1.5 && prety_img.at<uchar>(y, x) == 255)
					{
						newImg_tmp.at<uchar>(y, x) = 0;
					}
				}
			}
		}

		cv::findContours(newImg_tmp, contours1_tmp, hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
		int contor_index1 = -1;
		int max_contor1 = 0;
		for (size_t u = 0; u < contours1_tmp.size(); u++)
		{
			if (contours1_tmp[u].size() > max_contor1)
			{
				contor_index1 = u;
				max_contor1 = contours1_tmp[u].size();
			}
		}

		std::vector<std::vector<cv::Point>> contours2;
		cv::findContours(newImg, contours2, hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

		std::vector<cv::Point> contours_tmp;
		//if (label_vector_index_codebook.size() > 1)
		{
			contours1 = contours2;
		}

		contor_index = -1;
		max_contor = 0;
		for (size_t u = 0; u < contours1.size(); u++)
		{
			if (contours1[u].size() > max_contor)
			{
				contor_index = u;
				max_contor = contours1[u].size();
			}
		}

		contours_tmp = contours1[contor_index];

		int tmpCnt = 0;
		for (size_t j = 0; j < contours1[contor_index].size(); j++)
		{
			cv::Point2i pt;
			pt.x = contours1[contor_index][j].x;
			pt.y = contours1[contor_index][j].y;

			bool flag = checkIsWall(pt, 2);

			if (flag == true)
			{
				tmpCnt++;
				if (tmpCnt > 2)
				{
					for (size_t k = 0; k < contours1[contor_index].size(); k++)
					{
						contours_tmp[k] = contours1[contor_index][(k + j) % contours1[contor_index].size()];
					}
					break;
				}
			}
		}

		contours1[contor_index] = contours_tmp;

		std::vector<int> virPtTraj;
		std::vector<cv::Point2i> virtualPos;
		std::vector<std::pair<int, int>> virPtIndex;
		for (size_t j = 0; j < contours1[contor_index].size(); j++)
		{
			// contours.push_back(cv::Point2i(contours1[0][j].x, contours1[0][j].y));
			cv::Point2i pt;
			pt.x = contours1[contor_index][j].x;
			pt.y = contours1[contor_index][j].y;

			bool flag = checkIsWall(pt, 2);
			std::vector<int> neibors;
			int neibor_cnt = check_neigbors(pt.x, pt.y, currId, indexed_map, neibors);

			if (flag == false && neibor_cnt < 2)
			{
				virPtTraj.push_back(j);
			}
			else
			{
				if (virPtTraj.size() >= 3)
				{
					int start = virPtTraj[0] - 1;
					if (start < 0)
						start = 0;
					int end = virPtTraj[virPtTraj.size() - 1] + 1;
					if (end > contours1[contor_index].size() - 1)
						end = contours1[contor_index].size() - 1;

					virPtIndex.push_back(std::make_pair(start, end));
				}

				virPtTraj.clear();
			}
		}

		if (virPtTraj.size() >= 3)
		{
			int start = virPtTraj[0] - 1;
			if (start < 0)
				start = 0;
			int end = virPtTraj[virPtTraj.size() - 1] + 1;
			if (end > contours1[contor_index].size() - 1)
				end = contours1[contor_index].size() - 1;

			virPtIndex.push_back(std::make_pair(start, end));
		}

		virPtTraj.clear();

		NavMsg::Polygon_t plys;
		plys.isRecognized =true;
		plys.cleanForbidden =0;
		plys.yModeEnable =0;
		plys.fanRank =0;
		std::vector<NavMsg::Pose_t> entryPoses;
		std::vector<int> entryWidths;
		std::vector<int32_t> entryIds;
		xlog->Info("virPtIndex.size() is %d ", virPtIndex.size());
		std::vector<cv::Point> final_out;
		if (virPtIndex.size()==0&&label_vector_index_codebook.size() > 1)
		{
			xlog->Error("virPtIndex is 0 ");
			for (size_t j = 0; j < contours1[contor_index].size(); j++)
			{
				// contours.push_back(cv::Point2i(contours1[0][j].x, contours1[0][j].y));
				cv::Point2i pt;
				pt.x = contours1[contor_index][j].x;
				pt.y = contours1[contor_index][j].y;

				bool flag = checkIsWall(pt, 2);
				std::vector<int> neibors;
				int neibor_cnt = check_neigbors(pt.x, pt.y, currId, indexed_map, neibors);

				if (flag == false && neibor_cnt < 2)
				{
					virPtTraj.push_back(j);
				}
				else
				{
					if (virPtTraj.size() >= 1)
					{
						int start = virPtTraj[0] - 1;
						if (start < 0)
							start = 0;
						int end = virPtTraj[virPtTraj.size() - 1] + 1;
						if (end > contours1[contor_index].size() - 1)
							end = contours1[contor_index].size() - 1;

						virPtIndex.push_back(std::make_pair(start, end));
					}

					virPtTraj.clear();
				}
			}
		}

		double epsilon =2;
		if (label_vector_index_codebook.size()>1)
		{
			epsilon =4;
		}
		
		
		for (size_t j = 0; j < virPtIndex.size(); j++)
		{
			int start = 0;
			int end = 0;
			if (j == 0)
			{
				start = 0;
				end = virPtIndex[j].first;
			}
			else
			{
				start = virPtIndex[j - 1].second;
				end = virPtIndex[j].first;
			}

			std::vector<cv::Point> pts;

			for (size_t k = start; k < end; k++)
			{
				pts.push_back(contours1[contor_index][k]);
			}

			xlog->Info("pts size start is %d %d %d ", pts.size(), start, end);
			if (pts.size() > 0)
			{
				std::vector<cv::Point> out;
				cv::approxPolyDP(pts, out, 2, false);
				for (size_t k = 0; k < out.size(); k++)
				{
					final_out.push_back(out[k]);
				}
			}

			std::vector<cv::Point> pts1;
			std::vector<cv::Point> out1;
			for (size_t k = virPtIndex[j].first; k < virPtIndex[j].second; k++)
			{
				pts1.push_back(contours1[contor_index][k]);
			}
			if (pts1.size() == 0)
			{
				xlog->Error("pts size  is 0 ! %d ", virPtIndex[j].first);
				continue;
			}

			cv::approxPolyDP(pts1, out1, 2, false);

			for (size_t k = 0; k < out1.size(); k++)
			{
				xlog->Info("out1 %d is  %d  %d", k, out1[k].x, out1[k].y);
				final_out.push_back(out1[k]);
			}

			cv::Point p1 = contours1[contor_index][virPtIndex[j].first];
			cv::Point p2 = contours1[contor_index][virPtIndex[j].second];

			xlog->Info("first pos  is %d  %d  %d  %d", p1.x, p1.y, p2.x, p2.y);

			NavMsg::Pose_t pt;
			int center = (virPtIndex[j].first + virPtIndex[j].second) / 2;
			cv::Point p3 = contours1[contor_index][center];
			pt.px = (p1.x + p2.x) / 2;
			pt.py = (p1.y + p2.y) / 2;
			pt.pa = atan2(p1.y - p2.y, p1.x - p2.x);

			float minDist = INFINITY;
			float newPa = 0;
			cv::Point newPt;
			bool isFindValidPt=false;
			for (int k = 0; k < out1.size(); k++)
			{
				cv::Point2i p00(out1[k].x, out1[k].y);
				cv::Point2i p01(out1[(k + 1)%out1.size()].x, out1[(k + 1)%out1.size()].y);

				float dist=1000;
				cv::Point2i pd;
				bool isvalid = PointToline(p00, p01, p3,pd,dist);
				if (isvalid&&dist < minDist)
				{
					minDist = dist;
					newPa = atan2(p00.y - p01.y, p00.x - p01.x);
					xlog->Info("ploy %d dist %f  pa: %f  %d  %d  %d  %d  %d  %d ", k, dist, newPa, pd.x,  pd.y,p00.x, p00.y, p01.x, p01.y);
					newPt.x =  pd.x;
					newPt.y =  pd.y;
					isFindValidPt=true;
				}
			}
					
			if (isFindValidPt)
			{
				pt.px = newPt.x;
				pt.py = newPt.y;
				pt.pa = newPa;

				xlog->Info("pt %f %f  p3 %d  %d newPt:  %d  %d  ", pt.px, pt.py, p3.x,p3.y,newPt.x, newPt.y);
			}
			else
			{
		 		pt.px = p3.x;
				pt.py = p3.y;
				pt.pa = pt.pa; 
				xlog->Error("isFindValidPt false ");
			}

			int nextEntry = -1;
			bool sucess = check_neigbor_id(pt.px, pt.py, currId, indexed_map, nextEntry, label_vector_index_codebook.size());
			if (sucess && nextEntry > 0)
			{
				int entry_width = sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
				if (!contains(entryIds, nextEntry))
				{
					entryIds.push_back(nextEntry);
					xlog->Debug("entry  is %f  %f  %f  %d ", pt.px, pt.py, pt.pa, nextEntry);
					entryPoses.push_back(pt);
					entryWidths.push_back(entry_width);
				}
				else
				{
					for (size_t k = 0; k < entryIds.size(); k++)
					{
						if (entryIds[k]==nextEntry)
						{
							if (entryWidths[k] < entry_width)
							{
								entryWidths[k] = entry_width;
								entryPoses[k] = pt;
								xlog->Debug("update entry  is %f  %f  %f  %d ", pt.px, pt.py, pt.pa, nextEntry);
							}
							break;
						}		
					}	
				}
			}
			else
				xlog->Error("entry failed ");
		}

		plys.entryNum = entryIds.size();
		plys.nextBlkIdOfEntries = entryIds;
		plys.entryPoses = entryPoses;

		int start = 0;
		int end = 0;
		if (0 == virPtIndex.size())
		{
			start = 0;
			end = contours1[contor_index].size() - 1;
		}
		else
		{
			start = virPtIndex[virPtIndex.size() - 1].second;
			end = contours1[contor_index].size() - 1;
		}
		std::vector<cv::Point> pts;
		for (size_t k = start; k < end; k++)
		{
			pts.push_back(contours1[contor_index][k]);
		}

		if (pts.size() > 0)
		{
			std::vector<cv::Point> out;
			cv::approxPolyDP(pts, out, 2, false);
			for (size_t k = 0; k < out.size(); k++)
			{
				final_out.push_back(out[k]);
			}
		}

		std::vector<cv::Point> newPloy;
		newPloy = final_out;
		contours_poly[i - 1] = final_out;

		// printf("expand \n");
		cv::Mat map_new = color_segmented_map.clone();
		char name[50];
		// sprintf(name, "seg_map_room1.jpg");
		// cv::imwrite(name,result);
		// cv::ArrayList<cv::MatOfPoint>  list = new cv::ArrayList<>();
		cv::drawContours(map_new, contours_poly, i - 1, cv::Scalar(100, 255, 255), 1, 8);
		// cv::imshow("room", color_segmented_map);
		sprintf(name, "/app/fj212/seg_map_room%ld.jpg", i);
		cv::imwrite(name, map_new);
		xlog->Debug("corners : %d \n", contours_poly[i - 1].size());

		std::vector<NavMsg::Point_t> ployPoints;
		NavMsg::Point_t prePt;
		for (size_t j = newPloy.size(); j > 0; j--)
		{
			NavMsg::Point_t pt;
			pt.x = newPloy[j - 1].x;
			pt.y = newPloy[j - 1].y;
			if (pt.x < 0 || pt.y < 0 || pt.y > indexed_map.rows || pt.x > indexed_map.cols)
			{
				// xlog->Info("out of img  %d  %d ",pt.x,pt.y);
				continue;
			}

			if (j < newPloy.size())
			{
				float dist = sqrt((pt.x - prePt.x) * (pt.x - prePt.x) + (pt.y - prePt.y) * (pt.y - prePt.y));
				if (dist <= 2)
				{
					continue;
				}
			}

			ployPoints.push_back(pt);
			// printf("pt is %f %f \n",pt.x,pt.y);
			prePt = pt;
		}


		std::vector<cv::Point> out;
		cv::approxPolyDP(contours1_tmp[contor_index1], out, 2, false);
		std::vector<NavMsg::Point_t> ployPointsRaw;
		//or (size_t k = 0; k < out.size(); k++)
		for (size_t k = out.size(); k >0; k--)
		{
			NavMsg::Point_t pt;
			pt.x = out[k-1].x;
			pt.y = out[k-1].y;
			ployPointsRaw.push_back(pt);
		}
	

		plys.id = currId;
		plys.points = ployPoints;
		plys.pointNum = ployPoints.size();
		plys.pointsRaw = ployPointsRaw;
		plys.pointRawNum = ployPointsRaw.size();
		blockArrays.blks.push_back(plys);

		currId++;
	}
	cv::imwrite("color_seg_map.png", color_segmented_map);

	blockArrays.blkNum = blockArrays.blks.size();

#if 0
    cv::Point lastPt;
	lastPt.x=330;blockArrays.blks[0].entryPoses[0].px;
	lastPt.y=100;blockArrays.blks[0].entryPoses[0].py;
	printf("lastPt.x is %d %d \n",lastPt.x,lastPt.y);
	for (size_t i = 0; i < blockArrays.blkNum-1; i++)
	{
		/* code */
		int start= i+1;
		int goal = i+2;
		std::vector<std::pair<int, int>> room_path;
		std::vector<bool> visit;
		visit.resize(blockArrays.blkNum, false);
		int entry_num;
		NavMsg::Pose_t pt;
		getNextBlkEntry(start,goal,blockArrays,pt,visit);
		//printf("entry_num is %d \n",entry_num);
		cv::Point currPt;
		currPt.x=pt.px;
		currPt.y=pt.py;
		printf("currPt.x is %d %d \n",currPt.x,currPt.y);
		cv::line(color_segmented_map,lastPt,currPt,cv::Scalar(100,200,255),2);

		char name[10];
		sprintf(name, "%d", i+1);
	
		cv::putText(color_segmented_map, name, lastPt, 1, 2, cv::Scalar(0,255,0), 3);
		//cv::putText(color_segmented_map,lastPt,cv::Scalar(100,200,255),2);

		lastPt  =  currPt;


		if (i==blockArrays.blkNum-2)
		{
			char name[10];
			sprintf(name, "%d", i+2);
	
			cv::putText(color_segmented_map, name, lastPt, 1, 2, cv::Scalar(0,255,0), 3);
		}

	}


	blockArrays = new_blockArrays;
		

	 cv::imwrite("seg_map_final.jpg",color_segmented_map);

#endif

	m_indexed_map = indexed_map.clone();
	// m_allPolys = contours_poly;

	std::vector<std::vector<cv::Point>> contours0;
	for (size_t i = 0; i < blockArrays.blkNum; i++)
	{
		for (size_t j = 0; j < blockArrays.blks[i].entryPoses.size(); j++)
		{
			NavMsg::Pose_t pt = blockArrays.blks[i].entryPoses[j];

			if (contours0.size() == 0)
			{
				std::vector<cv::Point> curr_bound;
				curr_bound.push_back(cv::Point(pt.px - 10, pt.py + 10));
				curr_bound.push_back(cv::Point(pt.px + 10, pt.py + 10));
				curr_bound.push_back(cv::Point(pt.px + 10, pt.py - 10));
				curr_bound.push_back(cv::Point(pt.px - 10, pt.py - 10));
				contours0.push_back(curr_bound);

				for (size_t ii = pt.px - 10; ii < pt.px + 10; ii++)
				{
					for (size_t jj = pt.py - 10; jj < pt.py + 10; jj++)
					{

						if (filter_img.at<uchar>(jj, ii) == 100)
						{
							check_valid_Point(filter_img, ii, jj);
						}
					}
				}
			}
			else
			{
				bool isInclude = false;
				for (size_t k = 0; k < contours0.size(); k++)
				{
					int ret = cv::pointPolygonTest(contours0[k], cv::Point2f(pt.px, pt.py), false);

					if (ret == 1)
					{
						isInclude = true;
						break;
					}
				}

				if (isInclude == false)
				{
					std::vector<cv::Point> curr_bound;
					curr_bound.push_back(cv::Point(pt.px - 10, pt.py + 10));
					curr_bound.push_back(cv::Point(pt.px + 10, pt.py + 10));
					curr_bound.push_back(cv::Point(pt.px + 10, pt.py - 10));
					curr_bound.push_back(cv::Point(pt.px - 10, pt.py - 10));
					contours0.push_back(curr_bound);

					for (size_t ii = pt.px - 10; ii < pt.px + 10; ii++)
					{
						for (size_t jj = pt.py - 10; jj < pt.py + 10; jj++)
						{

							if (filter_img.at<uchar>(jj, ii) == 100)
							{
								check_valid_Point(filter_img, ii, jj);
							}
						}
					}
				}
			}
		}
	}
}

bool segment_map::checkIsWall(cv::Point2i u, int radius)
{
	float minDist = 1000;
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

			if (newS.x < 0 || newS.y < 0 || newS.y >= m_indexed_map.rows || newS.x >= m_indexed_map.cols)
			{
				// printf("out of img  %d  %d \n",newS.x,newS.y);
				return true;
			}

			if (m_indexed_map.at<int>(newS.y, newS.x) == 0)
			{
				return true;
			}
		}
	}

	return false;
}

void segment_map::checkDir(std::vector<cv::Point2i> mapPoints, int width, int height, float &pa)
{
	pa = 0;
	cv::Mat rowSum(1, height, CV_8UC1, cv::Scalar(0));
	cv::Mat colSum(1, width, CV_8UC1, cv::Scalar(0));
	for (size_t j = 0; j < mapPoints.size(); j++)
	{
		int y = mapPoints[j].y;
		int x = mapPoints[j].x;

		if (y >= 0 && y < height)
		{
			rowSum.at<unsigned char>(0, y)++;
		}

		if (x >= 0 && x < width)
		{
			colSum.at<unsigned char>(0, x)++;
		}
	}

	int row_max = 0;
	int col_max = 0;
	for (size_t j = 0; j < width; j++)
	{
		if (colSum.at<unsigned char>(0, j) > col_max)
			col_max = colSum.at<unsigned char>(0, j);
	}
	for (size_t j = 0; j < height; j++)
	{
		if (rowSum.at<unsigned char>(0, j) > row_max)
			row_max = rowSum.at<unsigned char>(0, j);
	}

	if (row_max < col_max)
	{
		pa = M_PI_2;
	}
}

bool segment_map::check_neigbor_id(int x, int y, int currId, cv::Mat segmented_map, int &entry_id, int len)
{
	std::vector<int> entryCandidata(len);
	bool isFindEntry = false;
	for (int v = -3 + y; v < 3 + y; ++v)
	{
		for (int u = -3 + x; u < 3 + x; ++u)
		{
			if (v < 0 || v >= segmented_map.rows)
				continue;

			if (u < 0 || u >= segmented_map.cols)
				continue;

			// only consider cells labeled as a room
			const int label = segmented_map.at<int>(v, u);
			if (label != currId)
				continue;

			for (int dv = -2; dv <= 2; ++dv)
			{
				for (int du = -2; du <= 2; ++du)
				{
					if (dv == 0 && du == 0)
						continue;
					const int neighbor_label = segmented_map.at<int>(v + dv, u + du);
					if (neighbor_label > 0 && neighbor_label < 65280 && (neighbor_label != label))
					{
						// either the room cell has a direct border to a different room or the room cell has a neighbor from the same room label with a connecting path to another room
						entry_id = neighbor_label;
						entryCandidata[neighbor_label - 1]++;
						isFindEntry = true;
						// return true;
					}
				}
			}
		}
	}

	int maxCnt = 0;
	int bestIndex = -1;
	if (isFindEntry)
	{
		for (size_t i = 0; i < len; i++)
		{
			if (entryCandidata[i] > maxCnt)
			{
				maxCnt = entryCandidata[i];
				bestIndex = i;
			}
		}
		entry_id = bestIndex + 1;

		return true;
	}

	return false;
}

int segment_map::check_neigbors(int x, int y, int currId, cv::Mat segmented_map, std::vector<int> &neibor_ids)
{
	neibor_ids.clear();
	for (int v = -2 + y; v <= 2 + y; ++v)
	{
		for (int u = -2 + x; u <= 2 + x; ++u)
		{
			if (v < 0 || v >= segmented_map.rows)
				continue;

			if (u < 0 || u >= segmented_map.cols)
				continue;

			// only consider cells labeled as a room
			const int label = segmented_map.at<int>(v, u);
			if (label == currId || label == 0)
				continue;

			if (!contains(neibor_ids, label))
			{
				neibor_ids.push_back(label);
			}
		}
	}

	return neibor_ids.size();
}


bool  segment_map::get_curr_id(int x,int y,int& currId,cv::Mat segmented_map)
{
	std::vector<int> candidataIds;
	int maxRoom =30;
	candidataIds.resize(maxRoom);
	for (int v = -3 + y; v < 3 + y; ++v)
	{
		for (int u = -3 + x; u < 3 + x; ++u)
		{
			if (v < 0 || v >= segmented_map.rows)
				continue;

			if (u < 0 || u >= segmented_map.cols)
				continue;

			// only consider cells labeled as a room
			const int label = segmented_map.at<int>(v, u);
			if (label ==0||label>= 65280)
				continue;
			if (label>maxRoom)
			{
				xlog->Error("room id  %d is too big",label);
				continue;
			}
			
			candidataIds[label - 1]++;
		}
	}

	int maxCnt = 0;
	int bestIndex = -1;
	for (size_t i = 0; i < maxRoom; i++)
	{
		if (candidataIds[i] > maxCnt)
		{
			maxCnt = candidataIds[i];
			bestIndex = i;
		}
	}
	currId = bestIndex+1;

	if (currId>=1)
	{
		return true;
	}
	else return false;
}

void segment_map::check_room_segment(cv::Point2i robot_pos, NavMsg::BlockArray_t &blockArrays, cv::Point2i charge_pos, int curr_room_id, bool hasCharge)
{
	// blockArrays.blks.clear();
	//  colorize the segmented map with the indices of the room_center vector;
	
	if (curr_room_id<=0)
	{
		curr_room_id = m_indexed_map.at<int>(robot_pos.y, robot_pos.x);
	}
	
	if (curr_room_id<=0||curr_room_id >= 65280)
	{
		bool flag= get_curr_id(robot_pos.x, robot_pos.y,curr_room_id,m_indexed_map);
		if (flag==false)
		{
			xlog->Error("find curr_id failed  %d  %d  ", robot_pos.x, robot_pos.y);
			curr_room_id =1;
		}	
	}
	cv::Mat new_segmented_map = m_indexed_map.clone();
	NavMsg::BlockArray_t newBlockArray;
	int roomID = 1;
	int plysSize = blockArrays.blks.size();
	if (plysSize<1)
	{
		return;
	}
	
	for (size_t i = 1; i <= plysSize; ++i)
	{
		cv::Mat newImg = cv::Mat(cv::Size(m_indexed_map.cols, m_indexed_map.rows), CV_8UC1, cv::Scalar(0));
		// choose random color for each room
		std::vector<cv::Point2i> contours;
		std::vector<cv::Point2i> room_points;
		int validCnt = 0;
		for (size_t v = 0; v < m_indexed_map.rows; ++v)
		{
			for (size_t u = 0; u < m_indexed_map.cols; ++u)
			{
				if (m_indexed_map.at<int>(v, u) == blockArrays.blks[i - 1].id)
				{
					newImg.at<unsigned char>(v, u) = 100;
					room_points.push_back(cv::Point2i(u, v));
					validCnt++;
				}
			}
		}

		if (validCnt == 0)
		{
			xlog->Error("failed check room \n");
			continue;
		}

		std::vector<std::vector<cv::Point>> contours1;
		std::vector<cv::Vec4i> hierarcy;

		cv::findContours(newImg, contours1, hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

		std::vector<cv::Point2i> entrys;
		std::vector<int> entrysId;
		float min_dist2wall = 1000;
		int bestIndex = -1;
		int isNotWallCnt = 0;
		for (size_t i = 0; i < contours1.size(); i++)
		{
			xlog->Info("roomID is %d   %d", i, contours1[i].size());
		}
		
		for (size_t j = 0; j < contours1[0].size(); j++)
		{
			contours.push_back(cv::Point2i(contours1[0][j].x, contours1[0][j].y));
		}

		float pa = 0;
		checkDir(room_points, m_indexed_map.cols, m_indexed_map.rows, pa);
		//pa = 0;
		NavMsg::Polygon_t plys = blockArrays.blks[i - 1];
		plys.id = roomID;
		plys.zPa = pa;
		xlog->Info("roomID is %d   pa is %f ", roomID, pa);

		if (plys.entryPoses.size() ==0&&curr_room_id == plys.id)
		{
			xlog->Error("roomID is %d   no entrys! ", roomID);
			
			float min_dist2wall = 1000;
			int bestIndex = -1;
			for (size_t j = 0; j < contours.size(); j++)
			{
				int x = contours[j].x;
				int y = contours[j].y;
				float dist2Dock = sqrt((x - charge_pos.x) * (x - charge_pos.x) + (y - charge_pos.y) * (y - charge_pos.y));
				float dist = sqrt((x - robot_pos.x) * (x - robot_pos.x) + (y - robot_pos.y) * (y - robot_pos.y));
			//	printf(" %d   %d  %d  %f \n",j,x,y,dist);
				// if (dist<min_dist2wall&&dist2Dock>16)
				if (dist < min_dist2wall && ((hasCharge && dist2Dock > 16) || hasCharge == 0))
				//if (dist<min_dist2wall)
				{
					min_dist2wall = dist;
					bestIndex = j;
				}
			}
			//if (bestIndex != -1)
			{
				//plys.wallPose.px = contours[bestIndex].x;
				//plys.wallPose.py = contours[bestIndex].y;
				plys.wallPose.px = robot_pos.x;
				plys.wallPose.py = robot_pos.y;
				xlog->Debug("wallpose  is %f  %f   %d \n", plys.wallPose.px, plys.wallPose.py,plys.pointNum);
				// plys.wallPose.pa=pa;
				cv::Point p2(plys.wallPose.px, plys.wallPose.py);
				float minDist = INFINITY;
				float newPa = 0;
				cv::Point2f newPt;
		
				for (size_t j = 0; j < plys.pointNum; j++)
				{
					cv::Point2i p0(plys.points[j].x, plys.points[j].y);
					cv::Point2i p1(plys.points[(j + 1) % plys.pointNum].x, plys.points[(j + 1) % plys.pointNum].y);

					float dist=1000;
					cv::Point2i pd;
					bool isvalid = PointToline(p0, p1, p2,pd,dist);
					//float dist = PointToline(p0, p1, p2);
					printf("point is %i %i %i %i %d %f\n",p0.x,p0.y,p1.x,p1.y,isvalid,dist);
					float dist2Dock = sqrt((pd.x - charge_pos.x) * (pd.x - charge_pos.x) + (pd.y - charge_pos.y) * (pd.y - charge_pos.y));
				
					if (hasCharge==true&&isvalid==true && dist < minDist && dist2Dock>20)
					{
						minDist = dist;
						newPa = atan2(plys.points[(j + 1) % plys.pointNum].y - plys.points[j].y, plys.points[(j + 1) % plys.pointNum].x - plys.points[j].x);

						xlog->Debug("ploy %d dist %f  pa: %f  %d  %d ",j,dist,newPa,pd.x,pd.y);

						newPt.x = pd.x;
						newPt.y = pd.y; 
					} 
					else if(hasCharge==false&&isvalid==true && dist < minDist)
					{
						minDist = dist;
						newPa = atan2(plys.points[(j + 1) % plys.pointNum].y - plys.points[j].y, plys.points[(j + 1) % plys.pointNum].x -plys.points[j].x);
						xlog->Debug("ploy %d dist %f  pa: %f  %d  %d ",j,dist,newPa,pd.x,pd.y);
						newPt.x = pd.x;
						newPt.y = pd.y; 
					} 
					
				}
				plys.wallPose.px = newPt.x;
				plys.wallPose.py = newPt.y;
				plys.wallPose.pa = newPa;
				xlog->Debug("new wallpose 1 is %f  %f  %f \n", plys.wallPose.px, plys.wallPose.py, newPa);
			}
		}
		else 
		{	
			if(plys.entryPoses.size()>0)
				plys.wallPose = plys.entryPoses[0];
			else 
			{
				plys.wallPose.px= contours[0].x;
				plys.wallPose.py= contours[0].y;
				plys.wallPose.pa=0;
				xlog->Debug("new wallpose 2 is %f  %f  %f \n", plys.wallPose.px, plys.wallPose.py, plys.wallPose.pa);
			}
		}

		newBlockArray.blks.push_back(plys);
		roomID++;
	}
	newBlockArray.blkNum = newBlockArray.blks.size();
	m_indexed_map = new_segmented_map.clone();
	newBlockArray.needMapExplorer= blockArrays.needMapExplorer;
	blockArrays = newBlockArray;

	std::vector<bool> visit;
	visit.resize(blockArrays.blkNum, false);
	xlog->Info("num is %d",blockArrays.blkNum);
	int curr_Id = 1;
	// new_blockArrays.blks.push_back(blockArrays.blks[curr_Id-1]);
	visit[curr_Id - 1] = true;
	blockArrays.blks[curr_Id - 1].cleanOrder = 1;
	for (int j = 0; j < blockArrays.blkNum - 1; j++)
	{
		bool isFindEntry = false;
		int start = curr_Id;
		// blockArrays.blks[curr_Id-1].cleanOrder=j+1;
		xlog->Info(" %d order is %d ", curr_Id, blockArrays.blks[curr_Id - 1].cleanOrder);
		for (int i = 0; i < blockArrays.blks[curr_Id - 1].entryNum; i++)
		{
			int nextId = blockArrays.blks[curr_Id - 1].nextBlkIdOfEntries[i];
			if (visit[nextId - 1] == false)
			{
				// new_blockArrays.blks.push_back(blockArrays.blks[nextId-1]);
				visit[nextId - 1] = true;
				curr_Id = nextId;
				isFindEntry = true;
				break;
			}
		}

		if (isFindEntry == false)
		{
			float minDist=1000;
			int bestId=-1;
			NavMsg::Pose_t pt0= blockArrays.blks[curr_Id - 1].wallPose; 
			for (size_t i = 0; i < blockArrays.blkNum; i++)
			{
				if (visit[i] == false)
				{
					for (size_t k = 0; k < blockArrays.blks[i].entryNum; k++)
					{
						NavMsg::Pose_t pt1= blockArrays.blks[i].entryPoses[k]; 
						float dist = sqrt((pt1.px-pt0.px)*(pt1.px-pt0.px)+(pt1.py-pt0.py)*(pt1.py-pt0.py));
						if (dist<minDist)
						{
							minDist =dist;
							bestId = i;
						}		
					}
				}
			}
			if (bestId!=-1)
			{
				visit[bestId] = true;
				curr_Id = bestId + 1;
			}
			else 
			{
				visit[curr_Id] = true;
				curr_Id = curr_Id + 1;
			}	
		}
		blockArrays.blks[curr_Id - 1].cleanOrder = j + 2;

		std::vector<bool> newvisit;
		newvisit.resize(blockArrays.blkNum, false);
		NavMsg::Pose_t pt;
		int goal = curr_Id;
		bool res = getNextBlkEntry(start, goal, blockArrays, pt, newvisit);
		if (!res) 
		{
			xlog->Error("getNextBlkEntry failed");
		//	pt = blockArrays.blks[goal - 1].entryPoses[0];
		}
		
		xlog->Info("block %d to %d entry is  %f %f %f ", start, goal, pt.px, pt.py, pt.pa);
		blockArrays.blks[goal - 1].wallPose = pt;
	}

	for (int i = 0; i < blockArrays.blkNum; i++)
	{
		int order = blockArrays.blks[i].cleanOrder;
		xlog->Info(" %d order is %d ", i, order);
	}

	xlog->Debug("blockArrays.blkNum is %d \n", newBlockArray.blkNum);
}


void segment_map::Recheck_room_segment(std::vector<int> room_ids,NavMsg::BlockArray_t &blockArrays)
{
	// blockArrays.blks.clear();
	//  colorize the segmented map with the indices of the room_center vector;
	for (size_t i = 0; i < room_ids.size(); ++i)
	{
		int currId = room_ids [i]-1;
		NavMsg::Polygon_t plys = blockArrays.blks[currId];
		printf("entry size is %d \n",plys.entryPoses.size());
		if(plys.entryPoses.size()>0)
			plys.wallPose = plys.entryPoses[0];
		if (plys.entryPoses.size() ==0)
		{
			xlog->Error("roomID is %d   no entrys! ", currId);
			int index =-1;
			float minDist=0;
			for (size_t j = 0; j < plys.pointNum; j++)
			{
				cv::Point2i p0(plys.points[j].x, plys.points[j].y);
				cv::Point2i p1(plys.points[(j + 1) % plys.pointNum].x, plys.points[(j + 1) % plys.pointNum].y);
				float dist = sqrt((p0.x-p1.x)*(p0.x-p1.x)+(p0.y-p1.y)*(p0.y-p1.y));
				if(dist>minDist)
				{
					minDist = dist;
					index =j;
				}
			}

			cv::Point2i p0(plys.points[index].x, plys.points[index].y);
			cv::Point2i p1(plys.points[(index + 1) % plys.pointNum].x, plys.points[(index + 1) % plys.pointNum].y);
			cv::Point2i p2((p0.x+p1.x)/2,(p0.y+p1.y)/2);	
			plys.wallPose.px = p2.x;
			plys.wallPose.py = p2.y;
			plys.wallPose.pa =  atan2(plys.points[(index + 1) % plys.pointNum].y - plys.points[index].y, plys.points[(index + 1) % plys.pointNum].x - plys.points[index].x);

			printf("%d %d - %d %d \n",p0.x,p0.y,p1.x,p1.y);			
			xlog->Debug("new wallpose 2 is %f  %f  %f \n", plys.wallPose.px, plys.wallPose.py, plys.wallPose.pa);
		}
		blockArrays.blks[currId].wallPose = plys.wallPose;
	}
}

void segment_map::check_room_wallPose(cv::Point2i robot_pos, NavMsg::BlockArray_t &blockArrays, cv::Point2i charge_pos, int curr_room_id, bool hasCharge)
{
	// blockArrays.blks.clear();
	//  colorize the segmented map with the indices of the room_center vector;
	curr_room_id = m_indexed_map.at<int>(robot_pos.y, robot_pos.x);
	if (curr_room_id<=0||curr_room_id >= 65280)
	{
		bool flag= get_curr_id(robot_pos.x, robot_pos.y,curr_room_id,m_indexed_map);
		if (flag==false)
		{
			xlog->Error("find curr_id failed  %d  %d  ", robot_pos.x, robot_pos.y);
			curr_room_id =1;
		}	
	}
	cv::Mat new_segmented_map = m_indexed_map.clone();
	NavMsg::BlockArray_t newBlockArray;
	int roomID = 1;
	int plysSize = blockArrays.blks.size();
	for (size_t i = 1; i <= plysSize; ++i)
	{
		NavMsg::Polygon_t plys = blockArrays.blks[i - 1];

		if (plys.entryPoses.size() ==0&&curr_room_id == plys.id)
		{
			xlog->Error("roomID is %d   no entrys!! ", roomID);
			
			{
				//plys.wallPose.px = contours[bestIndex].x;
				//plys.wallPose.py = contours[bestIndex].y;
				//plys.wallPose.px = robot_pos.x;
				//plys.wallPose.py = robot_pos.y;
				xlog->Debug("wallpose  is %f  %f   %d \n", plys.wallPose.px, plys.wallPose.py,plys.pointNum);
				// plys.wallPose.pa=pa;
				cv::Point p2(plys.wallPose.px, plys.wallPose.py);
				float minDist = INFINITY;
				float newPa = 0;
				cv::Point2f newPt;
				bool findWallPose = false;
		
				for (size_t j = 0; j < plys.pointNum; j++)
				{
					cv::Point2f p0(plys.points[j].x, plys.points[j].y);
					cv::Point2f p1(plys.points[(j + 1) % plys.pointNum].x, plys.points[(j + 1) % plys.pointNum].y);

					float dist=1000;
					cv::Point2f pd;
					bool isvalid = PointTolinef(p0, p1, p2,pd,dist);
					//float dist = PointToline(p0, p1, p2);
					printf("point is %f %f %f %f %d %f\n",p0.x,p0.y,p1.x,p1.y,isvalid,dist);
					float dist2Dock = sqrt((pd.x - charge_pos.x) * (pd.x - charge_pos.x) + (pd.y - charge_pos.y) * (pd.y - charge_pos.y));
				
					if (hasCharge==true&&isvalid==true && dist < minDist && dist2Dock>1)
					{
						minDist = dist;
						newPa = atan2(plys.points[(j + 1) % plys.pointNum].y - plys.points[j].y, plys.points[(j + 1) % plys.pointNum].x - plys.points[j].x);

						xlog->Debug("ploy %d dist %f  pa: %f  %d  %d ",j,dist,newPa,pd.x,pd.y);

						newPt.x = pd.x;
						newPt.y = pd.y; 

						findWallPose =true;
					}
					else if (hasCharge==false&&isvalid==true && dist < minDist)
					{
						minDist = dist;
						newPa = atan2(plys.points[(j + 1) % plys.pointNum].y - plys.points[j].y, plys.points[(j + 1) % plys.pointNum].x - plys.points[j].x);

						xlog->Debug("ploy %d dist %f  pa: %f  %d  %d ",j,dist,newPa,pd.x,pd.y);

						newPt.x = pd.x;
						newPt.y = pd.y; 

						findWallPose =true;
					}
				}

				if (findWallPose == true)
				{
					plys.wallPose.px = newPt.x;
					plys.wallPose.py = newPt.y;
					plys.wallPose.pa = newPa;
				}
				
				xlog->Debug("new wallpose 2 is %f  %f  %f \n", plys.wallPose.px, plys.wallPose.py, newPa);
			}
		}
		else 
		{	
			if(plys.entryPoses.size()>0)
				plys.wallPose = plys.entryPoses[0];
		}

		newBlockArray.blks.push_back(plys);
		roomID++;
	}
	newBlockArray.blkNum = newBlockArray.blks.size();
	newBlockArray.needMapExplorer= blockArrays.needMapExplorer;
	blockArrays = newBlockArray;
}




bool segment_map::room_block_segment(cv::Mat indexed_img, std::vector<int> room_ids, NavMsg::BlockArray_t &blockArrays)
{
	NavMsg::BlockArray_t new_blockArrays = blockArrays;
	for (size_t i = 0; i < room_ids.size(); ++i)
	{
		int id = room_ids [i];
		cv::Mat newImg = cv::Mat(cv::Size(indexed_img.cols, indexed_img.rows), CV_8UC1, cv::Scalar(0));

		printf("111 \n");
		int validCnt = 0;
		const cv::Vec3b color((rand() % 250) + 1, (rand() % 250) + 1, (rand() % 250) + 1);
		for (size_t v = 0; v < indexed_img.rows; ++v)
		{
			for (size_t u = 0; u < indexed_img.cols; ++u)
			{
				if (indexed_img.at<int>(v, u) == id)
				{
					newImg.at<unsigned char>(v, u) = 100;
					validCnt++;
				}
			}
		}

		printf("222 \n");

		std::vector<std::vector<cv::Point>> contours1;
		std::vector<cv::Vec4i> hierarcy;
		cv::findContours(newImg, contours1, hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

		int index = 0;
		int maxContors = 0;
		for (size_t j = 0; j < contours1.size(); j++)
		{
			if (contours1[j].size() > maxContors)
			{
				maxContors = contours1[j].size();
				index = j;
			}
		}

		for (size_t j = 0; j < contours1[index].size(); j++)
		{
			for (int ii = -2; ii <= 2; ii++)
			{
				for (int jj = -2; jj <= 2; jj++)
				{
					int x = contours1[index][j].x + ii;
					int y = contours1[index][j].y + jj;
					float dist = sqrt(ii * ii + jj * jj);

					if (dist <= 2 && indexed_img.at<uchar>(y, x) == 0)
					{
						newImg.at<uchar>(y, x) = 100;
						// newImg.at<unsigned char>(v, u) = 100;
					}
				}
			}
		}

		index = 0;
		maxContors = 0;
		cv::findContours(newImg, contours1, hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
		for (size_t j = 0; j < contours1.size(); j++)
		{
			printf("contor %d  is %d",j,contours1[j].size());
			if (contours1[j].size() > maxContors)
			{
				maxContors = contours1[j].size();
				index = j;
			}
		}

		std::vector<cv::Point> newPloy;
		cv::approxPolyDP(contours1[index], newPloy, 2, true);
		xlog->Info("room %d corners : %d ", id,newPloy.size());


		char name[50];
		// sprintf(name, "seg_map_room1.jpg");
		// cv::imwrite(name,result);
		// cv::ArrayList<cv::MatOfPoint>  list = new cv::ArrayList<>();
		cv::drawContours(newImg, contours1, index, cv::Scalar(200), 1, 8);
		// cv::imshow("room", color_segmented_map);
		sprintf(name, "/app/fj212/forbidden_seg_map_room%ld.png", id);
		cv::imwrite(name, newImg);
		
		std::vector<NavMsg::Point_t> newPloyPoints;
		NavMsg::Point_t prePt;
		for (size_t j = newPloy.size(); j > 0; j--)
		{
			NavMsg::Point_t pt;
			pt.x = newPloy[j - 1].x;
			pt.y = newPloy[j - 1].y;

			if (j < newPloy.size())
			{
				float dist = sqrt((pt.x - prePt.x) * (pt.x - prePt.x) + (pt.y - prePt.y) * (pt.y - prePt.y));
				float distX = abs(pt.x - prePt.x);
				float distY = abs(pt.y - prePt.y);
				if (dist <= 2)
				{
					continue;
				}
			}

			if (pt.x > indexed_img.cols || pt.y > indexed_img.rows || pt.x <= 0 || pt.y <= 0)
			{
				continue;
			}
			prePt = pt;

			newPloyPoints.push_back(pt);
			// printf("pt is %f  %f \n",pt.x,pt.y);
		}
		new_blockArrays.blks[id - 1].points = newPloyPoints;
		new_blockArrays.blks[id - 1].pointNum = newPloyPoints.size();
	}

	blockArrays = new_blockArrays;
	return true;
}


bool segment_map::room_block_resegment(cv::Mat indexed_img,cv::Mat forbidden_img,std::vector<int> room_ids,NavMsg::BlockArray_t& blockArrays,cv::Point3f mapInfo)
{
	NavMsg::BlockArray_t new_blockArrays = blockArrays;
	for (size_t i = 0; i < room_ids.size(); ++i)
	{
		int id = room_ids [i];
		cv::Mat newImg = cv::Mat(cv::Size(indexed_img.cols, indexed_img.rows), CV_8UC1, cv::Scalar(0));

		printf("111 \n");
		int validCnt = 0;
		const cv::Vec3b color((rand() % 250) + 1, (rand() % 250) + 1, (rand() % 250) + 1);
		for (size_t v = 0; v < indexed_img.rows; ++v)
		{
			for (size_t u = 0; u < indexed_img.cols; ++u)
			{
				if (indexed_img.at<int>(v, u) == id && forbidden_img.at<unsigned char>(v, u)==0)
				{
					newImg.at<unsigned char>(v, u) = 100;
					validCnt++;
				}
			}
		}	

		printf("222 \n");

		std::vector<std::vector<cv::Point>> contours1;
		std::vector<cv::Vec4i> hierarcy;
		cv::findContours(newImg, contours1, hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

		int index = 0;
		int maxContors = 0;
		for (size_t j = 0; j < contours1.size(); j++)
		{
			if (contours1[j].size() > maxContors)
			{
				maxContors = contours1[j].size();
				index = j;
			}
		}

		for (size_t j = 0; j < contours1[index].size(); j++)
		{
			for (int ii = -2; ii <= 2; ii++)
			{
				for (int jj = -2; jj <= 2; jj++)
				{
					int x = contours1[index][j].x + ii;
					int y = contours1[index][j].y + jj;
					float dist = sqrt(ii * ii + jj * jj);

					if (dist <= 2 && indexed_img.at<uchar>(y, x) == 0)
					{
						newImg.at<uchar>(y, x) = 100;
						// newImg.at<unsigned char>(v, u) = 100;
					}
				}
			}
		}

		index = 0;
		maxContors = 0;
		cv::findContours(newImg, contours1, hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
		for (size_t j = 0; j < contours1.size(); j++)
		{
			printf("contor %d  is %d",j,contours1[j].size());

			NavMsg::Pose_t pt = blockArrays.blks[id-1].wallPose;
			int px = (int)round((pt.px - mapInfo.x) / mapInfo.z);
            int py = (int)round((pt.py - mapInfo.y) / mapInfo.z);
			float ret = cv::pointPolygonTest(contours1[j], cv::Point2f(px, py), true);
			if (ret < -2)
			{
				continue;
			}

			if (contours1[j].size() > maxContors)
			{
				maxContors = contours1[j].size();
				index = j;
			}
		}

		std::vector<cv::Point> newPloy;
		cv::approxPolyDP(contours1[index], newPloy, 2, true);
		xlog->Info("room %d corners : %d ", id,newPloy.size());


		char name[50];
		// sprintf(name, "seg_map_room1.jpg");
		// cv::imwrite(name,result);
		// cv::ArrayList<cv::MatOfPoint>  list = new cv::ArrayList<>();
		cv::drawContours(newImg, contours1, index, cv::Scalar(200), 1, 8);
		// cv::imshow("room", color_segmented_map);
		sprintf(name, "/app/fj212/forbidden_seg_map_room%ld.png", id);
		cv::imwrite(name, newImg);

		sprintf(name, "/app/fj212/forbidden_seg_map %ld.png", id);
		cv::imwrite(name, forbidden_img);
		
		std::vector<NavMsg::Point_t> newPloyPoints;
		NavMsg::Point_t prePt;
		for (size_t j = newPloy.size(); j > 0; j--)
		{
			NavMsg::Point_t pt;
			pt.x = newPloy[j - 1].x;
			pt.y = newPloy[j - 1].y;

			if (j < newPloy.size())
			{
				float dist = sqrt((pt.x - prePt.x) * (pt.x - prePt.x) + (pt.y - prePt.y) * (pt.y - prePt.y));
				float distX = abs(pt.x - prePt.x);
				float distY = abs(pt.y - prePt.y);
				if (dist <= 2)
				{
					continue;
				}
			}

			if (pt.x > indexed_img.cols || pt.y > indexed_img.rows || pt.x <= 0 || pt.y <= 0)
			{
				continue;
			}
			prePt = pt;

			newPloyPoints.push_back(pt);
			// printf("pt is %f  %f \n",pt.x,pt.y);
		}
		new_blockArrays.blks[id - 1].points = newPloyPoints;
		new_blockArrays.blks[id - 1].pointNum = newPloyPoints.size();
	}

	blockArrays = new_blockArrays;
	return true;
}

double min(float x, float y)
{
	return x < y ? x : y;
}

double max(float x, float y)
{
	return x > y ? x : y;
}

/* float calNormals(float x1, float y1,float x2,float y2)
{
	double n = x1*x2+y1*y2;
	float m = sqrtf(x1*x1+y1*y1)*sqrtf(x2*x2+y2*y2);
	return arccos(n/m);
} */


bool PointIsInline(NavMsg::Point_t a, NavMsg::Point_t b, NavMsg::Point_t  p)
{
	float ap_ab = (b.x - a.x) * (p.x - a.x) + (b.y - a.y) * (p.y - a.y); // cross( a , p , b );
	if (ap_ab <= 0)
		return false;

	float d2 = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
	if (ap_ab >= d2)
		return false;

	return true;
}

//排斥实验
bool IsRectCross(const NavMsg::Point_t &p1, const NavMsg::Point_t &p2, const NavMsg::Point_t &q1, const NavMsg::Point_t &q2)
{
	bool ret = min(p1.x, p2.x) <= max(q1.x, q2.x) &&
			   min(q1.x, q2.x) <= max(p1.x, p2.x) &&
			   min(p1.y, p2.y) <= max(q1.y, q2.y) &&
			   min(q1.y, q2.y) <= max(p1.y, p2.y);
	return ret;
}

//跨立判断
bool IsLineSegmentCross(const NavMsg::Point_t &P1, const NavMsg::Point_t &P2, const NavMsg::Point_t &Q1, const NavMsg::Point_t &Q2)
{
	if (
		((Q1.x - P1.x) * (Q1.y - Q2.y) - (Q1.y - P1.y) * (Q1.x - Q2.x)) * ((Q1.x - P2.x) * (Q1.y - Q2.y) - (Q1.y - P2.y) * (Q1.x - Q2.x)) < 0 ||
		((P1.x - Q1.x) * (P1.y - P2.y) - (P1.y - Q1.y) * (P1.x - P2.x)) * ((P1.x - Q2.x) * (P1.y - P2.y) - (P1.y - Q2.y) * (P1.x - P2.x)) < 0)
		return true;
	else
		return false;
}

bool segment_map::GetCrossPoint(const NavMsg::Point_t &p1, const NavMsg::Point_t &p2, const NavMsg::Point_t &q1, const NavMsg::Point_t &q2, float &x, float &y)
{
	if (IsRectCross(p1, p2, q1, q2))
	{
		if (IsLineSegmentCross(p1, p2, q1, q2))
		{
			//求交点
			float tmpLeft, tmpRight;
			tmpLeft = (q2.x - q1.x) * (p1.y - p2.y) - (p2.x - p1.x) * (q1.y - q2.y);
			tmpRight = (p1.y - q1.y) * (p2.x - p1.x) * (q2.x - q1.x) + q1.x * (q2.y - q1.y) * (p2.x - p1.x) - p1.x * (p2.y - p1.y) * (q2.x - q1.x);

			x = tmpRight / tmpLeft;

			tmpLeft = (p1.x - p2.x) * (q2.y - q1.y) - (p2.y - p1.y) * (q1.x - q2.x);
			tmpRight = p2.y * (p1.x - p2.x) * (q2.y - q1.y) + (q2.x - p2.x) * (q2.y - q1.y) * (p1.y - p2.y) - q2.y * (q1.x - q2.x) * (p2.y - p1.y);
			y = tmpRight / tmpLeft;
			return true;
		}
	}
	return false;
}



bool segment_map::single_room_segment(std::vector<NavMsg::Point_t> blockVerPts, int id, NavMsg::BlockArray_t &blockArrays,std::vector<NavMsg::Point_t>& realVertPts)
{
	std::vector<NavMsg::Point_t> newPloyPoints;
	// blockArrays.blks[id-1].pointNum;
	if (blockVerPts.size() < 4)
	{
		return false;
	}

	NavMsg::Point_t p01 = blockVerPts[0];
	NavMsg::Point_t p02 = blockVerPts[1];
	int startIndex = -1;
	int endIndex = -1;
	float x = 0;
	float y = 0;
	int firstCross =0;

	bool iscloseToWall=false;
	float minDist = INFINITY;
	cv::Point2f closePd;
	int pointNums = blockArrays.blks[id - 1].pointNum;
	for (size_t i = 0; i < blockArrays.blks[id - 1].pointNum; i++)
	{	
		cv::Point2f p0( blockArrays.blks[id - 1].points[i].x,  blockArrays.blks[id - 1].points[i].y);
		cv::Point2f p1( blockArrays.blks[id - 1].points[(i + 1) % pointNums].x,  blockArrays.blks[id - 1].points[(i + 1) % pointNums].y);
		cv::Point2f p2(p01.x,p01.y); 

		float dist=1000;
		cv::Point2f pd;
		bool isvalid = PointTolinef(p0, p1, p2,pd,dist);
		printf("point is %f %f %f %f %d %f\n",p0.x,p0.y,p1.x,p1.y,isvalid,dist);
		if (isvalid==true)
		{
			if (dist<minDist)
			{
				minDist = dist;
				iscloseToWall =true;
				closePd.x = pd.x;  closePd.y = pd.y;
				startIndex = i; 
			}	
		}
	}

	bool iscloseToWall1=false;
	float minDist1 = INFINITY;
	cv::Point2f closePd1;
	for (size_t i = 0; i < blockArrays.blks[id - 1].pointNum; i++)
	{	
		cv::Point2f p0( blockArrays.blks[id - 1].points[i].x,  blockArrays.blks[id - 1].points[i].y);
		cv::Point2f p1( blockArrays.blks[id - 1].points[(i + 1) % pointNums].x,  blockArrays.blks[id - 1].points[(i + 1) % pointNums].y);
		cv::Point2f p2(blockVerPts[2].x,blockVerPts[2].y); 

		float dist=1000;
		cv::Point2f pd;
		bool isvalid = PointTolinef(p0, p1, p2,pd,dist);
		printf("next point is %f %f %f %f %d %f\n",p0.x,p0.y,p1.x,p1.y,isvalid,dist);
		if (isvalid==true)
		{
			if (dist<minDist1)
			{
				minDist1 = dist;
				iscloseToWall1 =true;
				closePd1.x = pd.x;  closePd1.y = pd.y; 
				endIndex = i;
			}	
		}
	}

	std::vector<NavMsg::Point_t> stationPts;
	if (iscloseToWall&& minDist<0.15)
	{
		NavMsg::Point_t p11;
		p11.x = closePd.x; p11.y = closePd.y;
		p01 = blockVerPts[2];
		p02 = blockVerPts[3];
		endIndex =-1;
		for (size_t i = 0; i < blockArrays.blks[id - 1].pointNum; i++)
		{
			NavMsg::Point_t p1 = blockArrays.blks[id - 1].points[i];
			NavMsg::Point_t p2 = blockArrays.blks[id - 1].points[(i + 1) % pointNums];

			bool flag = GetCrossPoint(p01, p02, p1, p2, x, y);

			NavMsg::Point_t crossPt;
			crossPt.x= x;
			crossPt.y= y;
			bool flag1= PointIsInline(p01, p02, crossPt);
			if (flag && flag1)
			{
				xlog->Info("p3  %f  %f,  %f %f,  %f %f ", p1.x, p1.y, p2.x, p2.y, x, y);
				endIndex = i;
				break;
			}
		}
		NavMsg::Point_t p22;
		if (endIndex!=-1)
		{
			p22.x = x; p22.y = y;
		}
		else 
		{
			xlog->Error(" error endIndex is valid  ");
			return false;
		}
		
		stationPts.push_back(p11);
		stationPts.push_back(blockVerPts[2]);
		stationPts.push_back(p22);

		NavMsg::Point_t p33;
		p33.x = p11.x + p22.x-blockVerPts[2].x;
		p33.y = p11.y + p22.y-blockVerPts[2].y;
		
		realVertPts.push_back(p11);
		realVertPts.push_back(blockVerPts[2]);
		realVertPts.push_back(p22);
		realVertPts.push_back(p33);
	}
	else if (iscloseToWall1&& minDist1<0.15)
	{
		p01 = blockVerPts[0];
		p02 = blockVerPts[1];
		startIndex =-1;
		for (size_t i = 0; i < blockArrays.blks[id - 1].pointNum; i++)
		{
			NavMsg::Point_t p1 = blockArrays.blks[id - 1].points[i];
			NavMsg::Point_t p2 = blockArrays.blks[id - 1].points[(i + 1) % blockArrays.blks[id - 1].pointNum];

			bool flag = GetCrossPoint(p01, p02, p1, p2, x, y);
			NavMsg::Point_t crossPt;
			crossPt.x= x;
			crossPt.y= y;
			bool flag1= PointIsInline(p01, p02, crossPt);
			if (flag && flag1)
			{
				startIndex = i;
				xlog->Info("p1  %f  %f,  %f %f,  %f %f  ", p1.x, p1.y, p2.x, p2.y, x, y);
				break;
			}
		}

		NavMsg::Point_t p11;
		if (startIndex!=-1)
		{
			p11.x = x; p11.y = y;
		}
		else 
		{
			xlog->Error(" error startIndex is valid  ");
			return false;
		}

		NavMsg::Point_t p22;
		p22.x = closePd1.x; p22.y = closePd1.y;
		
		stationPts.push_back(p11);
		stationPts.push_back(blockVerPts[0]);
		stationPts.push_back(p22);

		NavMsg::Point_t p33;
		p33.x = p11.x + p22.x-blockVerPts[0].x;
		p33.y = p11.y + p22.y-blockVerPts[0].y;
		realVertPts.push_back(p11);
		realVertPts.push_back(blockVerPts[0]);
		realVertPts.push_back(p22);
		realVertPts.push_back(p33);
	}
	else 
	{
		startIndex = -1;
		p01 = blockVerPts[0];
		p02 = blockVerPts[1];
		for (size_t i = 0; i < blockArrays.blks[id - 1].pointNum; i++)
		{
			NavMsg::Point_t p1 = blockArrays.blks[id - 1].points[i];
			NavMsg::Point_t p2 = blockArrays.blks[id - 1].points[(i + 1) % blockArrays.blks[id - 1].pointNum];

			bool flag = GetCrossPoint(p01, p02, p1, p2, x, y);
			NavMsg::Point_t crossPt;
			crossPt.x= x;
			crossPt.y= y;
			bool flag1= PointIsInline(p01, p02, crossPt);
			if (flag && flag1)
			{
				startIndex = i;
				xlog->Info("p1  %f  %f,  %f %f,  %f %f  ", p1.x, p1.y, p2.x, p2.y, x, y);
				break;
			}
		}

		NavMsg::Point_t p11;
		p11.x = x; p11.y = y;

		endIndex =-1;
		p01 = blockVerPts[2];
		p02 = blockVerPts[3];
		for (size_t i = 0; i < blockArrays.blks[id - 1].pointNum; i++)
		{
			NavMsg::Point_t p1 = blockArrays.blks[id - 1].points[i];
			NavMsg::Point_t p2 = blockArrays.blks[id - 1].points[(i + 1) % blockArrays.blks[id - 1].pointNum];

			bool flag = GetCrossPoint(p01, p02, p1, p2, x, y);
			NavMsg::Point_t crossPt;
			crossPt.x= x;
			crossPt.y= y;
			bool flag1= PointIsInline(p01, p02, crossPt);
			if (flag && flag1)
			{
				xlog->Info("p3  %f  %f,  %f %f,  %f %f ", p1.x, p1.y, p2.x, p2.y, x, y);
				endIndex = i;
				break;
			}
		}

		NavMsg::Point_t p22;
		p22.x = x; p22.y = y;

		if (startIndex == -1 || endIndex == -1)
		{
			xlog->Error(" error startIndex endIndex ");
			return false;
		}

		stationPts.push_back(p11);
		stationPts.push_back(blockVerPts[0]);
		stationPts.push_back(blockVerPts[2]);
		stationPts.push_back(p22);

		realVertPts.push_back(p11);
		realVertPts.push_back(blockVerPts[0]);
		realVertPts.push_back(blockVerPts[2]);
		realVertPts.push_back(p22);
	}

	xlog->Info(" startIndex endIndex is %d %d ",startIndex,endIndex);

	if (startIndex <= endIndex)
	{
		for (size_t i = 0; i <= startIndex; i++)
		{
			NavMsg::Point_t p1 = blockArrays.blks[id - 1].points[i];
			newPloyPoints.push_back(p1);
		}

		for (size_t i = 0; i < stationPts.size(); i++)
		{
			newPloyPoints.push_back(stationPts[i]);
		}
		
		for (size_t i = endIndex + 1; i < blockArrays.blks[id - 1].pointNum; i++)
		{
			NavMsg::Point_t p1 = blockArrays.blks[id - 1].points[i];
			newPloyPoints.push_back(p1);
		}
	}
	else if (startIndex > endIndex)
	{
		for (size_t i = endIndex + 1; i <= startIndex; i++)
		{
			newPloyPoints.push_back(blockArrays.blks[id - 1].points[i%blockArrays.blks[id - 1].pointNum]);
		}

		/* for (size_t i = startIndex + 1; i <= endIndex+blockArrays.blks[id - 1].pointNum; i++)
		{
			newPloyPoints.push_back(blockArrays.blks[id - 1].points[i%blockArrays.blks[id - 1].pointNum]);
		} */

		for (size_t i = 0; i < stationPts.size(); i++)
		{
			newPloyPoints.push_back(stationPts[i]);
			printf("i is %d \n",i);
		}
	}

	for (size_t i = 0; i < newPloyPoints.size(); i++)
	{
		xlog->Info("index pt is %f %f  ", i, newPloyPoints[i].x, newPloyPoints[i].y);
	}

	blockArrays.blks[id - 1].points = newPloyPoints;
	blockArrays.blks[id - 1].pointNum = newPloyPoints.size();

	return true;





	for (size_t i = 0; i < blockArrays.blks[id - 1].pointNum; i++)
	{
		NavMsg::Point_t p1 = blockArrays.blks[id - 1].points[i];
		NavMsg::Point_t p2 = blockArrays.blks[id - 1].points[(i + 1) % blockArrays.blks[id - 1].pointNum];

		bool flag = GetCrossPoint(p01, p02, p1, p2, x, y);
		NavMsg::Point_t crossPt;
		crossPt.x= x;
		crossPt.y= y;
		bool flag1= PointIsInline(p01, p02, crossPt);
		if (flag && flag1)
		{
			startIndex = i;
			firstCross =1;
			xlog->Info("p1  %f  %f,  %f %f,  %f %f  ", p1.x, p1.y, p2.x, p2.y, x, y);
			break;
		}
	}

	if (startIndex == -1)
	{
		p01 = blockVerPts[0];
		p02 = blockVerPts[2];
		for (size_t i = 0; i < blockArrays.blks[id - 1].pointNum; i++)
		{
			NavMsg::Point_t p1 = blockArrays.blks[id - 1].points[i];
			NavMsg::Point_t p2 = blockArrays.blks[id - 1].points[(i + 1) % blockArrays.blks[id - 1].pointNum];

			bool flag = GetCrossPoint(p01, p02, p1, p2, x, y);
			NavMsg::Point_t crossPt;
			crossPt.x= x;
			crossPt.y= y;
			bool flag1= PointIsInline(p01, p02, crossPt);
			if (flag && flag1)
			{
				startIndex = i; 
				firstCross = 2;
				xlog->Info("p2  %f  %f,  %f %f,  %f %f ", p1.x, p1.y, p2.x, p2.y, x, y);
				break;
			}
		}
		
	}

	int secondCross =0;
	NavMsg::Point_t p11;
	p11.x = x;
	p11.y = y;

	p01 = blockVerPts[2];
	p02 = blockVerPts[3];
	for (size_t i = 0; i < blockArrays.blks[id - 1].pointNum; i++)
	{
		NavMsg::Point_t p1 = blockArrays.blks[id - 1].points[i];
		NavMsg::Point_t p2 = blockArrays.blks[id - 1].points[(i + 1) % blockArrays.blks[id - 1].pointNum];

		bool flag = GetCrossPoint(p01, p02, p1, p2, x, y);

		NavMsg::Point_t crossPt;
		crossPt.x= x;
		crossPt.y= y;
		bool flag1= PointIsInline(p01, p02, crossPt);
		if (flag && flag1)
		{
			xlog->Info("p3  %f  %f,  %f %f,  %f %f ", p1.x, p1.y, p2.x, p2.y, x, y);
			endIndex = i;
			secondCross = 1;
			break;
		}
	}

	if (endIndex == -1)
	{
		p01 = blockVerPts[0];
		p02 = blockVerPts[2];
		for (size_t i = 0; i < blockArrays.blks[id - 1].pointNum; i++)
		{
			NavMsg::Point_t p1 = blockArrays.blks[id - 1].points[i];
			NavMsg::Point_t p2 = blockArrays.blks[id - 1].points[(i + 1) % blockArrays.blks[id - 1].pointNum];

			bool flag = GetCrossPoint(p01, p02, p1, p2, x, y);
			NavMsg::Point_t crossPt;
			crossPt.x= x;
			crossPt.y= y;
			bool flag1= PointIsInline(p01, p02, crossPt);
			if (flag && flag1)
			{
				xlog->Info("p4  %f  %f,  %f %f,  %f %f ", p1.x, p1.y, p2.x, p2.y, x, y);
				endIndex = i;
				secondCross = 2;
				break;
			}
		}
		
	}
	

	NavMsg::Point_t p22;
	p22.x = x;
	p22.y = y;

	xlog->Debug("startIndex is %d %d ", startIndex, endIndex);

	

	if (startIndex == -1 || endIndex == -1)
	{
		return false;
	}
	else
	{
		if (firstCross==2&&secondCross==1)
		{
			stationPts.push_back(p11);
			stationPts.push_back(blockVerPts[2]);
			stationPts.push_back(p22);

			NavMsg::Point_t p33;
			p33.x = p11.x + p22.x-blockVerPts[2].x;
			p33.y = p11.y + p22.y-blockVerPts[2].y;
			
			realVertPts.push_back(p11);
			realVertPts.push_back(blockVerPts[2]);
			realVertPts.push_back(p22);
			realVertPts.push_back(p33);
		}
		else if (firstCross==1&&secondCross==2)
		{
			stationPts.push_back(p11);
			stationPts.push_back(blockVerPts[0]);
			stationPts.push_back(p22);

			NavMsg::Point_t p33;
			p33.x = p11.x + p22.x-blockVerPts[0].x;
			p33.y = p11.y + p22.y-blockVerPts[0].y;
			realVertPts.push_back(p11);
			realVertPts.push_back(blockVerPts[0]);
			realVertPts.push_back(p22);
			realVertPts.push_back(p33);
		}
		else 
		{
			stationPts.push_back(p11);
			stationPts.push_back(blockVerPts[0]);
			stationPts.push_back(blockVerPts[2]);
			stationPts.push_back(p22);

			realVertPts.push_back(p11);
			realVertPts.push_back(blockVerPts[0]);
			realVertPts.push_back(blockVerPts[2]);
			realVertPts.push_back(p22);
		}
	}

	if (startIndex <= endIndex)
	{
		for (size_t i = 0; i <= startIndex; i++)
		{
			NavMsg::Point_t p1 = blockArrays.blks[id - 1].points[i];
			//newPloyPoints.push_back(blockArrays.blks[id - 1].points[i]);
			newPloyPoints.push_back(p1);
		}
		/* newPloyPoints.push_back(p11);
		newPloyPoints.push_back(blockVerPts[0]);
		newPloyPoints.push_back(blockVerPts[2]);
		newPloyPoints.push_back(p22); */

		for (size_t i = 0; i < stationPts.size(); i++)
		{
			newPloyPoints.push_back(stationPts[i]);
		}
		
		for (size_t i = endIndex + 1; i < blockArrays.blks[id - 1].pointNum; i++)
		{
			//newPloyPoints.push_back(blockArrays.blks[id - 1].points[i]);
			NavMsg::Point_t p1 = blockArrays.blks[id - 1].points[i];
			newPloyPoints.push_back(p1);
		}
	}
	else if (startIndex > endIndex)
	{
		for (size_t i = startIndex + 1; i <= endIndex+blockArrays.blks[id - 1].pointNum; i++)
		{
			newPloyPoints.push_back(blockArrays.blks[id - 1].points[i%blockArrays.blks[id - 1].pointNum]);
		}
		/* newPloyPoints.push_back(p11);
		newPloyPoints.push_back(blockVerPts[0]);
		newPloyPoints.push_back(blockVerPts[2]);
		newPloyPoints.push_back(p22); */
		for (size_t i = 0; i < stationPts.size(); i++)
		{
			newPloyPoints.push_back(stationPts[i]);
		}
	}

	for (size_t i = 0; i < newPloyPoints.size(); i++)
	{
		xlog->Info("index pt is %f %f  ", i, newPloyPoints[i].x, newPloyPoints[i].y);
	}

	blockArrays.blks[id - 1].points = newPloyPoints;
	blockArrays.blks[id - 1].pointNum = newPloyPoints.size();

	return true;
}



bool segment_map::rect_room_segment(std::vector<NavMsg::Point_t> blockVerPts,std::vector<int> room_ids,NavMsg::BlockArray_t& blockArrays)
{
	std::vector<NavMsg::Point_t> newPloyPoints;
	// blockArrays.blks[id-1].pointNum;
	if (blockVerPts.size() < 4)
	{
		return false;
	}

	NavMsg::Point_t p01 = blockVerPts[0];
	NavMsg::Point_t p02 = blockVerPts[1];
	int startIndex = -1;
	int endIndex = -1;
	float x = 0;
	float y = 0;
	int id=1;
	for (size_t k = 0; k < room_ids.size(); k++)
	{
		int id = room_ids[k];
	
		for (size_t j = 0; j < blockVerPts.size(); j++)
		{
			p01 = blockVerPts[j];
			p02 = blockVerPts[(j+1)%blockVerPts.size()];
			for (size_t i = 0; i < blockArrays.blks[id - 1].pointNum; i++)
			{
				NavMsg::Point_t p1 = blockArrays.blks[id - 1].points[i]; 	
				NavMsg::Point_t p2 = blockArrays.blks[id - 1].points[(i + 1) % blockArrays.blks[id - 1].pointNum];
				bool flag = GetCrossPoint(p01, p02, p1, p2, x, y);
				if (flag)
				{
					startIndex = i;
					xlog->Info("p1  %f  %f,  %f %f,  %f %f ", p1.x, p1.y, p2.x, p2.y, x, y);
					break;
				}
			}
		}
	}

	if (startIndex==-1)
	{
		p01 = blockVerPts[0];
		p02 = blockVerPts[2];
		startIndex = -1;
		int id=1;
		for (size_t k = 0; k < room_ids.size(); k++)
		{
			int id = room_ids[k];
		
			for (size_t j = 0; j < blockVerPts.size(); j++)
			{
				p01 = blockVerPts[j];
				p02 = blockVerPts[(j+1)%blockVerPts.size()];
				for (size_t i = 0; i < blockArrays.blks[id - 1].pointNum; i++)
				{
					NavMsg::Point_t p1 = blockArrays.blks[id - 1].points[i]; 	
					NavMsg::Point_t p2 = blockArrays.blks[id - 1].points[(i + 1) % blockArrays.blks[id - 1].pointNum];
					bool flag = GetCrossPoint(p01, p02, p1, p2, x, y);
					if (flag)
					{
						startIndex = i;
						xlog->Info("p1  %f  %f,  %f %f,  %f %f ", p1.x, p1.y, p2.x, p2.y, x, y);
						break;
					}
				}
			}
		}
	}
	


	NavMsg::Point_t p11;
	p11.x = x;
	p11.y = y;

	p01 = blockVerPts[2];
	p02 = blockVerPts[3];
	for (size_t i = 0; i < blockArrays.blks[id - 1].pointNum; i++)
	{
		NavMsg::Point_t p1 = blockArrays.blks[id - 1].points[i];
		NavMsg::Point_t p2 = blockArrays.blks[id - 1].points[(i + 1) % blockArrays.blks[id - 1].pointNum];

		bool flag = GetCrossPoint(p01, p02, p1, p2, x, y);
		if (flag)
		{
			xlog->Info("p1  %f  %f,  %f %f,  %f %f ", p1.x, p1.y, p2.x, p2.y, x, y);
			endIndex = i;
			break;
		}
	}

	NavMsg::Point_t p22;
	p22.x = x;
	p22.y = y;

	xlog->Debug("startIndex is %d %d ", startIndex, endIndex);

	if (startIndex == -1 || endIndex == -1)
	{
		return false;
	}

	if (startIndex <= endIndex)
	{
		for (size_t i = 0; i <= startIndex; i++)
		{
			newPloyPoints.push_back(blockArrays.blks[id - 1].points[i]);
		}
		newPloyPoints.push_back(p11);
		newPloyPoints.push_back(blockVerPts[0]);
		newPloyPoints.push_back(blockVerPts[2]);
		newPloyPoints.push_back(p22);
		for (size_t i = endIndex + 1; i < blockArrays.blks[id - 1].pointNum; i++)
		{
			newPloyPoints.push_back(blockArrays.blks[id - 1].points[i]);
		}
	}
	else if (startIndex > endIndex)
	{
		for (size_t i = startIndex + 1; i <= endIndex; i++)
		{
			newPloyPoints.push_back(blockArrays.blks[id - 1].points[i]);
		}
		newPloyPoints.push_back(p11);
		newPloyPoints.push_back(blockVerPts[0]);
		newPloyPoints.push_back(blockVerPts[2]);
		newPloyPoints.push_back(p22);
	}

	for (size_t i = 0; i < newPloyPoints.size(); i++)
	{
		xlog->Info("index pt is %f %f  ", i, newPloyPoints[i].x, newPloyPoints[i].y);
	}

	blockArrays.blks[id - 1].points = newPloyPoints;
	blockArrays.blks[id - 1].pointNum = newPloyPoints.size();

	return true;
}



bool segment_map::mergeRooms(cv::Mat &indexed_img, int id1, int id2, NavMsg::BlockArray_t &blockArrays)
{
	if (id1 > id2)  //id1 < id2 
	{
		int temp = id1;
		id1 = id2;
		id2 = temp;
	}

	xlog->Info("merge room  id is %d %d ", id1, id2);

	cv::Mat indexed_img_new = indexed_img.clone();
	cv::Mat newImg = cv::Mat(cv::Size(indexed_img.cols, indexed_img.rows), CV_8UC1, cv::Scalar(0));
	for (size_t v = 0; v < indexed_img.rows; ++v)
	{
		for (size_t u = 0; u < indexed_img.cols; ++u)
		{
			if (indexed_img.at<int>(v, u) == id2)
			{
				indexed_img_new.at<int>(v, u) = id1;
				newImg.at<unsigned char>(v, u) = 100;
			}
			else if (indexed_img.at<int>(v, u) == id1)
			{
				newImg.at<unsigned char>(v, u) = 100;
			}
		}
	}

	std::vector<std::vector<cv::Point>> contours1;
	std::vector<cv::Vec4i> hierarcy;
	std::vector<std::vector<cv::Point>> contours_poly(1);

	// cv::findContours(cv::Mat(contours), contours1, hierarcy, 0, CV_CHAIN_APPROX_NONE);
	cv::findContours(newImg, contours1, hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

	cv::Mat result;
	cv::approxPolyDP(contours1[0], contours_poly[0], 2, true);

	std::vector<cv::Point> newPloy;
	// expand_polygon(contours_poly[0],newPloy);
	newPloy = expand_polygon1(contours_poly[0], 1, -1);
	contours_poly[0] = newPloy;
	cv::Mat map_new = indexed_img.clone();
	char name[50];
	cv::drawContours(map_new, contours_poly, 0, cv::Scalar(100, 255, 255), 1, 8);
	sprintf(name, "/app/fj212/seg_map_room_merge %d.png", 0);
	cv::imwrite(name, map_new);

	// printf("corners : %d \n",contours_poly[0].size());

	std::vector<NavMsg::Point_t> newPloyPoints;
	NavMsg::Point_t prePt;
	for (size_t j = newPloy.size(); j > 0; j--)
	{
		NavMsg::Point_t pt;
		pt.x = newPloy[j - 1].x;
		pt.y = newPloy[j - 1].y;
		if (j < newPloy.size())
		{
			float dist = sqrt((pt.x - prePt.x) * (pt.x - prePt.x) + (pt.y - prePt.y) * (pt.y - prePt.y));
			if (dist <= 2)
			{
				continue;
			}
		}

		if (pt.x > indexed_img.cols || pt.y > indexed_img.rows)
		{
			continue;
		}

		prePt = pt;
		newPloyPoints.push_back(pt);
		xlog->Info("pt is %f  %f ", pt.x, pt.y);
	}

	blockArrays.blks[id1 - 1].points = newPloyPoints;
	blockArrays.blks[id1 - 1].pointNum = newPloyPoints.size();
	blockArrays.blks[id1 - 1].nextBlkIdOfEntries.clear();
	blockArrays.blks[id1 - 1].entryPoses.clear();

	std::vector<int32_t> entrysId;
	int pointNum = blockArrays.blks[id1 - 1].pointNum;
	for (size_t j = 0; j < newPloyPoints.size(); j++)
	{

		GMapping::IntPoint p0(newPloyPoints[j].x, newPloyPoints[j].y);
		GMapping::IntPoint p1(newPloyPoints[(j + 1) % pointNum].x, newPloyPoints[(j + 1) % pointNum].y);
		GMapping::GridLineTraversalLine line;
		GMapping::IntPoint *m_linePoints;
		m_linePoints = new GMapping::IntPoint[20000];
		line.points = m_linePoints;
		GMapping::GridLineTraversal::gridLine(p0, p1, &line);
		// printf("j %d \n",j);
		int freePtCnt = 0;
		bool findEntryFlag = false;
		for (int k = 0; k < line.num_points - 1; k++)
		{
			cv::Point2i pt;
			pt.x = m_linePoints[k].x;
			pt.y = m_linePoints[k].y;
			bool flag = checkIsWall(pt, 3);
			if (flag == false)
			{
				freePtCnt++;
			}
			else if (findEntryFlag)
			{
				NavMsg::Pose_t pt;
				//printf("%d \n", k - freePtCnt / 2);
				pt.px = m_linePoints[k - freePtCnt / 2].x;
				pt.py = m_linePoints[k - freePtCnt / 2].y;
				pt.pa = atan2(newPloyPoints[(j + 1) % pointNum].y - newPloyPoints[j].y, newPloyPoints[(j + 1) % pointNum].x - newPloyPoints[j].x);

				int nextEntry = -1;
				bool sucess = check_neigbor_id(pt.px, pt.py, id1, newImg, nextEntry, 30);
				if (sucess && nextEntry > 0)
				{
					entrysId.push_back(nextEntry);
					printf("entry  is %f  %f  %f \n", pt.px, pt.py, pt.pa);
					blockArrays.blks[id1 - 1].entryPoses.push_back(pt);
					break;
				}
			}
			else
			{
				freePtCnt = 0;
			}

			if (freePtCnt >= 4)
			{
				findEntryFlag = true;
			}
		}
	}

	if (entrysId.size() >= 1)
	{
		blockArrays.blks[id1 - 1].wallPose = blockArrays.blks[id1 - 1].entryPoses[0];
	}
	else
	{
		float min_dist2wall = 1000;
		int bestIndex = -1;
		std::vector<cv::Point> contours = contours1[0];
		float pa = 0;
		checkDir(contours, m_indexed_map.cols, m_indexed_map.rows, pa);
		for (size_t j = 0; j < contours.size(); j++)
		{
			float x = contours[j].x;
			float y = contours[j].y;
			float dist = sqrt((x - m_indexed_map.cols / 2) * (x - m_indexed_map.cols / 2) + (y - m_indexed_map.rows / 2) * (y - m_indexed_map.rows / 2));
			if (dist < min_dist2wall)
			{
				min_dist2wall = dist;
				bestIndex = j;
			}
		}
		if (bestIndex != -1)
		{
			blockArrays.blks[id1 - 1].wallPose.px = contours[bestIndex].x;
			blockArrays.blks[id1 - 1].wallPose.py = contours[bestIndex].y;
			blockArrays.blks[id1 - 1].wallPose.pa = pa;
		}
	}

	blockArrays.blks[id1 - 1].entryNum = entrysId.size();
	blockArrays.blks[id1 - 1].nextBlkIdOfEntries = entrysId;

	std::vector<NavMsg::Polygon_t>::iterator it = blockArrays.blks.begin();
	blockArrays.blks.erase(it + id2 - 1);
	blockArrays.blkNum = blockArrays.blks.size();
	xlog->Debug("blockArrays.blkNum is  %d \n", blockArrays.blkNum);
	for (size_t i = 0; i < blockArrays.blks.size(); i++)
	{
		if (blockArrays.blks[i].id > id2)
		{
			blockArrays.blks[i].id--;
		}
	}

	indexed_img = indexed_img_new.clone();
	// update other roomsid
	for (size_t v = 0; v < indexed_img.rows; ++v)
	{
		for (size_t u = 0; u < indexed_img.cols; ++u)
		{
			if (indexed_img_new.at<int>(v, u) > id2&&indexed_img_new.at<int>(v, u) <100)
			{
				indexed_img.at<int>(v, u) = indexed_img_new.at<int>(v, u) - 1;
			}
		}
	}

	return true;
}



 bool segment_map::checkMergeRooms(cv::Mat& indexed_img,int id1,int id2)
 {
	cv::Mat indexed_img_new = indexed_img.clone();
	cv::Mat newImg = cv::Mat(cv::Size(indexed_img.cols, indexed_img.rows), CV_8UC1, cv::Scalar(0));
	for (size_t v = 0; v < indexed_img.rows; ++v)
	{
		for (size_t u = 0; u < indexed_img.cols; ++u)
		{
			if (indexed_img.at<int>(v, u) == id1)
			{
				newImg.at<unsigned char>(v, u) = 100;
			}
		}
	}

	std::vector<std::vector<cv::Point>> contours1;
	std::vector<cv::Vec4i> hierarcy;
	std::vector<std::vector<cv::Point>> contours_poly(1);

	// cv::findContours(cv::Mat(contours), contours1, hierarcy, 0, CV_CHAIN_APPROX_NONE);
	cv::findContours(newImg, contours1, hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

	int contor_index =0;
	for (size_t j = 0; j < contours1[contor_index].size(); j++)
	{
		// contours.push_back(cv::Point2i(contours1[0][j].x, contours1[0][j].y));
		cv::Point2i pt;
		pt.x = contours1[contor_index][j].x;
		pt.y = contours1[contor_index][j].y;

		for (int v = -2 + pt.y; v <= 2 + pt.y; ++v)
		{
			for (int u = -2 + pt.x; u <= 2 + pt.x; ++u)
			{
				if (v < 0 || v >= indexed_img.rows)
					continue;

				if (u < 0 || u >= indexed_img.cols)
					continue;

				// only consider cells labeled as a room
				const int label = indexed_img.at<int>(v, u);
				if (label == id2)
				{
					return true;
				}
			}
		}
	}

	return false;
 }

int segment_map::splitRooms(int numId, NavMsg::Point_t pos1, NavMsg::Point_t pos2, NavMsg::Point_t pos3, cv::Mat &indexed_img, NavMsg::BlockArray_t &blockArrays)
{
	float h1 = pos1.x;
	float w1 = pos1.y;
	float h2 = pos2.x;
	float w2 = pos2.y;
	float k = 0;
	float b = 0;
	bool isNaf = false;
	if (abs(h2 - h1) < 1)
	{
		b = h1;
		isNaf = true;
	}
	else
	{
		k = (w2 - w1) / (h2 - h1); //# 直线斜率
		b = w1 - k * h1;		   // # 偏移量
	}
	xlog->Debug("num %d b %f %f",numId,b,k);

	int new_id = blockArrays.blks.size() + 1;

	cv::Mat indexed_img_new = indexed_img.clone();
	cv::Mat newImg1 = cv::Mat(cv::Size(indexed_img.cols, indexed_img.rows), CV_8UC1, cv::Scalar(0));
	cv::Mat newImg2 = cv::Mat(cv::Size(indexed_img.cols, indexed_img.rows), CV_8UC1, cv::Scalar(0));
	for (size_t v = 0; v < indexed_img.rows; ++v)
	{
		for (size_t u = 0; u < indexed_img.cols; ++u)
		{
			if (indexed_img.at<int>(v, u) == numId)
			{
				if (isNaf)
				{
					if (u <= b)
					{
						indexed_img_new.at<int>(v, u) = new_id;
						newImg2.at<unsigned char>(v, u) = 100;
					}
					else
						newImg1.at<unsigned char>(v, u) = 100;
				}
				else if ((u * k + b) >= v)
				{
					indexed_img_new.at<int>(v, u) = new_id;
					newImg2.at<unsigned char>(v, u) = 100;
				}
				else
					newImg1.at<unsigned char>(v, u) = 100;
			}
		}
	}

	indexed_img = indexed_img_new.clone();

	std::vector<std::vector<cv::Point>> contours1;
	std::vector<cv::Vec4i> hierarcy;
	std::vector<std::vector<cv::Point>> contours_poly(2);
	// cv::findContours(cv::Mat(contours), contours1, hierarcy, 0, CV_CHAIN_APPROX_NONE);
	cv::findContours(newImg1, contours1, hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	if (contours1.size()==0)
	{
		xlog->Error("numId is > roomNum  \n");
		return 0;
	}
	
	cv::approxPolyDP(contours1[0], contours_poly[0], 2, true);
	std::vector<cv::Point> newPloy;
	float minDist1 = INFINITY;
	float minDist2 = INFINITY;
	int closeIndex1 = -1;
	int closeIndex2 = -1;
	for (size_t i = 0; i < contours_poly[0].size(); i++)
	{
		float dist1 = sqrt((pos1.x - contours_poly[0][i].x) * (pos1.x - contours_poly[0][i].x) + (pos1.y - contours_poly[0][i].y) * (pos1.y - contours_poly[0][i].y));
		float dist2 = sqrt((pos2.x - contours_poly[0][i].x) * (pos2.x - contours_poly[0][i].x) + (pos2.y - contours_poly[0][i].y) * (pos2.y - contours_poly[0][i].y));
		if (dist1 <= 3 && dist1 < minDist1)
		{
			minDist1 = dist1;
			closeIndex1 = i;
		}

		if (dist2 <= 3 && dist2 < minDist2)
		{
			minDist2 = dist2;
			closeIndex2 = i;
		}
	}

	// expand_polygon(contours_poly[0],newPloy);
	newPloy = expand_polygon1(contours_poly[0], 1, -1);
	printf("ploy size is %d %d \n",newPloy.size(),contours_poly[1].size());
	contours_poly[0] = newPloy;
	std::vector<NavMsg::Point_t> newPloyPoints;
	NavMsg::Point_t prePt;
	for (size_t j = newPloy.size(); j > 0; j--)
	{
		NavMsg::Point_t pt;
		pt.x = newPloy[j - 1].x;
		pt.y = newPloy[j - 1].y;

		if (j < newPloy.size())
		{
			float dist = sqrt((pt.x - prePt.x) * (pt.x - prePt.x) + (pt.y - prePt.y) * (pt.y - prePt.y));
			if (dist <= 2)
			{
				continue;
			}
		}

		/* if (j == closeIndex1)
		{
			pt.x = pos1.x;
			pt.y = pos1.y;
		}
		else if (j == closeIndex2)
		{
			pt.x = pos2.x;
			pt.y = pos2.y;
		} */

		if (pt.x > indexed_img.cols || pt.y > indexed_img.rows)
		{
			continue;
		}
		prePt = pt;
		newPloyPoints.push_back(pt);
		printf("pt is %f  %f \n", pt.x, pt.y);
	}

	NavMsg::Pose_t pt;
	pt.px = pos3.x;
	pt.py = pos3.y;
	//pt.px = (pos1.x + pos2.x) / 2;
	//pt.py = (pos1.y + pos2.y) / 2;
	cv::Point2i p3;
	p3.x = pos3.x;
	p3.y = pos3.y;
	float newPa = 0;
	float minDist = INFINITY;
	bool isFindValidPt = false;
	for (int k = 0; k < newPloyPoints.size(); k++)
	{
		cv::Point2i p00(newPloyPoints[k].x, newPloyPoints[k].y);
		cv::Point2i p01(newPloyPoints[(k + 1)%newPloyPoints.size()].x, newPloyPoints[(k + 1)%newPloyPoints.size()].y);

		float dist=1000;
		cv::Point2i pd;
		bool isvalid = PointToline(p00, p01, p3,pd,dist);
		if (isvalid&&dist < minDist&&fabs(dist)<10)
		{
			minDist = dist;
			newPa = atan2(p01.y - p00.y, p01.x - p00.x);
			xlog->Info("newpa ploy %d dist %f  pa: %f  %d  %d  %d  %d  %d  %d ", k, dist, newPa, pd.x,  pd.y,p00.x, p00.y, p01.x, p01.y);

			isFindValidPt=true;
		}
	}
	if (isFindValidPt==false)
	{
		xlog->Error("newpa is error ");
	}
	
	xlog->Info("newpa is %f",newPa);
	pt.pa = newPa;

	blockArrays.blks[numId - 1].points = newPloyPoints;
	blockArrays.blks[numId - 1].pointNum = newPloyPoints.size();
	blockArrays.blks[numId - 1].nextBlkIdOfEntries.clear();
	blockArrays.blks[numId - 1].entryPoses.clear();
	blockArrays.blks[numId - 1].nextBlkIdOfEntries.push_back(new_id);
	blockArrays.blks[numId - 1].entryPoses.push_back(pt);
	blockArrays.blks[numId - 1].entryNum = blockArrays.blks[numId - 1].entryPoses.size();
	blockArrays.blks[numId - 1].wallPose = pt;

	std::vector<std::vector<cv::Point>> contours2;
	std::vector<cv::Vec4i> hierarcy2;
	cv::findContours(newImg2, contours2, hierarcy2, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	cv::approxPolyDP(contours2[0], contours_poly[1], 2, true);

	std::vector<cv::Point> newPloy2 = expand_polygon1(contours_poly[1], 1, -1);
	closeIndex1 = -1;
	closeIndex2 = -1;
	for (size_t i = 0; i < contours_poly[1].size(); i++)
	{
		float dist1 = sqrt((pos1.x - contours_poly[1][i].x) * (pos1.x - contours_poly[1][i].x) + (pos1.y - contours_poly[1][i].y) * (pos1.y - contours_poly[1][i].y));
		float dist2 = sqrt((pos2.x - contours_poly[1][i].x) * (pos2.x - contours_poly[1][i].x) + (pos2.y - contours_poly[1][i].y) * (pos2.y - contours_poly[1][i].y));
		if (dist1 <= 3 && dist1 < minDist1)
		{
			minDist1 = dist1;
			closeIndex1 = i;
		}

		if (dist2 <= 3 && dist2 < minDist2)
		{
			minDist2 = dist2;
			closeIndex2 = i;
		}
	}
	
	printf("ploy1 size is %d %d %d %d \n",newPloy2.size(),contours_poly[1].size(),closeIndex1,closeIndex2);
	contours_poly[1] = newPloy2;
	newPloyPoints.clear();
	NavMsg::Point_t prePt1;
	for (size_t j = newPloy2.size(); j > 0; j--)
	{
		NavMsg::Point_t pt;
		pt.x = newPloy2[j - 1].x;
		pt.y = newPloy2[j - 1].y;

		/* printf("pt 01 is %f  %f  %f  %f\n", pt.x, pt.y,prePt1.x,prePt1.y);
		if (j == closeIndex1)
		{
			pt.x = pos1.x;
			pt.y = pos1.y;
		}
		else if (j == closeIndex2)
		{
			pt.x = pos2.x;
			pt.y = pos2.y;
		}

		printf("pt 02 is %f  %f  %f  %f\n", pt.x, pt.y,prePt1.x,prePt1.y); */

		if (j < newPloy.size())
		{
			float dist = sqrtf((pt.x - prePt1.x) * (pt.x - prePt1.x) + (pt.y - prePt1.y) * (pt.y - prePt1.y));
			if (dist <= 2)
			{
				continue;
			}
		}

		if (pt.x > indexed_img.cols || pt.y > indexed_img.rows)
		{
			continue;
		}

		prePt1 = pt;
		newPloyPoints.push_back(pt);
		printf("pt 1 is %f  %f \n", pt.x, pt.y);
	}

	pt.pa = newPa+PI/2;

	NavMsg::Polygon_t plys = blockArrays.blks[numId - 1];
	plys.id = new_id;
	plys.points = newPloyPoints;
	plys.pointNum = newPloyPoints.size();
	plys.cleanOrder = plys.id;
	plys.cleanTime = 1;
	plys.fanRank = 0;

	plys.nextBlkIdOfEntries.clear();
	plys.entryPoses.clear();
	plys.nextBlkIdOfEntries.push_back(numId);
	plys.entryPoses.push_back(pt);
	plys.entryNum = plys.entryPoses.size();
	plys.wallPose = pt;

	plys.isRecognized =true;
	plys.cleanForbidden =0;
	plys.yModeEnable =0;

	blockArrays.blks.push_back(plys);
	blockArrays.blkNum = blockArrays.blks.size();

	return new_id;
}

NavMsg::Pose_t segment_map::findNearPoseInLine(NavMsg::Pose_t pose, NavMsg::Pose_t startPose, NavMsg::Pose_t endPose)
{
	NavMsg::Pose_t retPose = startPose;
	NavMsg::Pose_t tmpPose = startPose;
	float pa = atan2(endPose.py - startPose.py, endPose.px - startPose.px);
	float len = sqrt((endPose.px - startPose.px) * (endPose.px - startPose.px) + (endPose.py - startPose.py) * (endPose.py - startPose.py));
	float moveDis = 0;
	float step = 0.05;
	float minDis = 1000;
	float tmpDis = 1000;

	while (true)
	{
		moveDis += step;
		if (moveDis > len)
			break;

		tmpPose.px += step * cos(pa);
		tmpPose.py += step * sin(pa);
		tmpDis = sqrt((tmpPose.px - pose.px) * (tmpPose.px - pose.px) + (tmpPose.py - pose.py) * (tmpPose.py - pose.py));
		if (tmpDis < minDis)
		{
			minDis = tmpDis;
			retPose = tmpPose;
		}
	}

	return retPose;
}


 bool segment_map::findWallPose(NavMsg::Pose_t robot_pos,NavMsg::Pose_t charge_pos,NavMsg::BlockArray_t& blockArrays,int curr_Id,bool hasCharge)
 {
	if (curr_Id<=0)
	{
		return false;
	}
	
	NavMsg::Polygon_t plys = blockArrays.blks[curr_Id - 1];
	

	float newPa = 0;
	cv::Point2f newPt;
	float minDis = 1000;
	float tmpDis = 1000;
	NavMsg::Point_t ret;
	bool find_wall =false;
	for (size_t j = 0; j < plys.pointNum; j++)
	{
		NavMsg::Point_t p0= plys.points[j];
		NavMsg::Point_t p1= plys.points[(j + 1) % plys.pointNum];
		
		NavMsg::Point_t tmp = p0;

		float pa = atan2(p1.y - p0.y, p1.x - p0.x);
		float len = sqrtf((p1.x - p0.x)*(p1.x - p0.x)+(p1.y - p0.y)*(p1.y - p0.y));
		float moveDis = 0;
		float step = 0.05;		
		
		while(true)
		{
			moveDis += step;
			if(moveDis > len)
				break;
			
			tmp.x += step * cos(pa);
			tmp.y += step * sin(pa);
			tmpDis = sqrtf((tmp.x - robot_pos.px)*(tmp.x - robot_pos.px)+(tmp.y - robot_pos.py)*(tmp.y - robot_pos.py));//NavPoint(ret.x - p2.x, ret.y - p2.y).norm();
			float dist2Dock = sqrtf((tmp.x - charge_pos.px) * (tmp.x - charge_pos.px) + (tmp.y - charge_pos.py) * (tmp.y - charge_pos.py));

			if (hasCharge&&dist2Dock<0.8)
			{
				continue;
			}
			
			if(tmpDis < minDis)
			{
				//printf("%f %f,%f,%f %f \n",tmpDis,tmp.x,tmp.y,p0.x,p0.y);
				minDis = tmpDis;
				ret.x = tmp.x;
				ret.y = tmp.y;
				find_wall = true;
				newPa = atan2(plys.points[(j + 1) % plys.pointNum].y - plys.points[j].y, plys.points[(j + 1) % plys.pointNum].x - plys.points[j].x);

			}
		}
	}

	if (find_wall ==true)
	{
		xlog->Debug("new wallpose 4 is %f  %f  %f \n", ret.x, ret.y, newPa);
		blockArrays.blks[curr_Id - 1].wallPose.px = ret.x;
		blockArrays.blks[curr_Id - 1].wallPose.py = ret.y;
		blockArrays.blks[curr_Id - 1].wallPose.pa = newPa;
 	}
	else xlog->Info("not find new wallpose ");

	return true;
}


void segment_map::SetLog(XLog *newXlog)
{
	xlog = newXlog;
}
