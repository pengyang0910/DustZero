#include "DStarPlanner.h"
#include "DStarLite.h"
#include "opencv2/opencv.hpp"
#include "version.h"
#include "dstar_version.h"
#include "Msg/NavMsg/SlamMapRequest_t.hpp"
#include "Msg/NavMsg/PoseArray_t.hpp"
//#include "XCfg/xini.h"

/*----------------- DStarPlanner API  --------------*/
#define TEST_DSTAR
NavMsg::GridMap_t dstar_map_origin_temp;
std::vector<std::pair<float,float>> stuckMappoints;
std::vector<std::pair<float,float>> dynamicMappoints;
std::mutex m_mappoints;
uint8_t need_replan_flag = 0; 
void stuckMapPtSet(std::vector<std::pair<float,float>> points)
{
	for (size_t i = 0; i < points.size(); i++)
	{
		printf("stuckMapPtSet data is %f %f \n",points[i].first,points[i].second);
	}
	
}
void dynamicMapPtSet(std::vector<std::pair<float,float>> points)
{
	/* if (curr_level>=4)
	{
		return;
	} */
	
	//std::unique_lock<std::mutex> lock(m_mappoints);
	//std::unique_lock<std::mutex> lock(m_mappoints);
	for (size_t i = 0; i < points.size(); i++)
	{
		printf("dynamicMapPtSet data is %f %f \n",points[i].first,points[i].second);
		dynamicMappoints.push_back(points[i]);

	}
}

DStarPlanner_t::DStarPlanner_t(std::string _name, ini::IniFile *ptr) : BaseGPlanner_t(_name, ptr)
{
	dStarLitePt = new DStarLite();
	isLcmRunning = false;
	newMapComing = false;
	mapPt = NULL;
	gotMap = false;
	navmap = NULL;
	curMapTs = 0;
	slamMapTs = 0;
	replan_flag =false;
	curr_planRoom=0;
	last_planRoom=0;
	curr_level =1;
	goalPose.px = 0.001;
	goalPose.py = -0.002;
	is_stop_plan_flag = false;

	mainVersionInfo = XT212_DEPENDENCE_VERSION;
	subVersionInfo = DSTAR_VERSION;
 
    ini::IniFile & cfgIni = (*ptr);
	bool enLog = cfgIni.GetProperty("DstarPlanner","enLog_b").as<bool>();
	int log_level = cfgIni.GetProperty("DstarPlanner","logLevel_i").as<int>();
	test_dstar = cfgIni.GetProperty("DstarPlanner","testDstar_b").as<bool>();//false;//cf->ReadBool("test_dstar", false);
	robot_type = cfgIni.GetProperty("DstarPlanner","robotType_i").as<int>();//

	xlog = std::make_shared<XLog>(enLog);
	xlog->Initialise("dstar.log");
	xlog->SetThreshold(Type(log_level));
	xlog->Debug("dstar  mainVersion is %s ", mainVersionInfo.c_str());
	xlog->Debug("dstar subVersion is %s!", subVersionInfo.c_str());

	m_navComMapServerPt->registerDynamicMapPointsSetCB(dynamicMapPtSet);
	m_navComMapServerPt->registerStuckPointsSetCB(stuckMapPtSet);


	isLcmRunning = true;
#ifndef _WIN32
	prctl(PR_SET_NAME, "ILcmDstar");
	bindCpuCore(BIND_CPU_ID_INNER_LCM);
#endif
	mapSub = lcm.subscribe(LCM_CHANNEL_PlannerMap, &DStarPlanner_t::gridMapUpdate, this);
	mapSub->setQueueCapacity(2);

	mapInfoSub = lcm.subscribe(LCM_CHANNEL_MapInfo, &DStarPlanner_t::gridMapInfoUpdate, this);
	mapInfoSub->setQueueCapacity(2);

	blockSub = lcm.subscribe("BlockInfoFromSlam", &DStarPlanner_t::blockInfoUpdate, this);
	blockSub->setQueueCapacity(1);
#ifndef _WIN32
	prctl(PR_SET_NAME, "main");
	bindCpuCore(BIND_CPU_ID_MISC);
#endif
}

DStarPlanner_t::~DStarPlanner_t()
{
	delete dStarLitePt;
	//if (NULL != xlog)
	//	delete xlog;
}

ipoint2 DStarPlanner_t::trans2Grid(double x, double y)
{
	ipoint2 grid;

	if (mapPt == NULL||mapPt->resolution==0)
	{
		xlog->Error("trans2Grid no map");
		return grid;
	}
	
	

	double reso = mapPt->resolution;
	double width = mapPt->width;
	double height = mapPt->height;
	// grid.x = round(x / reso) + 6/0.05;//width / 2;
	// grid.y = round(y / reso) + 6/0.05;//height / 2;
	grid.x = round((x - mapPt->originPx) / reso);
	grid.y = round((y - mapPt->originPy) / reso);
	return grid;
}

NavPose DStarPlanner_t::tran2NavPose(state pt)
{
	NavPose tmpPose;

	if (mapPt == NULL||mapPt->resolution==0)
	{
		xlog->Error("tran2NavPose no map");
		return tmpPose;
	}

	double reso = mapPt->resolution;
	double width = mapPt->width;
	double height = mapPt->height;
	// tmpPose.px = round(pt.x - 6/0.05) * reso;
	// tmpPose.py = round(pt.y - 6/0.05) * reso;
	tmpPose.px = (pt.x - (round)(fabs(mapPt->originPx) / reso)) * reso;
	tmpPose.py = (pt.y - (round)(fabs(mapPt->originPy) / reso)) * reso;
	//printf("&&&&&&&&&&&&&oringin(%.3f, %.3f, %.3f)&&&&&&&&&&&&&&&n", \
      navmap->originPx, navmap->originPy, navmap->originPa);

	return tmpPose;
}

bool DStarPlanner_t::processContors()
{
	if (navmap == NULL)
	{
		return false;
	}

	/* for (size_t i = 0; i < forbidden_points.size(); i++)
	{
		int x= forbidden_points[i].x;
		int y= forbidden_points[i].y;
		dstar_map.data[(y) * (dstar_map.width) + (x)] = 100;
	} */
	

	cv::Mat img0(navmap->height, navmap->width, CV_8UC1, dstar_map.data.data()); // ATTENTION!!!: for opencv 3.1, source image is modified when using findcontour

	cv::Mat img;
	/*if (unclean_map.width>0)
	{
		cv::Mat imgZone(navmap->height, navmap->width, CV_8UC1, unclean_map.data.data()); // ATTENTION!!!: for opencv 3.1, source image is modified when using findcontour
		cv::bitwise_or(img0,imgZone,img);
	}
	else img= img0;
	*/

	img = img0;
#ifdef RK3566_BUILD
cv::imwrite("/app/fj212/dstarmap.jpg", img0);
#else 
cv::imwrite("/mnt/UDISK/fj212/dstarmap0.jpg", img0);
#endif
	

	dStarLitePt->setMap(img.data, navmap->width,navmap->height);

	uint64_t t0 = getTimeStap_us();
	cv::Mat tmpImg;
	img.copyTo(tmpImg);
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	uint64_t t1 = getTimeStap_us();
	cv::findContours(tmpImg, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
	dStarLitePt->init(0, 0, 1, 1);

	if (contours.size()==0)
	{
		xlog->Error("contours is  0 ");
	}
	

	cv::Mat imgTest(navmap->height, navmap->width, CV_8UC1, cv::Scalar(0));
	for (int i = 0; i < contours.size(); i++)
	{
		for (int j = 0; j < contours[i].size(); j++)
		{
			dStarLitePt->initCell(contours[i][j].x, contours[i][j].y, -1.0);
			imgTest.at<uchar>(contours[i][j].y, contours[i][j].x)=255;
		}
	}
	//cv::imwrite("/app/fj212/dstarmapTest.jpg", imgTest);

	return true;
}

void DStarPlanner_t::setRoomBlk(int id)
{
	std::vector<int> xVect;
	std::vector<int> yVect;
	xlog->Info("setRoomBlk %d ", id);
	if ( m_blockArrays.blkNum==0)
	{
		id =0;  
		xlog->Error("setRoomBlk no room ");
	}

	if (id<=0)
	{
		if (dstar_map_origin.width==0||dstar_map_origin.height==0)
		{
			xlog->Error("setRoomBlk is  0  error");
			return;
		}
		
		dstar_map.data.resize(dstar_map_origin.width * dstar_map_origin.height, 0);
		dstar_map.width = dstar_map_origin.width;
		dstar_map.height = dstar_map_origin.height;

		int map_width = dstar_map_origin.width;
		int map_height = dstar_map_origin.height;
		uint8_t* opData = new uint8_t[map_width*map_height];
		state pt;
		pt.x =0; pt.y = 0;
		NavPose tmpPose = tran2NavPose(pt);
		float _x_start = tmpPose.px;
		float _y_start = tmpPose.py;
		m_navComMapServerPt->getNavMapForGlobalPlanner(_x_start, _y_start, opData,  map_width,  map_height);
		for (unsigned int x = 0; x < map_width; x++)
		{
			for (unsigned int y = 0; y < map_height; y++)
			{
				dstar_map.data[(y) * (dstar_map.width) + (x)] = opData[y * (map_width) + x];
			}
		}
		xlog->Debug("map info %d %d  %d", map_width,map_height,dstar_map.width);
		return;
	}
	

	for (size_t j = 0; j < m_blockArrays.blks[id - 1].pointNum; j++)
	{
		ipoint2 st = trans2Grid(m_blockArrays.blks[id - 1].points[j].x, m_blockArrays.blks[id - 1].points[j].y);
		xVect.push_back(st.x);
		yVect.push_back(st.y);

	}

	int minY = *std::min_element(yVect.begin(), yVect.end()) - 10;
	int maxY = *std::max_element(yVect.begin(), yVect.end()) + 10;
	int minX = *std::min_element(xVect.begin(), xVect.end()) - 10;
	int maxX = *std::max_element(xVect.begin(), xVect.end()) + 10;

	if (minX < 0)
		minX = 0;
	else if (maxX > dstar_map_origin.width)
		maxX = dstar_map_origin.width;

	if (minY < 0)
		minY = 0;
	else if (maxY > dstar_map_origin.height)
		maxY = dstar_map_origin.height;

	xlog->Debug("MINMAX is  %d %d %d  %d ", minX, maxX, minY, maxY);

	// dstar_map.data=dstar_map_origin.data;
	dstar_map.data.resize(dstar_map_origin.width * dstar_map_origin.height, 0);
	dstar_map.width = dstar_map_origin.width;
	dstar_map.height = dstar_map_origin.height;
	//memset(&dstar_map.data, 3, dstar_map_origin.width * dstar_map_origin.height);

	for (unsigned int x = 0; x < dstar_map.width; x++)
	{
		for (unsigned int y = 0; y < dstar_map.height; y++)
		{
			dstar_map.data[(y) * (dstar_map.width) + (x)] = 3; //unkown
		}
	} 
	state pt;
	pt.x= minX;
	pt.y= minY;
	NavPose tmpPose = tran2NavPose(pt);
	float _x_start = tmpPose.px;
	float _y_start = tmpPose.py;
	int map_width=maxX-minX;
	int map_height=maxY-minY;
	uint8_t* opData = new uint8_t[map_width*map_height];
	m_navComMapServerPt->getNavMapForGlobalPlanner(_x_start, _y_start, opData,  map_width,  map_height);
	for (unsigned int x = 0; x < map_width; x++)
	{
		for (unsigned int y = 0; y < map_height; y++)
		{
			dstar_map.data[(y+minY) * (dstar_map.width) + (x+minX)] = opData[y * (map_width) + x];
		}
	}
}

float DStarPlanner_t::distPtToRoom(int id, int x, int y)
{
	std::vector<cv::Point> curr_ploys;
	for (size_t jj = m_blockArrays.blks[id - 1].pointNum; jj > 0; jj--)
	{
		cv::Point st;
		float x = m_blockArrays.blks[id - 1].points[jj - 1].x;
		float y = m_blockArrays.blks[id - 1].points[jj - 1].y;
		ipoint2 grid = trans2Grid(x, y);
		st.x = grid.x;
		st.y = grid.y;
		curr_ploys.push_back(st);
	}

	float dist = cv::pointPolygonTest(curr_ploys, cv::Point2f(x, y), true);
	xlog->Debug("dist is %f ", dist);

	return dist;
}

void DStarPlanner_t::plan()
{
	//ipoint2 start0 = trans2Grid(startPose.px, startPose.py);
    //int id01 = -1;
	//bool flag01 = checkRoomId(id01, start0.x, start0.y);
	//if (replan_flag&&curr_planRoom ==id01&&flag01==true)
	if (replan_flag&&need_replan_flag)
	{
		if (curr_planRoom==last_planRoom)   //replan 
		{
			//replan  add new marks
			int index =-1;
			for (size_t i = 0; i < g_roomsId.size(); i++)
			{
				if (curr_planRoom == g_roomsId[i])
				{
					index =i;
					break;
				}
			}

			if (index<0)
			{
				xlog->Error("find curr_planRoom error ");	
				//index = 0;
			}
			
			
			NavMsg::Pose_t goalPose1;
			goalPose1.px = goalPose.px;
			goalPose1.py = goalPose.py;
			
			if(index>=0)
			{
				goalPose1 = g_planPoses[index].second;
			}
			
			NavMsg::Pose_t startPose1;
			startPose1.px = startPose.px;
			startPose1.py = startPose.py;
			
			ipoint2 st1 = trans2Grid(startPose1.px, startPose1.py);
			ipoint2 ed1 = trans2Grid(goalPose1.px, goalPose1.py);
			xlog->Info("DStar Planning: stPose1(%.2f, %.2f) gPose1(%.2f, %.2f)  index: %d",
			   startPose1.px, startPose1.py, goalPose1.px, goalPose1.py,index);
			
			bool replanRes = startRePlan(st1.x, st1.y, ed1.x, ed1.y);	
			if (replanRes==false)
			{
				planPath.clear();
				//dstarPlanFlag = false;
				curr_level++;
				xlog->Info("DStar path  no  need to starPlan %d",curr_level);	
				setRoomBlk(curr_planRoom);
				int planRes = startPlan(st1.x, st1.y, ed1.x, ed1.y);
				if (planRes ==0)
				{
					xlog->Info("DStar repath  final failed ");	
					planPath.clear();
				}
				else 
				{
					std::list<state> dstar_path = dStarLitePt->getPath();
					std::vector<NavPose> curr_path;
					std::list<state>::iterator it;
					planPath.clear();
					xlog->Info("DStar path  ");
					std::string str1;
    				char str2[25];
					for (it = dstar_path.begin(); it != dstar_path.end(); ++it)
					{
						NavPose tmpPose = tran2NavPose(*it);
						sprintf(str2,"%.2f %.2f\n",tmpPose.px,tmpPose.py);
						str1 = str1 + str2; 	
						planPath.push_back(tmpPose);
						curr_path.push_back(tmpPose);
					}
					xlog->Info("%s",str1.c_str());
					
					if(index>=0)
					{
						xlog->Info("DStar path index is %d  ",index);
						for (size_t i = index+1; i < g_roomsId.size(); i++)
						{
							for (size_t j = 0; j < g_planPath[i].size(); j++)
							{
								planPath.push_back(g_planPath[i][j]);
							}	
						}
					}
				}	
			}
			else
			{
				std::list<state> dstar_path = dStarLitePt->getPath();
				std::vector<NavPose> curr_path;
				std::list<state>::iterator it;
				planPath.clear();
				xlog->Info("DStar path  ");
				std::string str1;
    			char str2[25];
				xlog->Info("tmpPose is: %d ",dstar_path.size());
				for (it = dstar_path.begin(); it != dstar_path.end(); ++it)
				{
					NavPose tmpPose = tran2NavPose(*it);
					sprintf(str2,"%.2f %.2f\n",tmpPose.px,tmpPose.py);
					//std::string str = str2; 
					str1 = str1 + str2;
					//strcat(str1,str2);  			
					planPath.push_back(tmpPose);
					curr_path.push_back(tmpPose);
				}
				xlog->Info("%s",str1.c_str());
				if(index>=0)
				{
					xlog->Info("DStar path index3 is %d  ",index);
					for (size_t i = index+1; i < g_roomsId.size(); i++)
					{
						for (size_t j = 0; j < g_planPath[i].size(); j++)
						{
							planPath.push_back(g_planPath[i][j]);
						}	
					}
				}
			}	
		}
		else   // startPlan 
		{
			int index =-1;
			for (size_t i = 0; i < g_roomsId.size(); i++)
			{
				if (curr_planRoom == g_roomsId[i])
				{
					index =i;
					break;
				}	
			}
			
			//curr_planRoom = g_roomsId[i];
			if (index<0)
			{
				setRoomBlk(last_planRoom);
				xlog->Error("find curr_planRoom error  room id is %d %d",last_planRoom,curr_planRoom);	
			}
			else if (curr_planRoom>=1)
			{
				setRoomBlk(curr_planRoom);
			}
			else
			{
				dstar_map = dstar_map_origin;
				int map_width = dstar_map_origin.width;
				int map_height = dstar_map_origin.height;
				uint8_t* opData = new uint8_t[map_width*map_height];

				state pt;
				pt.x =0; pt.y = 0;
				NavPose tmpPose = tran2NavPose(pt);
				float _x_start = tmpPose.px;
				float _y_start = tmpPose.py;
				m_navComMapServerPt->getNavMapForGlobalPlanner(_x_start, _y_start, opData,  map_width,  map_height);
				for (unsigned int x = 0; x < map_width; x++)
				{
					for (unsigned int y = 0; y < map_height; y++)
					{
						dstar_map.data[(y) * (dstar_map.width) + (x)] = opData[y * (map_width) + x];
					}
				}
			}
			
			NavMsg::Pose_t startPose1;
			startPose1.px = startPose.px;
			startPose1.py = startPose.py;
			
			NavMsg::Pose_t goalPose1;
			goalPose1.px = goalPose.px;
			goalPose1.py = goalPose.py;
			
			if(index>=0)
			{
				//startPose1 = g_planPoses[index].first;
				goalPose1 = g_planPoses[index].second;
			}
			
			xlog->Info("DStar start Planning: stPose1(%.2f, %.2f) gPose1(%.2f, %.2f)  index: %d",
			   startPose1.px, startPose1.py, goalPose1.px, goalPose1.py,index);
			ipoint2 st1 = trans2Grid(startPose1.px, startPose1.py);
			ipoint2 ed1 = trans2Grid(goalPose1.px, goalPose1.py);
			
			int planRes = startPlan(st1.x, st1.y, ed1.x, ed1.y);
			std::list<state> dstar_path = dStarLitePt->getPath();
			if (planRes == 0)
			{
				planPath.clear();
				//dstarPlanFlag = false;
			}
			else
			{
				std::vector<NavPose> curr_path;
				planPath.clear();
				std::list<state>::iterator it;
				for (it = dstar_path.begin(); it != dstar_path.end(); ++it)
				{
					NavPose tmpPose = tran2NavPose(*it);
					xlog->Debug("tmpPose 0 is  %f  %f ",tmpPose.px,tmpPose.py);
					planPath.push_back(tmpPose);
					curr_path.push_back(tmpPose);
				}

				if(index>=0)
				{
					xlog->Info("DStar path index3 is %d  ",index);
					for (size_t i = index+1; i < g_roomsId.size(); i++)
					{
						for (size_t j = 0; j < g_planPath[i].size(); j++)
						{
							planPath.push_back(g_planPath[i][j]);
						}	
					}
				}
			}
	
			last_planRoom = curr_planRoom ;
		}
		
		xlog->Info("DStar planPath  is %d  ",planPath.size());
		planPath = GetSmoothPath(planPath);
		need_replan_flag = 0;
		gotPlan = true;
		xlog->Info("DStar plan end");
		return;
	}
	
	
	std::lock_guard<std::mutex> lock(mtx);
	xlog->Info("DStar Planning: stPose(%.2f, %.2f) gPose(%.2f, %.2f) ",
			   startPose.px, startPose.py,  goalPose.px, goalPose.py);

	std::shared_ptr<NavMsg::GridMap_t> origin_map = m_navComMapServerPt->getOriginalSlamMap();
	
	if (origin_map==nullptr)
	{
		xlog->Error("dstar has got no map");
		return;
	}

	dstar_map_origin = *origin_map;

	uint64_t st0 = getTimeStap_us();

	ipoint2 st = trans2Grid(startPose.px, startPose.py);
	ipoint2 ed = trans2Grid(goalPose.px, goalPose.py);
	if (st.x<0||st.x>dstar_map_origin.width||st.y<0||st.y>dstar_map_origin.height)
	{
		xlog->Error("dstar start points error ");
		return;
	}
	if (ed.x<0||ed.x>dstar_map_origin.width||ed.y<0||ed.y>dstar_map_origin.height)
	{
		xlog->Error("dstar end points error ");
		return;
	}
	
	int id1 = -1;
	int id2 = -1;
	bool flag1 = checkRoomId(id1, st.x, st.y);
	bool flag2 = checkRoomId(id2, ed.x, ed.y);
	planPath.clear();

	bool isblkFlag = false;
	bool globalPlanGlag = false;
	bool dstarPlanFlag = true;

	if (m_blockArrays.blkNum ==1)
	{
		if (m_wallPoses.size()>0)
		{
			int min_level = 4;
			int best_id =-1;
			state best_pt;
			for (size_t i = 0; i < m_wallPoses.size(); i++)
			{	
				ipoint2 ed1 = trans2Grid(m_wallPoses[i].px, m_wallPoses[i].py);
				float angle = atan2(st.y - ed1.y, st.x - ed1.x);
				ipoint2 new_ed;
				int dis = 0;
				bool isFindBestPoint = false;

				for (int i = 1; i <= 6; i++)
				{
					new_ed.x = ed1.x + i * cos(angle);
					new_ed.y = ed1.y + i * sin(angle);
					uint8_t value= dstar_map_origin.data[new_ed.y * (dstar_map_origin.width) + new_ed.x] & 3;
					ipoint2 new_ed1;
					new_ed1.x = ed1.x + (i+1) * cos(angle);
					new_ed1.y = ed1.y + (i+1) * sin(angle);
					ipoint2 new_ed2;
					new_ed2.x = ed1.x + (i+2) * cos(angle);
					new_ed2.y = ed1.y + (i+2) * sin(angle);
					uint8_t value1= dstar_map_origin.data[new_ed1.y * (dstar_map_origin.width) + new_ed1.x] & 3;
					uint8_t value2= dstar_map_origin.data[new_ed2.y * (dstar_map_origin.width) + new_ed2.x] & 3;
					xlog->Info("new_ed.x is %d %d %d  %d %d ",new_ed.x,new_ed.y,value,value1,value2);
					
					if (value==3&&value1==3&&value2==3)
					{
						dis = i;
						isFindBestPoint = true;
						break;
					}	
				}
		
				state pt;
				if (isFindBestPoint)
				{
					pt.x= new_ed.x;
					pt.y= new_ed.y;
					NavPose pose1=tran2NavPose(pt);
					xlog->Info("goal is in the wall new goal is %d  %d  %d:  %f  %f ", dis, new_ed.x,new_ed.y,pose1.px,pose1.py);
				}
				else
				{
					pt.x= ed1.x;
					pt.y= ed1.y;
				}	

				setRoomBlk(0);
				curr_level = 1;
				int planRes = startPlan(st.x, st.y, pt.x, pt.y);
				if (planRes==0)
				{
					continue;
				}
				else 
				{
					if (planRes < min_level)
					{
						min_level = planRes;
						best_id = i;
						best_pt = pt;
					}	
				}
			}	

			if (best_id>=0)
			{
				ed = trans2Grid(m_wallPoses[best_id].px, m_wallPoses[best_id].py);
				NavPose pose1=tran2NavPose(best_pt);
				xlog->Info("best_id is %d pose is %d %d , %f %f ",best_id,best_pt.x,best_pt.y,pose1.px,pose1.py);
			}
			else
			{
				xlog->Error("best_id is none");
			}

			m_wallPoses.clear();
		}
	}
	
	if (id1!=-1 && id2!=-1)
	{
		int pointNum = m_blockArrays.blks[id1].pointNum;
		int pointNum1 = m_blockArrays.blks[id2].pointNum;
		if (pointNum == 4 && pointNum1 == 4)
		{
			flag1 = false;
			flag2 = false;
		}
	}
	
	NavMsg::Pose_t startPose1;
	NavMsg::Pose_t goalPose1;
	std::pair<NavMsg::Pose_t, NavMsg::Pose_t> curr_planPoses;
	g_roomsId.clear();
	g_planPoses.clear();
	g_planPath.clear();

	if (flag1 && flag2)
	{
		xlog->Info("room id is %d  %d ", id1, id2);

		float dist = distPtToRoom(id1, ed.x, ed.y);

		if (id1 != id2 && dist < -3)
		{
			std::vector<std::pair<int, int>> room_path;
			int roomNum = m_blockArrays.blkNum;
			std::vector<bool> visit;
			visit.resize(roomNum, false);
			getNextBlkEntry(id1, id2, room_path, visit);
			xlog->Debug("room_path is %d ", room_path.size());
			std::vector<std::pair<int, int>> new_room_path;
			for (size_t i = 0; i < room_path.size(); i++)
			{
				bool pathValid=true;
				for (size_t k = 0; k < m_blockArrays.blks[room_path[i].first - 1].nextBlkIdOfEntries.size(); k++)
				{
					if (pathValid == false)  break;
					
					for (size_t j = i+2; j < room_path.size(); j++)
					{
						if(m_blockArrays.blks[room_path[i].first - 1].nextBlkIdOfEntries[k]==room_path[j].first)
						{
							pathValid = false;
							std::pair<int, int> curr_path;
							curr_path.first= room_path[i].first;
							curr_path.second = k;
							new_room_path.push_back(curr_path);
							i=i+1;
							xlog->Debug("new_path is %d  %d", curr_path.first,curr_path.second);
							break;
						}
					}
				}

				if (pathValid == true)
				{
					new_room_path.push_back(room_path[i]);
				}	
			} 
			xlog->Debug("new_room_path is %d ", new_room_path.size());
			room_path = new_room_path;

			for (size_t i = 0; i < room_path.size(); i++)
			{
				setRoomBlk(room_path[i].first);
				curr_planRoom = room_path[i].first;
				if (i == 0)
				{
					startPose1.px = startPose.px;
					startPose1.py = startPose.py;
				}
				else
					startPose1 = m_blockArrays.blks[room_path[i - 1].first - 1].entryPoses[room_path[i - 1].second];

				goalPose1 = m_blockArrays.blks[room_path[i].first - 1].entryPoses[room_path[i].second];

				xlog->Info("startPose is %f  %f  %f  %f ", startPose1.px, startPose1.py, goalPose1.px, goalPose1.py);
				g_roomsId.push_back(room_path[i].first);
				curr_planPoses.first = startPose1;
				curr_planPoses.second = goalPose1;
				g_planPoses.push_back(curr_planPoses);	
			}

			if (room_path.size() >= 1)
			{
				startPose1 = m_blockArrays.blks[room_path[room_path.size() - 1].first - 1].entryPoses[room_path[room_path.size() - 1].second];
				goalPose1.px = goalPose.px;
				goalPose1.py = goalPose.py;

				g_roomsId.push_back(id2);
				curr_planPoses.first = startPose1;
				curr_planPoses.second = goalPose1;
				g_planPoses.push_back(curr_planPoses);	

			}
			else if (room_path.size() == 0)
			{
				g_roomsId.push_back(0);

				startPose1.px = startPose.px;
				startPose1.py = startPose.py;
				goalPose1.px = goalPose.px;
				goalPose1.py = goalPose.py;
				curr_planPoses.first = startPose1;
				curr_planPoses.second = goalPose1;
				g_planPoses.push_back(curr_planPoses);	
			}

		}
		else if (m_blockArrays.blkNum==1&&id1 == id2 )
		{
			g_roomsId.push_back(id1);
			
			float angle = atan2(st.y - ed.y, st.x - ed.x);
			ipoint2 new_ed;
			int dis=0;
			bool isFindBestPoint=false;

			for (int i = 1; i <= 6; i++)
			{
				new_ed.x = ed.x + i * cos(angle);
				new_ed.y = ed.y + i * sin(angle);
				uint8_t value= dstar_map_origin.data[new_ed.y * (dstar_map_origin.width) + new_ed.x] & 3;
				ipoint2 new_ed1;
				new_ed1.x = ed.x + (i+1) * cos(angle);
				new_ed1.y = ed.y + (i+1) * sin(angle);
				ipoint2 new_ed2;
				new_ed2.x = ed.x + (i+2) * cos(angle);
				new_ed2.y = ed.y + (i+2) * sin(angle);
				uint8_t value1= dstar_map_origin.data[new_ed1.y * (dstar_map_origin.width) + new_ed1.x] & 3;
				uint8_t value2= dstar_map_origin.data[new_ed2.y * (dstar_map_origin.width) + new_ed2.x] & 3;
				xlog->Info("new_ed.x is %d %d %d  %d %d ",new_ed.x,new_ed.y,value,value1,value2);
				
				if (value==3&&value1==3&&value2==3)
				{
					dis = i;
					isFindBestPoint = true;
					break;
				}
				
			}
			float dist = distPtToRoom(id1, ed.x, ed.y);
	
			state pt;
			if (isFindBestPoint&&fabs(dist)<=3)
			{
				pt.x= new_ed.x;
				pt.y= new_ed.y;
				NavPose pose1=tran2NavPose(pt);
				xlog->Info("goal is in the wall new goal is %d  %d  %d:  %f  %f ", dis, new_ed.x,new_ed.y,pose1.px,pose1.py);
			}
			else
			{
				pt.x= ed.x;
				pt.y= ed.y;
			}	

			NavPose tmpPose = tran2NavPose(pt);
	
			startPose1.px = startPose.px;
			startPose1.py = startPose.py;
			goalPose1.px = tmpPose.px;
			goalPose1.py = tmpPose.py;
			curr_planPoses.first = startPose1;
			curr_planPoses.second = goalPose1;


			g_planPoses.push_back(curr_planPoses);
		}
		else
		{
			g_roomsId.push_back(id1);
			startPose1.px = startPose.px;
			startPose1.py = startPose.py;
			goalPose1.px = goalPose.px;
			goalPose1.py = goalPose.py;
			curr_planPoses.first = startPose1;
			curr_planPoses.second = goalPose1;
			g_planPoses.push_back(curr_planPoses);
		}
	}
	else
	{
		g_roomsId.push_back(0);
		startPose1.px = startPose.px;
		startPose1.py = startPose.py;
		goalPose1.px = goalPose.px;
		goalPose1.py = goalPose.py;
		curr_planPoses.first = startPose1;
		curr_planPoses.second = goalPose1;
		g_planPoses.push_back(curr_planPoses);	

	}

	for (size_t i = 0; i < g_roomsId.size(); i++)
	{
		curr_planRoom = g_roomsId[i];

		if (curr_planRoom>=1)
		{
			setRoomBlk(curr_planRoom);
		}
		else
		{
			dstar_map = dstar_map_origin;
			int map_width = dstar_map_origin.width;
			int map_height = dstar_map_origin.height;
			uint8_t* opData = new uint8_t[map_width*map_height];

			state pt;
			pt.x =0; pt.y = 0;
			NavPose tmpPose = tran2NavPose(pt);
			float _x_start = tmpPose.px;
			float _y_start = tmpPose.py;
			m_navComMapServerPt->getNavMapForGlobalPlanner(_x_start, _y_start, opData,  map_width,  map_height);
			for (unsigned int x = 0; x < map_width; x++)
			{
				for (unsigned int y = 0; y < map_height; y++)
				{
					dstar_map.data[(y) * (dstar_map.width) + (x)] = opData[y * (map_width) + x];
				}
			}
			xlog->Debug("map info %d %d  %d", map_width,map_height,dstar_map.width);
		}

		startPose1 = g_planPoses[i].first;
		goalPose1 = g_planPoses[i].second;
		ipoint2 st1 = trans2Grid(startPose1.px, startPose1.py);
		ipoint2 ed1 = trans2Grid(goalPose1.px, goalPose1.py);
		xlog->Info("DStar Planning: stPose2(%.2f, %.2f) gPose(%.2f, %.2f) ",
			   startPose1.px, startPose1.py, goalPose1.px, goalPose1.py);
		curr_level = 1;
		int planRes = startPlan(st1.x, st1.y, ed1.x, ed1.y);
		if (planRes==0)
		{
			planPath.clear();
			break;
		}
		
		std::list<state> dstar_path = dStarLitePt->getPath();
		if (dstar_path.size()==0)
		{
			planPath.clear();
			dstarPlanFlag = false;
			break;
		}
		
		std::vector<NavPose> curr_path;
		std::list<state>::iterator it;
		for (it = dstar_path.begin(); it != dstar_path.end(); ++it)
		{
			NavPose tmpPose = tran2NavPose(*it);
			// xlog->Debug("tmpPose is  %f  %f ",tmpPose.px,tmpPose.py);
			planPath.push_back(tmpPose);
			curr_path.push_back(tmpPose);
		}

		g_planPath.push_back(curr_path);
		last_planRoom = curr_planRoom ;
	}
	
	xlog->Info("DStarLite Plan done, pathSize=%d ", planPath.size());
	for (int i = 0; i < planPath.size(); i++)
	{
		//xlog->Info("planPath %d is  %f  %f ", i, planPath[i].px, planPath[i].py);
	}

	if (dstarPlanFlag ==false)
	{
		planPath.clear();
		xlog->Error("dstar failed!");
	}

	planPath = GetSmoothPath(planPath);
	

	uint64_t st1 = getTimeStap_us();
	uint32_t diff_t0 = st1 - st0;

	xlog->Info("total cost time is : %d   us ", diff_t0);

	gotPlan = true; 
}


std::vector<NavPose> DStarPlanner_t::GetSmoothPath(std::vector<NavPose> path)
{
	std::vector<NavPose> planPath_out = path;
	if (planPath_out.size() >= 1)
	{
		planPath_out.push_back(goalPose);
	}

	if (planPath_out.size() > 1)
	{
		smoothPath(planPath_out, 5);
		for (int i = 0; i < planPath_out.size() - 1; i++)
		{
			/* code */
			planPath_out[i].pa = atan2(planPath_out[i + 1].py - planPath_out[i].py, planPath_out[i + 1].px - planPath_out[i].px);
		}
		planPath_out.back().pa = planPath_out[planPath_out.size() - 2].pa;
	}

	if (planPath_out.size() > 3)
	{
		for (int i = 1; i < planPath_out.size() - 1; i++)
		{
			/* code */
			planPath_out[i].pa = (planPath_out[i - 1].pa + planPath_out[i].pa + planPath_out[i + 1].pa) / 3.0;
		}
	}

	xlog->Info("planPath_out is  %d", planPath_out.size());	
	return planPath_out;
}


std::vector<NavPose> DStarPlanner_t::GetPath()
{
	std::vector<NavPose> planPath_out = planPath;
	NavMsg::PoseArray_t globalPath;
	
    std::string str1;
    char str2[25];
	xlog->Info("GetPath %d\n", planPath_out.size());
	for(int i=0;i<planPath_out.size(); i++)
	{
		NavMsg::Pose_t p0;
		p0.px=  planPath_out[i].px;
		p0.py=  planPath_out[i].py;
		p0.pa=  planPath_out[i].pa;
		globalPath.poses.push_back(p0);
		sprintf(str2,"%d:%.2f %.2f\n", i,p0.px,p0.py);
		str1 = str1 + str2;	
	}
	xlog->Info("%s",str1.c_str());

	globalPath.poseNum = globalPath.poses.size();
	lcm.publish(LCM_CHANNEL_GlobalPlanPath,&globalPath);

	return planPath_out;
}

void DStarPlanner_t::smoothPath(std::vector<NavPose> &path, int window)
{
	// return;
	if (path.size() < window)
		return;
	float distX = path[0].px - path[path.size()-1].px; 
	float distY = path[0].py - path[path.size()-1].py;

	if(fabs(distX)<=fabs(distY)&&fabs(distX)>0)
	{
		float k = fabs(distY/distX);
		if (k>4&&fabs(distX)<0.4)
		{
			float accX = 0;
			for (size_t i = 1; i < path.size(); i++)
			{
				accX += fabs(path[i].px- path[i-1].px);
			}

			if (fabs(accX-distX) <= 0.1)
			{
				for (size_t i = 1; i < path.size() - 1; i++)
				{
					path[i].px = path[0].px - distX*i/path.size();
				}
			}

			xlog->Info("smoothX is  %f  %f  %f", k, distX, accX);	
		}	
	}
	else if(fabs(distY)>0)
	{
		float k = fabs(distX/distY);
		if (k>4&&fabs(distY)<0.4)
		{
			float accY = 0;
			for (size_t i = 1; i < path.size(); i++)
			{
				accY += fabs(path[i].py- path[i-1].py);
			}

			if (fabs(accY-distY) <= 0.1)
			{
				for (size_t i = 1; i < path.size() - 1; i++)
				{
					path[i].py = path[0].py - distY*i/path.size();
				}
			}
			xlog->Info("smoothY is  %f  %f  %f ", k, distY,accY);		
		}	
	}

	//for (size_t i = 0; i < path.size() - window / 2 - 1; i++)
	for (size_t i = 0; i < path.size() - 5; i++)
	{
		/* code */
		float tmpx = 0, tmpy = 0;
		for (int j = 0; j < window; j++)
		{
			/* code */
			tmpx += path[i + j].px;
			tmpy += path[i + j].py;
		}

		path[i + window / 2].px = tmpx / window;
		path[i + window / 2].py = tmpy / window;
	}

	window =3;
	for (size_t i = path.size() - 4; i < path.size() - 2; i++)
	{
		/* code */
		float tmpx = 0, tmpy = 0;
		for (int j = 0; j < 3; j++)
		{
			/* code */
			tmpx += path[i + j].px;
			tmpy += path[i + j].py;
		}

		path[i + window / 2].px = tmpx / window;
		path[i + window / 2].py = tmpy / window;
	}


}

void DStarPlanner_t::SetPose(NavPose sPose, NavPose gPose)
{
	//printf("DStarPlanner: SetPose stPose(%.2f, %.2f, %.2f) edPose(%.2f, %.2f, %.2f)\r\n",\
          sPose.px, sPose.py, sPose.pa, gPose.px, gPose.py, gPose.pa);
	if (goalPose == gPose)
	{
		replan_flag = true;
		xlog->Info("DStarPlanner: SetPose stPose(%.2f, %.2f, %.2f) edPose(%.2f, %.2f, %.2f)\r\n",\
          sPose.px, sPose.py, sPose.pa, gPose.px, gPose.py, gPose.pa);

		ipoint2 st = trans2Grid(sPose.px, sPose.py);
		if (st.x<0||st.x>dstar_map_origin.width||st.y<0||st.y>dstar_map_origin.height)
		{
			xlog->Error("dstar start points error ");
			return;
		}
	
	}
	else
	{
		replan_flag = false; 
		curr_level = 1;
	}
	
	startPose = sPose;
	goalPose = gPose;

	done = false;
	gotPlan = false;
	init = true;
}

void DStarPlanner_t::SetWallPose(std::vector<NavPose> wallPoses)
{
	m_wallPoses = wallPoses;
	xlog->Info("SetWallPose size is  %d ",wallPoses.size());
}


void DStarPlanner_t::SetCurrRoomId(int id)
{
  //curr_planRoom =id;
    ipoint2 st = trans2Grid(startPose.px, startPose.py);
    xlog->Info("SetCurrRoomId is   %d ",id);
    int id1 = -1;
	bool flag1 = checkRoomId(id1, st.x, st.y);
	if (flag1==true)
	{
		if (id!= id1)
		{
			xlog->Error("SetCurrRoomId is error room id is %d %d ",curr_planRoom,id1);
			curr_planRoom = id1;
		}	
		else curr_planRoom = id;
	}
	else
	{
		curr_planRoom = 0;
		xlog->Error("SetCurrRoomId is error room block  is  none ");
	}
	need_replan_flag =1;
}


void DStarPlanner_t::markObs(NavPose p)
{
	if (mapPt == NULL||mapPt->resolution==0)
		return;
	ipoint2 grid = trans2Grid(p.px, p.py);

	xlog->Info("markObs  %f %f " ,p.px, p.py);
	//m_navComMapServerPt->getNavMapForGlobalPlanner(_x_start, _y_start, opData,  map_width,  map_height);
	//float x, float y,float theta, uint8_t model[NCM_OBS_MODEL_SIZE][NCM_OBS_MODEL_SIZE]
	//m_navComMapServerPt->setObsPoint(float x, float y,float theta, uint8_t model[NCM_OBS_MODEL_SIZE][NCM_OBS_MODEL_SIZE]);
	std::vector<ncm_obs_point_t> points;
	ncm_obs_point_t pt;
	pt.x= p.px;
	pt.y= p.py;
	pt.weight =100;
	//points.push_back(pt);
	
	#define ROBOT_DRADIUS_PIXEL (6)
	static uint8_t d_robot_map[2 * ROBOT_DRADIUS_PIXEL + 1][2 * ROBOT_DRADIUS_PIXEL + 1] =
		{
		0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,
	 	0,0,0,0,1,1,1,1,1,0,0,0,0,
	 	0,0,0,1,1,1,1,1,1,1,0,0,0,
	 	0,0,1,1,1,1,1,1,1,1,1,0,0,
	 	0,0,1,1,1,1,1,1,1,1,1,0,0,
	 	0,0,1,1,1,1,1,1,1,1,1,0,0,
	 	0,0,1,1,1,1,1,1,1,1,1,0,0,
	 	0,0,1,1,1,1,1,1,1,1,1,0,0,
	 	0,0,1,1,1,1,1,1,1,1,1,0,0,
	 	0,0,0,1,1,1,1,1,1,1,0,0,0,
	 	0,0,0,0,1,1,1,1,1,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,
	};

	
	for (int yy = -ROBOT_DRADIUS_PIXEL; yy <= ROBOT_DRADIUS_PIXEL; yy++)
	{
		for (int xx = -ROBOT_DRADIUS_PIXEL; xx <= ROBOT_DRADIUS_PIXEL; xx++)
		{
			if (d_robot_map[yy + ROBOT_DRADIUS_PIXEL][xx + ROBOT_DRADIUS_PIXEL])
			{
				int xs = grid.x + xx;
				int ys = grid.y + yy;
				

				// dStarLitePt->initCell(xs, ys, -1.0);
				// dstar_map.data[xs * (navmap->height) + ys] = 100;
				if (xs < 0 || xs >= dstar_map.width || ys < 0 || ys >= dstar_map.height)
				{
					continue;
				}

				state pt0;
				pt0.x= xs;
				pt0.y= ys;
				NavPose tmpPose = tran2NavPose(pt0);

				//ncm_obs_point_t pt;
				pt.x= tmpPose.px;
				pt.y= tmpPose.py;
				pt.weight =100;
				points.push_back(pt);
			}
		}
	}

	m_navComMapServerPt->setObsPoints(points);

	//m_navComMapServerPt->setObsPoint(float x, float y,float the
	//inflatMap(grid.x, grid.y);
}

void DStarPlanner_t::markObsTraj(std::vector<NavPose> obsTraj)
{
	std::vector<ncm_obs_point_t> points;
	/* for (size_t i = 0; i < obsTraj.size(); i++)
	{
		ncm_obs_point_t pt;
		pt.x= obsTraj[i].px;
		pt.y= obsTraj[i].py;
		points.push_back(pt);
	}
	m_navComMapServerPt->setObsPoints(points); */
	
	// std::vector<ipoint2> curr_obsMap;
	std::vector<int> xVect;
	std::vector<int> yVect;
	std::vector<cv::Point> curr_contour;
	std::vector<std::vector<cv::Point>> curr_contours;
	// printf("markOBs dstar ----------------\n");
	xlog->Info("obsTraj size is %d : ", obsTraj.size());
	for (size_t i = 0; i < obsTraj.size(); i++)
	{
		ipoint2 grid = trans2Grid(obsTraj[i].px, obsTraj[i].py);
		if (i>1)
		{
			float dist = sqrt((obsTraj[i].px-obsTraj[i-1].px)*(obsTraj[i].px-obsTraj[i-1].px)+(obsTraj[i].py-obsTraj[i-1].py)*(obsTraj[i].py-obsTraj[i-1].py));
			if (dist>0.5)
			{
				xlog->Error("obsTraj size is wrong");
				return;
			}
			
		}
		
		// curr_obsMap.push_back(grid);
		xVect.push_back(grid.x);
		yVect.push_back(grid.y);
		xlog->Info(" %d %d ",grid.x,grid.y);
		curr_contour.push_back(cv::Point(grid.x, grid.y));
	}

	if (obsTraj.size()<2)
	{
		xlog->Error("obsTraj size is bellow 2");
		return;
	}
	
	int len = obsTraj.size();
	float dist = sqrt((obsTraj[0].px-obsTraj[len-1].px)*(obsTraj[0].px-obsTraj[len-1].px)+(obsTraj[0].py-obsTraj[len-1].py)*(obsTraj[0].py-obsTraj[len-1].py));
	if (dist > 0.6)
	{
		ncm_obs_point_t pt;
		pt.weight =100;
		for (size_t i = 0; i < obsTraj.size(); i++)
		{
			pt.x= obsTraj[i].px;
			pt.y= obsTraj[i].py;
			points.push_back(pt);
		}
		m_navComMapServerPt->setObsPoints(points);
		xlog->Error("obsTraj  is not normal");
	}
	else
	{
		curr_contours.push_back(curr_contour);

		cv::Mat img_mask = cv::Mat(cv::Size(dstar_map_origin.width, dstar_map_origin.height), CV_8UC1, cv::Scalar(0));
		
		int minY = *std::min_element(yVect.begin(), yVect.end());
		int maxY = *std::max_element(yVect.begin(), yVect.end());
		int minX = *std::min_element(xVect.begin(), xVect.end());
		int maxX = *std::max_element(xVect.begin(), xVect.end());

		if (minX < 0)
			minX = 0;
		else if (maxX > dstar_map_origin.width)
			maxX = dstar_map_origin.width;

		if (minY < 0)
			minY = 0;
		else if (maxY > dstar_map_origin.height)
			maxY = dstar_map_origin.height;

		cv::drawContours(img_mask, curr_contours, 0, cv::Scalar(100), cv::FILLED);

		//cv::imwrite("/mnt/UDISK/fj212/img_mask.jpg", img_mask);

		for (size_t ii = minX; ii < maxX; ii++)
		{
			for (size_t jj = minY; jj < maxY; jj++)
			{
				if (img_mask.at<uchar>(jj, ii)==100)
				{
					ncm_obs_point_t pt;
					pt.weight =100;

					state pt0;
					pt0.x= ii;
					pt0.y= jj;
					NavPose tmpPose = tran2NavPose(pt0);

					pt.x= tmpPose.px;
					pt.y= tmpPose.py;
					points.push_back(pt);
					//std::cout<<pt.x<<" "<<pt.y<<std::endl;
				}

			}
		}
		//std::cout<<"Points size is "<<points.size()<<std::endl;
		xlog->Info("Points size is %d : ", points.size());
		if (points.size()>300)
		{
			xlog->Error("Points size is unvalid ");
		}
		
		m_navComMapServerPt->setObsPoints(points);
		//std::cout<<"setObsPoints done"<<std::endl;
	}
}


void DStarPlanner_t::markBumperPts(std::vector<NavPose> obsTraj)
{

	std::vector<std::pair<float, float>> dstar_points;
	for (size_t i = 0; i < obsTraj.size(); i++)
	{
		ipoint2 grid = trans2Grid(obsTraj[i].px, obsTraj[i].py);
		xlog->Info("obsTraj size is %d,  %f %f , %d %d ", i,obsTraj[i].px,obsTraj[i].py,grid.x,grid.y);
		if (grid.x>=0&&grid.x<dstar_map_origin.width&&grid.y>=0&&grid.y<dstar_map_origin.height)
		{
			bumper_points.push_back(grid);
			std::pair<float, float> tmp;
			tmp.first= obsTraj[i].px;
			tmp.second= obsTraj[i].py;
			dstar_points.push_back(tmp);
		}
			
	}

	m_navComMapServerPt->setDstarMapPoints(dstar_points);
}



typedef struct {
  int     num_points;
  ipoint2*  points;
} GridLineTraversalLine;

void gridLineCore( ipoint2 start, ipoint2 end, GridLineTraversalLine *line )
{
  int dx, dy, incr1, incr2, d, x, y, xend, yend, xdirflag, ydirflag;
  int cnt = 0;

  dx = abs(end.x-start.x); dy = abs(end.y-start.y);
  
  if (dy <= dx) {
    d = 2*dy - dx; incr1 = 2 * dy; incr2 = 2 * (dy - dx);
    if (start.x > end.x) {
      x = end.x; y = end.y;
      ydirflag = (-1);
      xend = start.x;
    } else {
      x = start.x; y = start.y;
      ydirflag = 1;
      xend = end.x;
    }
    line->points[cnt].x=x;
    line->points[cnt].y=y;
    cnt++;
    if (((end.y - start.y) * ydirflag) > 0) {
      while (x < xend) {
	x++;
	if (d <0) {
	  d+=incr1;
	} else {
	  y++; d+=incr2;
	}
	line->points[cnt].x=x;
	line->points[cnt].y=y;
	cnt++;
      }
    } else {
      while (x < xend) {
	x++;
	if (d <0) {
	  d+=incr1;
	} else {
	  y--; d+=incr2;
	}
	line->points[cnt].x=x;
	line->points[cnt].y=y;
	cnt++;
      }
    }		
  } else {
    d = 2*dx - dy;
    incr1 = 2*dx; incr2 = 2 * (dx - dy);
    if (start.y > end.y) {
      y = end.y; x = end.x;
      yend = start.y;
      xdirflag = (-1);
    } else {
      y = start.y; x = start.x;
      yend = end.y;
      xdirflag = 1;
    }
    line->points[cnt].x=x;
    line->points[cnt].y=y;
    cnt++;
    if (((end.x - start.x) * xdirflag) > 0) {
      while (y < yend) {
	y++;
	if (d <0) {
	  d+=incr1;
	} else {
	  x++; d+=incr2;
	}
	line->points[cnt].x=x;
	line->points[cnt].y=y;
	cnt++;
      }
    } else {
      while (y < yend) {
	y++;
	if (d <0) {
	  d+=incr1;
	} else {
	  x--; d+=incr2;
	}
	line->points[cnt].x=x;
	line->points[cnt].y=y;
	cnt++;
      }
    }
  }
  line->num_points = cnt;
}

void gridLine( ipoint2 start, ipoint2 end, GridLineTraversalLine *line ) {
  int i,j;
  int half;
  ipoint2 v;
  gridLineCore( start, end, line );
  if ( start.x!=line->points[0].x ||
       start.y!=line->points[0].y ) {
    half = line->num_points/2;
    for (i=0,j=line->num_points - 1;i<half; i++,j--) {
      v = line->points[i];
      line->points[i] = line->points[j];
      line->points[j] = v;
    }
  }
}

// reset goMap flag
void DStarPlanner_t::mapUpdate()
{
	std::lock_guard<std::mutex> lock(mtx);
	gotMap = false;
	newMapComing = false;
	xlog->Info("Dstar mapUpdate! ");
	// mapReq.name = "DStarPlanner";
	// mapReq.requestMap = true;
	// lcm.publish(LCM_CHANNEL_MapRequest, &mapReq);
}

void DStarPlanner_t::PreStart()
{
	planPath.clear();
	dStarLitePt->init(0, 0, 0, 0);

	xlog->Info("DStar has synchronized timer! ");

	msgThread = createBindThread(ProNavName+"DStarMsg", std::bind(&DStarPlanner_t::msgProcess, this), BIND_CPU_ID_Planner);
	//msgThread = std::thread(&DStarPlanner_t::msgProcess, this);
	// UpdateMap();
}

PlannerStatus_t DStarPlanner_t::GetStatus()
{
	// printf("DStar status:%d\n", (int)status);
	if (!gotMap)
		return PlannerStatus_t::INIT;
	else
		return status;
}

void DStarPlanner_t::requestSlamMap()
{
	gotMap = false;
	NavMsg::SlamMapRequest_t mapReq;
	mapReq.name = MapReqPlanner;
	mapReq.requestMap = true;
	xlog->Info("DStar request for map at: %u ! ", getTimeStap_us());
	lcm.publish(LCM_CHANNEL_MapRequest, &mapReq);
}

void DStarPlanner_t::gridMapInfoUpdate(const lcm::ReceiveBuffer *rbuf,
									   const std::string &channel,
									   const NavMsg::GridMapInfo_t *msg)
{
	slamMapTs = msg->updateTsUs;
}

void DStarPlanner_t::blockInfoUpdate(const lcm::ReceiveBuffer *rbuf,
									 const std::string &channel,
									 const NavMsg::BlockArray_t *msg)
{
	m_blockArrays = *msg;

	if (mapPt == NULL || dstar_map_origin.width == 0)
	{
		xlog->Error("dstar has no map error !  ");

		return;
	}

	
	ncm_obs_point_t pt;
	pt.weight =100;

	std::vector<std::vector<cv::Point>> contours;
	std::vector<std::pair<ipoint2, ipoint2>> boundMinMax;
	xlog->Info("blks.size is %d  %d  %d\n",m_blockArrays.blkNum,dstar_map_origin.width,dstar_map_origin.height);
	cv::Mat img_mask = cv::Mat(cv::Size(dstar_map_origin.width, dstar_map_origin.height), CV_8UC1, cv::Scalar(0));

	forbidden_points.clear();
				
	std::vector<ncm_obs_point_t> points;
	for (size_t i = 0; i < m_blockArrays.blkNum; i++)
	{
		for (size_t j = 0; j < m_blockArrays.blks[i].forbiddenBothNum / 8; j++)
		{
			std::vector<cv::Point> curr_ploys;
			std::vector<int> xVect;
			std::vector<int> yVect;

			for (size_t k = 0; k < 8; k += 2)
			{
				xlog->Info("forbiddenBoth zone is %f, %f  ", m_blockArrays.blks[i].forbiddenBothZones[k + j * 8], m_blockArrays.blks[i].forbiddenBothZones[k + j * 8 + 1]);
				ipoint2 st = trans2Grid(m_blockArrays.blks[i].forbiddenBothZones[k + j * 8], m_blockArrays.blks[i].forbiddenBothZones[k + j * 8 + 1]);
				curr_ploys.push_back(cv::Point(st.x, st.y));
				xlog->Info("st.xy is %d, %d   ", st.x, st.y);
				xVect.push_back(st.x);
				yVect.push_back(st.y);
			}

			std::vector<std::vector<cv::Point>> curr_contours;
			curr_contours.push_back(curr_ploys);

			{
				
				int minY = *std::min_element(yVect.begin(), yVect.end());
				int maxY = *std::max_element(yVect.begin(), yVect.end());
				int minX = *std::min_element(xVect.begin(), xVect.end());
				int maxX = *std::max_element(xVect.begin(), xVect.end());

				if (minX < 0)
					minX = 0;
				else if (maxX > dstar_map_origin.width)
					maxX = dstar_map_origin.width;

				if (minY < 0)
					minY = 0;
				else if (maxY > dstar_map_origin.height)
					maxY = dstar_map_origin.height;

				cv::drawContours(img_mask, curr_contours, 0, cv::Scalar(100), cv::FILLED);

				//cv::imwrite("/mnt/UDISK/fj212/img_mask.jpg", img_mask);

				for (size_t ii = minX; ii < maxX; ii++)
				{
					for (size_t jj = minY; jj < maxY; jj++)
					{
						if (img_mask.at<uchar>(jj, ii)==100)
						{
							ncm_obs_point_t pt;
							pt.weight =100;

							state pt0;
							pt0.x= ii;
							pt0.y= jj;
							NavPose tmpPose = tran2NavPose(pt0);

							pt.x= tmpPose.px;
							pt.y= tmpPose.py;
							points.push_back(pt);
							//std::cout<<pt.x<<" "<<pt.y<<std::endl;
							ipoint2 ip1;
							ip1.x = ii;
							ip1.y = jj;
							forbidden_points.push_back(ip1);
						}

					}
				}
				//std::cout<<"Points size is "<<points.size()<<std::endl;
				//m_navComMapServerPt->setObsPoints(points);
				//std::cout<<"setObsPoints done"<<std::endl;
			}	
		}

		cv::Mat img_mask1 = cv::Mat(cv::Size(dstar_map_origin.width, dstar_map_origin.height), CV_8UC1, cv::Scalar(0));
		for (size_t j = 0; j < m_blockArrays.blks[i].virwallNum / 8; j++)
		{
			std::vector<cv::Point> curr_ploys;
			std::vector<int> xVect;
			std::vector<int> yVect;

			for (size_t k = 0; k < 8; k += 2)
			{
				//xlog->Info("zone is %f, %f  ", m_blockArrays.blks[i].virwallZones[k + j * 8], m_blockArrays.blks[i].forbiddenBothZones[k + j * 8 + 1]);
				ipoint2 st = trans2Grid(m_blockArrays.blks[i].virwallZones[k + j * 8], m_blockArrays.blks[i].virwallZones[k + j * 8 + 1]);
				curr_ploys.push_back(cv::Point(st.x, st.y));
				xlog->Info("st.xy is %d, %d   ", st.x, st.y);
				xVect.push_back(st.x);
				yVect.push_back(st.y);
			}

			std::vector<std::vector<cv::Point>> curr_contours;
			curr_contours.push_back(curr_ploys);

			{
				
				int minY = *std::min_element(yVect.begin(), yVect.end());
				int maxY = *std::max_element(yVect.begin(), yVect.end());
				int minX = *std::min_element(xVect.begin(), xVect.end());
				int maxX = *std::max_element(xVect.begin(), xVect.end());

				if (minX < 0)
					minX = 0;
				else if (maxX > dstar_map_origin.width)
					maxX = dstar_map_origin.width;

				if (minY < 0)
					minY = 0;
				else if (maxY > dstar_map_origin.height)
					maxY = dstar_map_origin.height;

				cv::drawContours(img_mask1, curr_contours, 0, cv::Scalar(100), cv::FILLED);

				//cv::imwrite("/mnt/UDISK/fj212/img_mask.jpg", img_mask);

				for (size_t ii = minX; ii < maxX; ii++)
				{
					for (size_t jj = minY; jj < maxY; jj++)
					{
						if (img_mask1.at<uchar>(jj, ii)==100)
						{
							ncm_obs_point_t pt;
							pt.weight =100;

							state pt0;
							pt0.x= ii;
							pt0.y= jj;
							NavPose tmpPose = tran2NavPose(pt0);

							pt.x= tmpPose.px;
							pt.y= tmpPose.py;
							points.push_back(pt);
							//std::cout<<pt.x<<" "<<pt.y<<std::endl;
							ipoint2 ip1;
							ip1.x = ii;
							ip1.y = jj;
							forbidden_points.push_back(ip1);
						}

					}
				}
				//std::cout<<"Points size is "<<points.size()<<std::endl;
				//m_navComMapServerPt->setObsPoints(points);
				//std::cout<<"setObsPoints done"<<std::endl;
			}	
		}
	}

	xlog->Info("unclean map done ! forbidden_points is %d ",forbidden_points.size());

	return;
}


void DStarPlanner_t::gridMapUpdate(const lcm::ReceiveBuffer *rbuf,
								   const std::string &channel,
								   const NavMsg::GridMap_t *msg)
{
	//printf("DStar mapCB %s\r\n", msg->caller.c_str());
	if (msg->caller != MapReqPlanner)
		return;
	//	printf("***************DStarPlanner: gridMapUpdate Ts=%.4f ****************\r\n", getTimeStap_us() / 1000000.0);
	std::lock_guard<std::mutex> lock(mtx);
	if (NULL == mapPt)
		mapPt = new NavMsg::GridMap_t();

	*mapPt = *msg;
	newMapComing = true;
	curMapTs = msg->timestamp_us;
	//	printf("*******************************DStarPlanner: gridMapUpdate\r\n");
}

void DStarPlanner_t::lcmHandle()
{

	while (1)
	{
		int ret = lcm.handleTimeout(1);
		if (ret <= 0)
			break;
	}
}

/**
 * @brief
 *
 */
void DStarPlanner_t::msgProcess()
{
	//bindCpuCore(BIND_CPU_ID_Planner);
	while (isLcmRunning)
	{
		if (mapPt == NULL)
		{
			NavMsg::SlamMapRequest_t mapReq;
			mapReq.name = MapReqPlanner;
			mapReq.requestMap = true;
			lcm.publish(LCM_CHANNEL_MapRequest, &mapReq);
			sleep_us(500000);
		}
		else if (slamMapTs > curMapTs + 1000000) // 1s torlenrence
		{
			NavMsg::SlamMapRequest_t mapReq;
			mapReq.name = MapReqPlanner;
			mapReq.requestMap = true;
			lcm.publish(LCM_CHANNEL_MapRequest, &mapReq);
			curMapTs = slamMapTs;
		}
		lcmHandle();
		if (newMapComing)
		{
			// dStarLitePt->reset();
			//			printf("*********** before inflatMap: %f ************\r\n", getTimeStap_us() / 1000000.0);
			std::lock_guard<std::mutex> lock(mtx);
			//			printf("*********** lock_guard inflatMap: %f ************\r\n", getTimeStap_us() / 1000000.0);
			bindNavMap(mapPt);
			newMapComing = false;
			gotMap = true;
			//			printf("*********** after inflatMap: %f ************\r\n", getTimeStap_us() / 1000000.0);
			// printf("***************DStarPlanner: gridMapUpdate Done Ts=%.4f ****************\r\n", timerServerPt->getTs());
		}
		sleep_us(100000);
	}
}

#if 1
void DStarPlanner_t::inflatMap(int x, int y)
{
#define ROBOT_RADIUS_PIXEL (5) // 5 * 5cm = 25cm
	//  width 17.b5b cm 
	static uint8_t robot_map[2 * ROBOT_RADIUS_PIXEL + 1][2 * ROBOT_RADIUS_PIXEL + 1] =
		{
		0,0,0,0,0,0,0,0,0,0,0,
	 	0,0,0,1,1,1,1,1,0,0,0,
	 	0,0,1,1,1,1,1,1,1,0,0,
	 	0,1,1,1,1,1,1,1,1,1,0,
	 	0,1,1,1,1,1,1,1,1,1,0,
	 	0,1,1,1,1,1,1,1,1,1,0,
	 	0,1,1,1,1,1,1,1,1,1,0,
	 	0,1,1,1,1,1,1,1,1,1,0,
	 	0,0,1,1,1,1,1,1,1,0,0,
	 	0,0,0,1,1,1,1,1,0,0,0,
	 	0,0,0,0,0,0,0,0,0,0,0,
	};
	// width  31.5    15.5
	// height  14 +19.5   
	// top  16
	// botom 19.5 
#define ROBOT_DRADIUS_PIXEL (6)
	static uint8_t d_robot_map[2 * ROBOT_DRADIUS_PIXEL + 1][2 * ROBOT_DRADIUS_PIXEL + 1] =
		{
		0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,
	 	0,0,0,0,1,1,1,1,1,0,0,0,0,
	 	0,0,0,1,1,1,1,1,1,1,0,0,0,
	 	0,0,1,1,1,1,1,1,1,1,1,0,0,
	 	0,0,1,1,1,1,1,1,1,1,1,0,0,
	 	0,0,1,1,1,1,1,1,1,1,1,0,0,
	 	0,0,1,1,1,1,1,1,1,1,1,0,0,
	 	0,0,1,1,1,1,1,1,1,1,1,0,0,
	 	0,0,1,1,1,1,1,1,1,1,1,0,0,
	 	0,0,0,1,1,1,1,1,1,1,0,0,0,
	 	0,0,0,0,1,1,1,1,1,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,
	};

	if (robot_type==212)
	{
		for (int yy = -ROBOT_DRADIUS_PIXEL; yy <= ROBOT_DRADIUS_PIXEL; yy++)
		{
			for (int xx = -ROBOT_DRADIUS_PIXEL; xx <= ROBOT_DRADIUS_PIXEL; xx++)
			{
				if (d_robot_map[yy + ROBOT_DRADIUS_PIXEL][xx + ROBOT_DRADIUS_PIXEL])
				{
					int xs = x + xx;
					int ys = y + yy;
					// dStarLitePt->initCell(xs, ys, -1.0);
					// dstar_map.data[xs * (navmap->height) + ys] = 100;
					if (xs < 0 || xs >= dstar_map.width || ys < 0 || ys >= dstar_map.height)
					{
						continue;
					}

					dstar_map.data[ys * (dstar_map.width) + xs] = 100;
				}
			}
		}
	}
	else
	{
		for (int yy = -ROBOT_RADIUS_PIXEL; yy <= ROBOT_RADIUS_PIXEL; yy++)
		{
			for (int xx = -ROBOT_RADIUS_PIXEL; xx <= ROBOT_RADIUS_PIXEL; xx++)
			{
				if (robot_map[yy + ROBOT_RADIUS_PIXEL][xx + ROBOT_RADIUS_PIXEL])
				{
					int xs = x + xx;
					int ys = y + yy;
					// dStarLitePt->initCell(xs, ys, -1.0);
					// dstar_map.data[xs * (navmap->height) + ys] = 100;
					if (xs < 0 || xs >= navmap->width || ys < 0 || ys >= navmap->height)
					{
						continue;
					}

					dstar_map_origin.data[ys * (navmap->width) + xs] = 100;
				}
			}
		}
	}
}
#else
void DStarPlanner_t::inflatMap(int x, int y)
{
	double robotRaidus = 0.2; // 0.2 0.15
	int step = robotRaidus / 0.05;
	for (int xs = x - step; xs < x + step; xs++)
	{
		for (int ys = y - step; ys < y + step; ys++)
		{
			if (std::sqrt((x - xs) * (x - xs) + (y - ys) * (y - ys)) < 4) // 4.0
			{
				// updateCell(xs, ys, -1.0);
				// dstar_map.data[xs * (navmap->height) + ys] = 100;
				dstar_map.data[ys * (navmap->width) + xs] = 100;
			}
		}
	}
}
#endif

void DStarPlanner_t::inflatMapNew(int x, int y,const float ratio)
{
	double robotRaidus = 4*ratio; // 0.2 0.15
	int step = 5;
	for (int xs = x - step; xs < x + step; xs++)
	{
		for (int ys = y - step; ys < y + step; ys++)
		{
			if (xs < 0 || xs >= dstar_map.width || ys < 0 || ys >= dstar_map.height)
			{
				continue;
			}
			
			if (std::sqrt((x - xs) * (x - xs) + (y - ys) * (y - ys)) < robotRaidus) // 4.0
			{
				// updateCell(xs, ys, -1.0);
				// dstar_map.data[xs * (navmap->height) + ys] = 100;
				dstar_map.data[ys * (dstar_map.width) + xs] = 100;
			}
		}
	}
}

void DStarPlanner_t::DStaMapSave(const char *filepath)
{
	FILE *fp = fopen(filepath, "wb");
	fwrite(&dstar_map_origin_temp.width, sizeof(unsigned int), 1, fp);
	fwrite(&dstar_map_origin_temp.height, sizeof(unsigned int), 1, fp);

	fwrite(dstar_map_origin_temp.data.data(), 1, dstar_map_origin_temp.width * dstar_map_origin_temp.height, fp);
	fclose(fp);
}

bool DStarPlanner_t::DStaMapLoad(const char *filepath)
{
	FILE *fp = fopen(filepath, "rb");
	if (fp == NULL)
		return false;

	int width, height;
	fread(&width, sizeof(unsigned int), 1, fp);
	fread(&height, sizeof(unsigned int), 1, fp);
	/*width = height = 320;*/

	if (mapPt)
	{
		mapPt->data.clear();
		delete mapPt;
	}
	mapPt = new NavMsg::GridMap_t();
	navmap = mapPt;
	navmap->data.resize(width * height, 0);
	navmap->width = width;
	navmap->height = height;

	dstar_map_origin = *navmap;
	dstar_map = dstar_map_origin;

	fread(dstar_map_origin.data.data(), 1, width * height, fp);
	fclose(fp);

	return true;
}

void DStarPlanner_t::bindNavMap(NavMsg::GridMap_t *_navmap)
{
	navmap = _navmap;
	//dstar_map = *_navmap;
	dstar_map_origin = *_navmap;

	//	printf("-----------xu dstar -----------\n");
	/*for(unsigned int x = 0; x < navmap->width; x++)
	  for(unsigned int y = 0; y < navmap->height; y++)
	  {
		// -1 untravel;
		// 0 means uncertain
		// 1 occupied
		//if( 100 == navmap->data[x + (navmap->width) * y])
		if( 100 == navmap->data[x * (navmap->height) + y])
		{
		  navmap->data[x * (navmap->height) + y] = 1;
		  //updateCell(x, y, -1);
		  //std::cout<<"Obs["<<np.px<<","<<np.py<<"]"<<std::endl;
		}
		else if(-1  == navmap->data[x * (navmap->height) + y])
		{
		  navmap->data[x * (navmap->height) + y] = 0;
		}
		else if(0 == navmap->data[x * (navmap->height) + y])
		{
		  navmap->data[x * (navmap->height) + y] = -1;
		}

	  }*/

	dStarLitePt->init(0, 0, 1, 1);
	for (unsigned int x = 0; x < navmap->width; x++)
	{
		for (unsigned int y = 0; y < navmap->height; y++)
		{
			// -1 untravel;
			// 0 means uncertain
			// 1 occupied
			dstar_map_origin.data[y * (navmap->width) + x] = dstar_map_origin.data[y * (navmap->width) + x] & 7;
			if (1 == dstar_map_origin.data[x + (navmap->width) * y])
			// if( 100 == navmap->data[x * (navmap->height) + y])
			{
				// updateCell(x, y, -1);
				// printf("(%d,%d), ", x, y);
				// printf("*********** before inflatMap: %f ************\r\n", getTs());
				inflatMap(x, y);
				// std::cout<<"Obs["<<np.px<<","<<np.py<<"]"<<std::endl;
				// printf("*********** after inflatMap: %f ************\r\n", getTs());
			}
			else if (7 == dstar_map_origin.data[x + (navmap->width) * y])  //free 
			{
				dstar_map_origin.data[x + (navmap->width) * y] = 0;
			}
			else
			{
				dstar_map_origin.data[x + (navmap->width) * y] = -1;  // unk
			}

			if (unclean_map.height == dstar_map_origin.height)
			{
				if (unclean_map.data[y * (dstar_map_origin.width) + x] == 100)
				{
					// inflatMap(x, y);
					dstar_map_origin.data[y * (navmap->width) + x] = 100;
				}
			}
		}
	}
	// pubInflatMap();

	for (size_t i = 0; i < obsMap_points.size(); i++)
	{
		int x = obsMap_points[i].x;
		int y = obsMap_points[i].y;
		dstar_map_origin.data[y * (navmap->width) + x] = 100;
	}

	if (test_dstar)
	{
		char name[50];
		sprintf(name, "/tmp/map.ds");
		//DStaMapSave(name);
	}
}

void DStarPlanner_t::pubInflatMap()
{
	lcm.publish(LCM_CHANNEL_DSTARMAP, &dstar_map);
}

void DStarPlanner_t::areaClear(int x, int y, float radius)
{
	int step = radius / 0.05;
	// std::cout << "step: " << step << " "<<navmap->width<<std::endl;
	xlog->Info("clear   %d  %d    %d  %d", navmap->height,navmap->width,dstar_map.height,dstar_map.width);
	for (unsigned int xs = x - step; xs <= x + step; xs++)
	{
		for (unsigned int ys = y - step; ys <= y + step; ys++)
		{
			if (ys < 0 || ys >= navmap->height || xs < 0 || xs >= navmap->width)
			{
				continue;
			}

			if (std::sqrt((xs - x) * (xs - x) + (ys - y) * (ys - y)) < 5) // 2.8
			{
				dstar_map.data[ys * (navmap->width) + xs] = 0;
				dStarLitePt->updateCell(xs, ys, 1);
			}
		}
	}
	//xlog->Info("clear done  %d %d  step is %d   %d  %d ", x, y, step,navmap->height,navmap->width);
	// std::cout<<"size: "<<clearedAreaVec.size()<<std::endl;
}

void  DStarPlanner_t::areaClearCell(int x, int y, float radius)
{
	int step = radius / 0.05;
	// std::cout << "step: " << step << " "<<navmap->width<<std::endl;
	xlog->Info("clear   %d  %d    %d  %d", navmap->height,navmap->width,dstar_map.height,dstar_map.width);
	for (unsigned int xs = x - step; xs <= x + step; xs++)
	{
		for (unsigned int ys = y - step; ys <= y + step; ys++)
		{
			if (ys < 0 || ys >= navmap->height || xs < 0 || xs >= navmap->width)
			{
				continue;
			}

			if (std::sqrt((xs - x) * (xs - x) + (ys - y) * (ys - y)) < 5) // 2.8
			{
				dstar_map.data[ys * (dstar_map.width) + xs] = 0;
				dStarLitePt->updateCell(xs, ys, 1);
			}
		}
	}
	//xlog->Info("clear done  %d %d  step is %d   %d  %d ", x, y, step,navmap->height,navmap->width);
	// std::cout<<"size: "<<clearedAreaVec.size()<<std::endl;
}

int DStarPlanner_t::startPlan(double sx, double sy, double ex, double ey)
{
	if (0 == dstar_map.width)
	{
		// std::cout << "ERROR! DStartLite: navmap is NULL!" << std::endl;
		xlog->Error("ERROR! DStartLite: navmap is NULL! ");
		return 0;
	}
	
	if(sx<0||sx>dstar_map.width ||sy<0||sy>dstar_map.height)
	{
		xlog->Error("ERROR! sx unvalid ");
		return 0;
	}
	
	if(ex<0||ex>dstar_map.width ||ey<0||ey>dstar_map.height)
	{
		xlog->Error("ERROR! ex unvalid ");
		return 0;
	}

//std::lock_guard<std::mutex> lock(m_mappoints);
	dynamicMappoints.clear();
	stuckMappoints.clear();

	if(curr_level==1)
	{
		bumper_points.clear();
		m_navComMapServerPt->clearDstarMap();
	}
	
	//m_navComMapServerPt->clearDstarMap();
	
	uint64_t st00 = getTimeStap_us();
	
	NavMsg::GridMap_t dstar_map_tmp = dstar_map;
	cv::Mat forbidden_img = cv::Mat(cv::Size(dstar_map.width, dstar_map.height), CV_8UC1, cv::Scalar(0));
	for (size_t i = 0; i < forbidden_points.size(); i++)
	{
		int x= forbidden_points[i].x;
		int y= forbidden_points[i].y;
		dstar_map_tmp.data[(y) * (dstar_map.width) + (x)] = 2;  //forbidden 
		forbidden_img.at<unsigned char>(y,x) =1;
	}
	for (size_t i = 0; i < unreachRegions.size(); i++)
	{
		for (size_t j = 0; j < unreachRegions[i].size(); j++)
		{
			ipoint2 pt = unreachRegions[i][j];
			dstar_map_tmp.data[(pt.y) * (dstar_map.width) + (pt.x)] = 2;  //unreachable  
		}
		
	}
	
	ipoint2 st;
	ipoint2 ed;
	st.x = sx;
	st.y = sy;
	ed.x = ex;
	ed.y = ey;
	int stuckCnt = 0;
	int dynamiCnt = 0;

	xlog->Info("**********(%d, %d),(%d, %d),(%.2f, %.2f),(%.2f, %.2f)***********\n",
				st.x, st.y, ed.x, ed.y, sx, sy, ex, ey);
	uint64_t st0 = getTimeStap_us();

	uint8_t value= dstar_map_tmp.data[ed.y * (dstar_map.width) + ed.x] & 3;
	if (value ==3)
	{
		xlog->Error("goal pt is unkown ");
		return 0;
	}
	else if (value ==1)
	{
		xlog->Error("goal pt is  obs ");
		return 0;
	}
	else if (value ==2)
	{
		xlog->Error("goal pt is unreachable ");
		return 0;
	}
	int resultError =0;
	
	if (curr_level<=1)
	{
		cv::Mat original_img = cv::Mat(cv::Size(dstar_map.width, dstar_map.height), CV_8UC1, cv::Scalar(0));
		for (int y = 0; y < dstar_map.height; y++)
		{
			for (int x = 0; x < dstar_map.width; x++)
			{
				uint8_t stuckValue= dstar_map_tmp.data[y * (dstar_map.width) + x] & 4;
				uint8_t value= dstar_map_tmp.data[y * (dstar_map.width) + x] & 3;
				uint8_t dynamicValue= dstar_map_tmp.data[y * (dstar_map.width) + x] & 8;
				stuckValue = stuckValue >>2;
				dynamicValue = dynamicValue >>3;
				if (stuckValue==1)
				{
					inflatMapNew(x,y,0.8);
					stuckCnt++;
				}
				else if (dynamicValue==1)
				{
					inflatMapNew(x,y,0.5);
					dynamiCnt++;
				}
				else
				{	
					if (1 == value )   //obs
					{
						inflatMapNew(x,y,1.1);
						original_img.at<uchar>(y, x) = 100;
					}
					else if (3 == value)    // unknown 
					{
						dstar_map.data[x + (dstar_map.width) * y] = -1;
					}
					else if (0 == value)  //free 
					{
						original_img.at<uchar>(y, x) = 255;
					}
					else   // 2  forbidden 
					{
						dstar_map.data[x + (dstar_map.width) * y] = -1;
					}
					
					// else  free 0
				}
			}
		}
		
		xlog->Info("stuckCnt is %d  %d",stuckCnt,dynamiCnt);
		cv::imwrite("/app/fj212/dstarmap_origin.jpg", original_img);

		// if the distance between start and goal is more than 7*0.05m, clear start and goal
		if (std::sqrt((st.x - ed.x) * (st.x - ed.x) + (st.y - ed.y) * (st.y - ed.y)) > 10)
		{
			xlog->Info("********dstarplan dis > 0.5 ******* ");
			areaClear(st.x, st.y, 0.2); // 0.3
			if (forbidden_img.at<unsigned char>(ed.y,ed.x) ==1)  
			{
				xlog->Error("********goal point is in forbidden ******* ");
				return 0;
			}
			else areaClear(ed.x, ed.y, 0.2);
		}
		else if (std::sqrt((st.x - ed.x) * (st.x - ed.x) + (st.y - ed.y) * (st.y - ed.y)) > 7)
		{
			xlog->Info("********dstarplan dis > 0.35 ******* ");
			areaClear(st.x, st.y, 0.15);
			if (forbidden_img.at<unsigned char>(ed.y,ed.x) ==0)
				areaClear(ed.x, ed.y, 0.2);
			else 
			{
				xlog->Error("********goal point is in forbidden ******* ");
				return 0;
			}
		}
		else
		{
			xlog->Info("********an dis < 0.35 ******* ");
			areaClear(st.x, st.y, 0.15);
			if (forbidden_img.at<unsigned char>(ed.y,ed.x) ==0)
				areaClear(ed.x, ed.y, 0.15);
			else 
			{
				xlog->Error("********goal point is in forbidden ******* ");
				return 0;
			}
		}
		xlog->Info("process contors ");
		if (processContors() == false)
			return 0;

#ifdef TEST_DSTAR 
		/* dstar_map_origin_temp = dstar_map;
		char name[50];
		sprintf(name, "/tmp/map.ds");
		DStaMapSave(name);  */
#endif

		dStarLitePt->draw();
		dStarLitePt->updateStart(st.x, st.y);
		dStarLitePt->updateGoal(ed.x, ed.y);
		dStarLitePt->draw();
		bool success = dStarLitePt->replan(resultError);
		uint64_t st3 = getTimeStap_us();
		uint32_t diff_t2 = st3 - st00;
		xlog->Info("startPlan first cost time is : %d  us   %d ",diff_t2,resultError);
		if (success)
		{
			xlog->Info("replan success first trys");	
			return 1;
		}
		else 
			curr_level=2;
	}

	if (curr_level==2)
	{
		dstar_map.data = dstar_map_tmp.data;
		for (int y = 0; y < dstar_map.height; y++)
		{
			for (int x = 0; x < dstar_map.width; x++)
			{
				uint8_t stuckValue= dstar_map_tmp.data[y * (dstar_map.width) + x] & 4;
				uint8_t value= dstar_map_tmp.data[y * (dstar_map.width) + x] & 3;
				uint8_t dynamicValue= dstar_map_tmp.data[y * (dstar_map.width) + x] & 8;
				stuckValue = stuckValue >>2;
				dynamicValue = dynamicValue >>3;
				if (stuckValue==1)
				{
					inflatMapNew(x,y,0.6);
					stuckCnt++;
				}
				else if (dynamicValue==1)
				{
					inflatMapNew(x,y,0.3);
				}
				else
				{	
					if (1 == value )   //obs
					{
						inflatMapNew(x,y,0.8);
					}
					else if (3 == value)    // unknown 
					{
						dstar_map.data[x + (dstar_map.width) * y] = -1;
					}	
					else if (0 == value)  //free // else  free 0
					{
						//dstar_map.data[x + (dstar_map.width) * y] = 0;
					}
					else if (2 == value)  // 2  forbidden 
					{
						dstar_map.data[x + (dstar_map.width) * y] = -1;
					}
				}
			}
		}

		// if the distance between start and goal is more than 7*0.05m, clear start and goal
		if (std::sqrt((st.x - ed.x) * (st.x - ed.x) + (st.y - ed.y) * (st.y - ed.y)) > 10)
		{
			xlog->Info("********dstarplan dis > 0.5 ******* ");
			areaClear(st.x, st.y, 0.2); // 0.3
			if (forbidden_img.at<unsigned char>(ed.y,ed.x) ==0)
				areaClear(ed.x, ed.y, 0.2);
			else 
			{
				xlog->Error("********goal point is in forbidden ******* ");
				return 0;
			}
		}
		else if (std::sqrt((st.x - ed.x) * (st.x - ed.x) + (st.y - ed.y) * (st.y - ed.y)) > 7)
		{
			xlog->Info("********dstarplan dis > 0.35 ******* ");
			areaClear(st.x, st.y, 0.15);
			if (forbidden_img.at<unsigned char>(ed.y,ed.x) ==0)
				areaClear(ed.x, ed.y, 0.2);
			else 
			{
				xlog->Error("********goal point is in forbidden ******* ");
				return 0;
			}
		}
		else
		{
			xlog->Info("********an dis < 0.35 ******* ");
			areaClear(st.x, st.y, 0.15);
			if (forbidden_img.at<unsigned char>(ed.y,ed.x) ==0)
				areaClear(ed.x, ed.y, 0.15);
			else 
			{
				xlog->Error("********goal point is in forbidden ******* ");
				return 0;
			}
		}

#ifdef TEST_DSTAR 
		/* dstar_map_origin_temp = dstar_map;
		char name[50];
		sprintf(name, "/tmp/map2.ds");
		DStaMapSave(name); */
#endif 

		xlog->Info("process contors 1");
		if (processContors() == false)
			return 0;

		dStarLitePt->updateStart(st.x, st.y);
		dStarLitePt->updateGoal(ed.x, ed.y);
		bool success = dStarLitePt->replan(resultError);
		uint64_t st3 = getTimeStap_us();
		uint32_t diff_t2 = st3 - st00;
		xlog->Info("startPlan second cost time is : %d us",diff_t2);
		if (success)
		{
			xlog->Info("replan success second trys");
			return 2;
		}
		else 
		{
			curr_level =3;
			xlog->Info("replan failed second trys! error is :%d ",resultError);
		}
			
	}

	if (curr_level==3)
	{
		dstar_map.data = dstar_map_tmp.data;
		for (int y = 0; y < dstar_map.height; y++)
		{
			for (int x = 0; x < dstar_map.width; x++)
			{
				uint8_t stuckValue= dstar_map_tmp.data[y * (dstar_map.width) + x] & 4;
				uint8_t value= dstar_map_tmp.data[y * (dstar_map.width) + x] & 3;
				uint8_t dynamicValue= dstar_map_tmp.data[y * (dstar_map.width) + x] & 8;
				stuckValue = stuckValue >>2;
				dynamicValue = dynamicValue >>3;
				if (stuckValue==1)
				{
					inflatMapNew(x,y,0.4);
					stuckCnt++;
				}
				else if (dynamicValue==1)
				{
					dstar_map.data[x + (dstar_map.width) * y] = 100;
					//inflatMapNew(x,y,0.2);
				}
				else
				{	
					if (1 == value )   //obs
					{
						inflatMapNew(x,y,0.5);
					}
					else if (3 == value)    // unknown 
					{
						dstar_map.data[x + (dstar_map.width) * y] = -1;
					}
					else if (2 == value)  // 2  forbidden 
					{
						dstar_map.data[x + (dstar_map.width) * y] = -1;
					}
					// else  free 0
				}
			}
		}

		// if the distance between start and goal is more than 7*0.05m, clear start and goal
		if (std::sqrt((st.x - ed.x) * (st.x - ed.x) + (st.y - ed.y) * (st.y - ed.y)) > 10)
		{
			xlog->Info("********dstarplan dis > 0.5 ******* ");
			areaClear(st.x, st.y, 0.2); // 0.3
			if (forbidden_img.at<unsigned char>(ed.y,ed.x) ==0)
				areaClear(ed.x, ed.y, 0.2);
			else 
			{
				xlog->Error("********goal point is in forbidden ******* ");
				return 0;
			}
		}
		else if (std::sqrt((st.x - ed.x) * (st.x - ed.x) + (st.y - ed.y) * (st.y - ed.y)) > 7)
		{
			xlog->Info("********dstarplan dis > 0.35 ******* ");
			areaClear(st.x, st.y, 0.15);
			if (forbidden_img.at<unsigned char>(ed.y,ed.x) ==0)
				areaClear(ed.x, ed.y, 0.2);
			else 
			{
				xlog->Error("********goal point is in forbidden ******* ");
				return 0;
			}
			
		}
		else
		{
			xlog->Info("********an dis < 0.35 ******* ");
			areaClear(st.x, st.y, 0.15);
			if (forbidden_img.at<unsigned char>(ed.y,ed.x) ==0)
				areaClear(ed.x, ed.y, 0.15);
			else 
			{
				xlog->Error("********goal point is in forbidden ******* ");
				return 0;
			}
		}

		if (test_dstar)
		{
			dstar_map_origin_temp = dstar_map;
			char name[50];
			sprintf(name, "/tmp/map3-%d_%d_%d_%d.ds", st.x, st.y, ed.x, ed.y);
			DStaMapSave(name);
		}
		
		xlog->Info("process contors 2");
		if (processContors() == false)
			return 0;

		dStarLitePt->updateStart(st.x, st.y);
		dStarLitePt->updateGoal(ed.x, ed.y);
		bool success = dStarLitePt->replan(resultError);
		uint64_t st3 = getTimeStap_us();
		uint32_t diff_t2 = st3 - st00;
		xlog->Info("startPlan third cost time is : %d  us ",diff_t2);
		if (success == true)
		{
			xlog->Info("startPlan sucess 3rd trys");
			return true;
		}
		else 
		{
			curr_level = 4;
			xlog->Info("startPlan failed 3rd trys! error is :%d ",resultError);
		}
	}

	if (curr_level==4)
	{
		dstar_map.data = dstar_map_tmp.data;
		for (int y = 0; y < dstar_map.height; y++)
		{
			for (int x = 0; x < dstar_map.width; x++)
			{
				uint8_t stuckValue= dstar_map_tmp.data[y * (dstar_map.width) + x] & 4;
				uint8_t value= dstar_map_tmp.data[y * (dstar_map.width) + x] & 3;
				uint8_t dynamicValue= dstar_map_tmp.data[y * (dstar_map.width) + x] & 8;
				stuckValue = stuckValue >>2;
				dynamicValue = dynamicValue >>3;
				if (stuckValue==1)
				{
					//inflatMapNew(x,y,0.3);
					dstar_map.data[x + (dstar_map.width) * y] = 100;
					stuckCnt++;
				}
				else if (dynamicValue==1)
				{
					//dstar_map.data[x + (dstar_map.width) * y] = 100;
					//inflatMapNew(x,y,0.2);
				}
				else
				{	
					if (1 == value )   //obs
					{
						inflatMapNew(x,y,0.3);
						dstar_map.data[x + (dstar_map.width) * y] = 100;
					}
					else if (3 == value)    // unknown 
					{
						dstar_map.data[x + (dstar_map.width) * y] = -1;
					}
					else if (2 == value)  // 2  forbidden 
					{
						dstar_map.data[x + (dstar_map.width) * y] = -1;
					}
					// else  free 0
				}
			}
		}

		// if the distance between start and goal is more than 7*0.05m, clear start and goal
		if (std::sqrt((st.x - ed.x) * (st.x - ed.x) + (st.y - ed.y) * (st.y - ed.y)) > 10)
		{
			xlog->Info("********dstarplan dis > 0.5 ******* ");
			areaClear(st.x, st.y, 0.2); // 0.3
			if (forbidden_img.at<unsigned char>(ed.y,ed.x) ==0)
				areaClear(ed.x, ed.y, 0.2);
			else 
			{
				xlog->Error("********goal point is in forbidden ******* ");
				return 0;
			}
		}
		else if (std::sqrt((st.x - ed.x) * (st.x - ed.x) + (st.y - ed.y) * (st.y - ed.y)) > 7)
		{
			xlog->Info("********dstarplan dis > 0.35 ******* ");
			areaClear(st.x, st.y, 0.15);
			if (forbidden_img.at<unsigned char>(ed.y,ed.x) ==0)
				areaClear(ed.x, ed.y, 0.2);
			else 
			{
				xlog->Error("********goal point is in forbidden ******* ");
				return 0;
			}
		}
		else
		{
			xlog->Info("********an dis < 0.35 ******* ");
			areaClear(st.x, st.y, 0.15);
			if (forbidden_img.at<unsigned char>(ed.y,ed.x) ==0)
				areaClear(ed.x, ed.y, 0.15);
			else 
			{
				xlog->Error("********goal point is in forbidden ******* ");
				return 0;
			}
		}

		if (test_dstar)
		{
			dstar_map_origin_temp = dstar_map;
			char name[50];
			sprintf(name, "/tmp/map4-%d_%d_%d_%d.ds", st.x, st.y, ed.x, ed.y);
			DStaMapSave(name);
		}
		
		xlog->Info("process contors 3");
		if (processContors() == false)
			return 0;

		dStarLitePt->updateStart(st.x, st.y);
		dStarLitePt->updateGoal(ed.x, ed.y);
		bool success = dStarLitePt->replan(resultError);
		if (success ==false)
		{
			xlog->Error("startPlan failed 4th trys! error is :%d ",resultError);
			std::vector<ipoint2> connectRegion;
			if (resultError==2)
			{
				// if(searchConnetRegion(connectRegion,ed.x,ed.y))
				// {
				// 	unreachRegions.push_back(connectRegion);
				// 	xlog->Info("find connectRegion size is %d",connectRegion.size());
				// }  
			}

		//	system("aplay /app/fj212/resource/robot_voice/8.wav");
			return 0;
		}
	}

	if (curr_level>4)
	{
		return 0;
	}
	

	uint64_t st3 = getTimeStap_us();
	uint32_t diff_t2 = st3 - st00;
	xlog->Info("startPlan cost time is : %d  us ",diff_t2);

	return 4;
}




bool DStarPlanner_t::startRePlan(double sx, double sy, double ex, double ey)
{
	xlog->Info("********dstarReplan******* ");
	if (0 == dstar_map.width)
	{
		// std::cout << "ERROR! DStartLite: navmap is NULL!" << std::endl;
		xlog->Error("ERROR! DStartLite: navmap is NULL! ");
		return false;
	}
	
	if(sx<0||sx>dstar_map.width ||sy<0||sy>dstar_map.height)
	{
		xlog->Error("ERROR! sx unvalid ");
		return false;
	}
	
	if(ex<0||ex>dstar_map.width ||ey<0||ey>dstar_map.height)
	{
		xlog->Error("ERROR! ex unvalid ");
		return false;
	}

	ipoint2 st;
	ipoint2 ed;
	st.x = sx;
	st.y = sy;
	ed.x = ex;
	ed.y = ey;
	int resultError =0;

	xlog->Info("**********(%d, %d),(%d, %d),(%.2f, %.2f),(%.2f, %.2f)***********\n",
			   st.x, st.y, ed.x, ed.y, sx, sy, ex, ey);
	uint64_t st0 = getTimeStap_us();
	cv::Mat mark_img = cv::Mat(cv::Size(dstar_map.width, dstar_map.height), CV_8UC1, cv::Scalar(0));

	for (size_t i = 0; i < bumper_points.size(); i++)
	{
		int xs= bumper_points[i].x;
		int ys= bumper_points[i].y;
	
		printf("{%d , %d},\n",xs,ys);
		/* if (dstar_map.data[ys * (dstar_map.width) + xs] != 100)
		{
			dStarLitePt->initCell(xs, ys, -1);
			dstar_map.data[ys * (dstar_map.width) + xs] = 100;

		    //xlog->Info("xy is %d %d ",xs,ys);
		}
		else  */
		{
			dStarLitePt->updateCell(xs, ys, -1);
			dstar_map.data[ys * (dstar_map.width) + xs] = 100;
		}
		mark_img.at<uchar>(ys, xs)=255;
	//
	}

	xlog->Info("bumper_points.size() is %d ",bumper_points.size());

	if(1)
	{
		//std::unique_lock<std::mutex> lock(m_mappoints);		
		if (curr_level<4)
		{
			//std::unique_lock<std::mutex> lock(m_mappoints);	
			//std::lock_guard<std::mutex> lock(mtx);
			//if (m_mappoints.try_lock())
			{	
				for (size_t i = 0; i < dynamicMappoints.size(); i++)
				{
					ipoint2 grid = trans2Grid(dynamicMappoints[i].first, dynamicMappoints[i].second);
					int xs = grid.x;
					int ys = grid.y;
					if (dstar_map.data[ys * (dstar_map.width) + xs] != 100)
					{
					//	dStarLitePt->updateCell(xs, ys, -1);
					//	dstar_map.data[ys * (dstar_map.width) + xs] = 100;
						xlog->Info("dynamic xy is %d %d ",xs,ys);
					}
				}	

			//	m_mappoints.unlock();
			}
		}


		for (size_t i = 0; i < stuckMappoints.size(); i++)
		{
			ipoint2 grid = trans2Grid(stuckMappoints[i].first, stuckMappoints[i].second);
			int xs = grid.x;
			int ys = grid.y;
			if (dstar_map.data[ys * (dstar_map.width) + xs] != 100)
			{
				//dStarLitePt->updateCell(xs, ys, -1);
				//dstar_map.data[ys * (dstar_map.width) + xs] = 100;
				xlog->Info("stuckMappoints xy is %d %d ",xs,ys);
			}
		}	


		//lock.unlock();
		// std::lock_guard<std::mutex> unlock(m_mappoints);	
	}

	cv::imwrite("/app/fj212/dstarmark_img.jpg", mark_img);

	for (size_t i = 0; i < bumper_points.size(); i++)
	//for (size_t i = 0; i < 0; i++)
	{
		int x= bumper_points[i].x;
		int y= bumper_points[i].y;
		//dStarLitePt->updateCell(x, y, -1.0);
		
		//if (dstar_map.data[ys0 * (dstar_map.width) + xs0] != 100)
		if (1)
		{
			if (curr_level==1)
			{
				double robotRaidus = 3*1; // 0.2 0.15
				int step = 5;
				for (int xs = x - step; xs < x + step; xs++)
				{
					for (int ys = y - step; ys < y + step; ys++)
					{
						if (xs < 0 || xs >= dstar_map.width || ys < 0 || ys >= dstar_map.height)  continue;
						if (dstar_map.data[ys * (dstar_map.width) + xs] == 100)  continue;
						
						if (std::sqrt((x - xs) * (x - xs) + (y - ys) * (y - ys)) <= robotRaidus) // 4.0
						{
							dStarLitePt->updateCell(xs, ys, -1.0);
							dstar_map.data[ys * (dstar_map.width) + xs] = 100;
						}
					}
				}
			}
			else if (curr_level==2)
			{
				double robotRaidus = 2; // 0.2 0.15
				int step = 4;
				for (int xs = x - step; xs < x + step; xs++)
				{
					for (int ys = y - step; ys < y + step; ys++)
					{
						if (xs < 0 || xs >= dstar_map.width || ys < 0 || ys >= dstar_map.height)  continue;
						if (dstar_map.data[ys * (dstar_map.width) + xs] == 100)  continue;
						
						if (std::sqrt((x - xs) * (x - xs) + (y - ys) * (y - ys)) <= robotRaidus) // 4.0
						{
							dStarLitePt->updateCell(xs, ys, -1.0);
							dstar_map.data[ys * (dstar_map.width) + xs] = 100;
						}
					}
				}
			}
			else if (curr_level==3)
			{
				double robotRaidus = 1.5; // 0.2 0.15
				int step = 3;
				for (int xs = x - step; xs < x + step; xs++)
				{
					for (int ys = y - step; ys < y + step; ys++)
					{
						if (xs < 0 || xs >= dstar_map.width || ys < 0 || ys >= dstar_map.height)  continue;
						if (dstar_map.data[ys * (dstar_map.width) + xs] == 100)  continue;
						
						if (std::sqrt((x - xs) * (x - xs) + (y - ys) * (y - ys)) <= robotRaidus) // 4.0
						{
							dStarLitePt->updateCell(xs, ys, -1.0);
							dstar_map.data[ys * (dstar_map.width) + xs] = 100;
						}
					}
				}
			}	
			else if (curr_level==4)
			{
				double robotRaidus = 4*0.25; // 0.2 0.15
				int step = 2;
				for (int xs = x - step; xs < x + step; xs++)
				{
					for (int ys = y - step; ys < y + step; ys++)
					{
						if (xs < 0 || xs >= dstar_map.width || ys < 0 || ys >= dstar_map.height)  continue;
						if (dstar_map.data[ys * (dstar_map.width) + xs] == 100)  continue;
						
						if (std::sqrt((x - xs) * (x - xs) + (y - ys) * (y - ys)) <= robotRaidus) // 4.0
						{
							dStarLitePt->updateCell(xs, ys, -1.0);
							dstar_map.data[ys * (dstar_map.width) + xs] = 100;
						}
					}
				}
			} 
		}	
	}

	ipoint2 new_st;
	int step = 3;
	int x = st.x;
	int y = st.y;
	int freeX=x;
	int freeY=y;
	// std::cout << "step: " << step << " "<<navmap->width<<std::endl;
	float minDist= INFINITY;
	bool isfind=false;
	if (dstar_map.data[y * (dstar_map.width) + x] !=0)
	{
		for (unsigned int xs = x - step; xs <= x + step; xs++)
		{
			for (unsigned int ys = y - step; ys <= y + step; ys++)
			{
				if (ys < 0 || ys >= dstar_map.height || xs < 0 || xs >= dstar_map.width)
				{
					continue;
				}

				if (dstar_map.data[ys * (dstar_map.width) + xs] ==0)
				{
					float dist = std::sqrt((xs - x) * (xs - x) + (ys - y) * (ys - y));
					if (dist< minDist)
					{
						minDist = dist;
						isfind=true;
						freeX = xs;
						freeY = ys;
					}	
				}
			}
		}

		xlog->Info("freeX is  %d %d %d",isfind,freeX,freeY);
	}

	new_st.x =freeX;
	new_st.y= freeY;

	cv::Mat img0(dstar_map.height, dstar_map.width, CV_8UC1, dstar_map.data.data()); // ATTENTION!!!: for opencv 3.1, source image is modified when using findcontour
	dStarLitePt->setMap(img0.data, dstar_map.width,dstar_map.height);
	cv::imwrite("/app/fj212/dstarmap.jpg", img0); 

	dStarLitePt->draw();
	uint64_t st1 = getTimeStap_us();
	dStarLitePt->updateStart(new_st.x, new_st.y);

	for (size_t i = 0; i < bumper_points.size(); i++)
	//for (size_t i = 0; i < 0; i++)
	{
		int x= bumper_points[i].x;
		int y= bumper_points[i].y;
		dStarLitePt->updateCell(x, y, -1.0);
	}

	//dStarLitePt->updateGoal(ed.x, ed.y);
	uint64_t st2 = getTimeStap_us();
	//dStarLitePt->draw();
	// printf("replan \n");

	if (test_dstar)
	{
		char name[50];
		sprintf(name, "/tmp/plan%d_%d_%d_%d.ds", st.x, st.y, ed.x, ed.y);
		dStarLitePt->DStarLiteSave(name);
		//dStarLitePt->DStarLiteLoad("/app/fj212/plan146_143_119_120.ds");
	}	

	bool success = dStarLitePt->replan(resultError);
	dStarLitePt->updateCell(new_st.x, new_st.y, 1);
	
	uint64_t st3 = getTimeStap_us();
	dStarLitePt->draw();

	if (success ==false)
	{
	//	system("aplay /app/fj212/resource/robot_voice/8.wav");
		xlog->Info("replan no path ");
		bool succes1 = dStarLitePt->replanPath();
		if (succes1 ==false)
		{
			xlog->Info("replanpath failed  again");
			return false;
		}
		else
		{
			xlog->Info("get replan path");
			return true;
		}
	}

	uint32_t diff_t0 = st1 - st0;
	uint32_t diff_t1 = st2 - st1;
	uint32_t diff_t2 = st3 - st2;

	xlog->Info("cost time is :  %d   %d   %d us ", diff_t0, diff_t1, diff_t2);

	return true;
}


void DStarPlanner_t::requestPlannerMap()
{
	NavMsg::SlamMapRequest_t mapReq;
	mapReq.name = MapReqPlanner;
	mapReq.requestMap = true;
	lcm.publish(LCM_CHANNEL_MapRequest, &mapReq);
}

bool DStarPlanner_t::checkRoomId(int &id, int x, int y)
{
	cv::Point2i robot_pos;
	robot_pos.x = x;
	robot_pos.y = y;
	id = -1;
	xlog->Info("robot_pos x y is %d %d ", robot_pos.x, robot_pos.y);

	float maxDist=-1000;
	for (size_t ii = 0; ii < m_blockArrays.blkNum; ii++)
	{
		std::vector<cv::Point> curr_ploys;
		for (size_t jj = m_blockArrays.blks[ii].pointNum; jj > 0; jj--)
		{
			cv::Point st;
			float x = m_blockArrays.blks[ii].points[jj - 1].x;
			float y = m_blockArrays.blks[ii].points[jj - 1].y;
			ipoint2 grid = trans2Grid(x, y);
			st.x = grid.x;
			st.y = grid.y;
			curr_ploys.push_back(st);
			//printf("st.x y is %d %d \n",st.x,st.y);
		}

		xlog->Info("curr_ploys is %d ", curr_ploys.size());
		if (curr_ploys.size() == 0)
		{
			xlog->Error("m_blockArrays %d is error !", ii);
			continue;
		}

		float ret = cv::pointPolygonTest(curr_ploys, cv::Point2f(robot_pos.x, robot_pos.y), true);

		if (ret > -2&&ret>maxDist)
		{
			maxDist = ret;
			id = m_blockArrays.blks[ii].id;
			xlog->Info("room id is  %d ", id);
			//return true;
		}
	}

	if (id>0)
	{
		xlog->Info("room best id is  %d ", id);
		return true;
	}
	

	xlog->Error("get room id failed! ");

	return false;
}

bool DStarPlanner_t::getNextBlkEntry(int curId, int dstId, std::vector<std::pair<int, int>> &roomPath, std::vector<bool> &visit)
{
	bool ret = false;
	if ((curId > m_blockArrays.blks.size()) || (dstId > m_blockArrays.blks.size()))
		return false;

	visit[curId - 1] = true;
	xlog->Info("entryNum IS %d ", m_blockArrays.blks[curId - 1].entryNum);
	for (int i = 0; i < m_blockArrays.blks[curId - 1].entryNum; i++)
	{
		if (m_blockArrays.blks[curId - 1].nextBlkIdOfEntries[i] == dstId)
		{
			xlog->Debug("find id ok id %d %d ", curId, i);
			roomPath.push_back(std::make_pair(curId, i));
			return true;
		}
	}

	bool isfind = false;
	for (int i = 0; i < m_blockArrays.blks[curId - 1].entryNum; i++)
	{
		if (visit[m_blockArrays.blks[curId - 1].nextBlkIdOfEntries[i] - 1] == false)
		{
			isfind = true;
			roomPath.push_back(std::make_pair(curId, i));

			ret = getNextBlkEntry(m_blockArrays.blks[curId - 1].nextBlkIdOfEntries[i], dstId, roomPath, visit);
			if (ret)
				break;
		}
	}

	if (isfind == false)
	{
		if (roomPath.size()>0)
		{
			roomPath.pop_back();
		}	
		else return false;	
	}

	return ret;
}

bool DStarPlanner_t::searchConnetRegion(std::vector<ipoint2> &connectRegion,int x,int y)
{
	connectRegion.clear();	
	int visit[500][500] = {0};
	int mov[4][2] = {1, 0, -1, 0, 0, 1, 0, -1};

	//std::vector<ipoint2> &connectRegion;
	std::queue<ipoint2> costPoint;
	ipoint2 start;
	start.x =x;
	start.y =y;
	costPoint.push(start);
	visit[200][200] =1;
	while (costPoint.size()>0)
	{
		ipoint2 curr =  costPoint.front();
		for (size_t i = 0; i < 4; i++)
		{
			int tx= curr.x+ mov[i][0];
			int ty= curr.y+ mov[i][1];
			ipoint2 next;
			next.x = tx;
			next.y = ty;

			if (tx<=start.x-200||tx>=start.x+200||ty<=start.y-200||ty>=start.y+200)
			{
				continue;
			}

			uint8_t value= dstar_map.data[ty * (dstar_map.width) + tx] & 3;
			int newX = tx - start.x +200;
			int newY = ty - start.y +200;
			if (value == 0 && visit[newX][newX] == 0)
			{
				costPoint.push(next);
				connectRegion.push_back(next);
				visit[newX][newX] =1;
			}	
			if (connectRegion.size()>1000)
			{
				return false;
			}
			
		}

		costPoint.pop();
		
	}
	
	if (connectRegion.size()>0)
	{
		return true;
	}
	
	return false;

}

bool DStarPlanner_t:: stopPlan()
{
 	is_stop_plan_flag =true;
	return true;
}

// extern "C"
//{

BaseGPlanner_t *CreatGPlanner(std::string name, ini::IniFile *ptr)
{
	return new DStarPlanner_t(name, ptr);
}
//}
