#include <cstring>
#include <limits>
#include <list>
#include <iostream>

#include "gmapping/scanmatcher/scanmatcher.h"
#include "gmapping/scanmatcher/gridlinetraversal.h"
// #define GENERATE_MAPS
#include <chrono>

namespace GMapping
{

	using namespace std;

	const double ScanMatcher::nullLikelihood = -.5;
	cv::Mat img_map = cv::Mat(cv::Size(300, 300), CV_32FC3, cv::Scalar(0,0,0)); 

	ScanMatcher::ScanMatcher() : m_laserPose(0, 0, 0)
	{
		// m_laserAngles=0;
		m_laserBeams = 0;
		m_optRecursiveIterations = 3;
		m_activeAreaComputed = false;

		// This  are the dafault settings for a grid map of 5 cm
		m_llsamplerange = 0.01;
		m_llsamplestep = 0.01;
		m_lasamplerange = 0.005;
		m_lasamplestep = 0.005;
		m_enlargeStep = 10.;
		m_fullnessThreshold = 0.1;
		m_angularOdometryReliability = 2;
		//m_linearOdometryReliability = 0.;
		// m_angularOdometryReliability=5;
		m_linearOdometryReliability = 3;
		m_freeCellRatio = sqrt(2.);
		m_initialBeamsSkip = 0;

		/*
			// This  are the dafault settings for a grid map of 10 cm
			m_llsamplerange=0.1;
			m_llsamplestep=0.1;
			m_lasamplerange=0.02;
			m_lasamplestep=0.01;
		*/
		// This  are the dafault settings for a grid map of 20/25 cm
		/*
			m_llsamplerange=0.2;
			m_llsamplestep=0.1;
			m_lasamplerange=0.02;
			m_lasamplestep=0.01;
			m_generateMap=false;
		*/

		m_linePoints = new IntPoint[20000];
	}

	ScanMatcher::~ScanMatcher()
	{
		delete[] m_linePoints;
	}

	void ScanMatcher::invalidateActiveArea()
	{
		m_activeAreaComputed = false;
	}

	void ScanMatcher::setLinearPara(float reliability)
	{
		m_linearOdometryReliability = reliability;
	}

	void ScanMatcher::setAnglePara(float reliability)
	{
		m_angularOdometryReliability = reliability;
	}


	/*
	void ScanMatcher::computeActiveArea(ScanMatcherMap& map, const OrientedPoint& p, const double* readings){
		if (m_activeAreaComputed)
			return;
		HierarchicalArray2D<PointAccumulator>::PointSet activeArea;
		OrientedPoint lp=p;
		lp.x+=cos(p.theta)*m_laserPose.x-sin(p.theta)*m_laserPose.y;
		lp.y+=sin(p.theta)*m_laserPose.x+cos(p.theta)*m_laserPose.y;
		lp.theta+=m_laserPose.theta;
		IntPoint p0=map.world2map(lp);
		const double * angle=m_laserAngles;
		for (const double* r=readings; r<readings+m_laserBeams; r++, angle++)
			if (m_generateMap){
				double d=*r;
				if (d>m_laserMaxRange)
					continue;
				if (d>m_usableRange)
					d=m_usableRange;

				Point phit=lp+Point(d*cos(lp.theta+*angle),d*sin(lp.theta+*angle));
				IntPoint p1=map.world2map(phit);

				d+=map.getDelta();
				//Point phit2=lp+Point(d*cos(lp.theta+*angle),d*sin(lp.theta+*angle));
				//IntPoint p2=map.world2map(phit2);
				IntPoint linePoints[20000] ;
				GridLineTraversalLine line;
				line.points=linePoints;
				//GridLineTraversal::gridLine(p0, p2, &line);
				GridLineTraversal::gridLine(p0, p1, &line);
				for (int i=0; i<line.num_points-1; i++){
					activeArea.insert(map.storage().patchIndexes(linePoints[i]));
				}
				if (d<=m_usableRange){
					activeArea.insert(map.storage().patchIndexes(p1));
					//activeArea.insert(map.storage().patchIndexes(p2));
				}
			} else {
				if (*r>m_laserMaxRange||*r>m_usableRange) continue;
				Point phit=lp;
				phit.x+=*r*cos(lp.theta+*angle);
				phit.y+=*r*sin(lp.theta+*angle);
				IntPoint p1=map.world2map(phit);
				assert(p1.x>=0 && p1.y>=0);
				IntPoint cp=map.storage().patchIndexes(p1);
				assert(cp.x>=0 && cp.y>=0);
				activeArea.insert(cp);

			}
		//this allocates the unallocated cells in the active area of the map
		//cout << "activeArea::size() " << activeArea.size() << endl;
		map.storage().setActiveArea(activeArea, true);
		m_activeAreaComputed=true;
	}
	*/
	void ScanMatcher::computeActiveArea(ScanMatcherMap &map, const OrientedPoint &p, const double *readings)
	{
		if (m_activeAreaComputed)
			return;
		OrientedPoint lp = p;
		lp.x += cos(p.theta) * m_laserPose.x - sin(p.theta) * m_laserPose.y;
		lp.y += sin(p.theta) * m_laserPose.x + cos(p.theta) * m_laserPose.y;
		lp.theta += m_laserPose.theta;

		//	printf("computeActiveArea m_generateMap is %d \n",m_generateMap);
		// if(m_generateMap)
		//	printf("lp is  %f  %f %f \n",lp.x,lp.y,lp.theta);

		// IntPoint p0=map.world2map(lp);

		Point min(map.map2world(0, 0));
		Point max(map.map2world(map.getMapSizeX() - 1, map.getMapSizeY() - 1));

		if (lp.x < min.x)
			min.x = lp.x;
		if (lp.y < min.y)
			min.y = lp.y;
		if (lp.x > max.x)
			max.x = lp.x;
		if (lp.y > max.y)
			max.y = lp.y;

		std::vector<float> data1;
		std::vector<float> data2;
		data1.resize(m_laserBeams);
		data2.resize(m_laserBeams);
		int cnt = 0;
		//	std::chrono::high_resolution_clock::time_point st0 = std::chrono::high_resolution_clock::now();
		/*determine the size of the area*/
		const double *angle = m_laserAngles + m_initialBeamsSkip;
		for (const double *r = readings + m_initialBeamsSkip; r < readings + m_laserBeams; r++, angle++)
		{
			cnt++;
			if (*r > m_laserMaxRange || *r <= 1e-6 || isnan(*r))
				continue;
			float d = *r > m_usableRange ? m_usableRange : *r;
			Point phit = lp;
			float theta = lp.theta + *angle;
			float d1 = d * cos(theta);
			float d2 = d * sin(theta);
			data1[cnt - 1] = d1;
			data2[cnt - 1] = d2;
			phit.x += d1; // d*cos(lp.theta+*angle);
			phit.y += d2; // d*sin(lp.theta+*angle);
			if (phit.x < min.x)
				min.x = phit.x;
			if (phit.y < min.y)
				min.y = phit.y;
			if (phit.x > max.x)
				max.x = phit.x;
			if (phit.y > max.y)
				max.y = phit.y;
		}
		// min=min-Point(map.getDelta(),map.getDelta());
		// max=max+Point(map.getDelta(),map.getDelta());

		if (!map.isInside(min) || !map.isInside(max))
		{
			Point lmin(map.map2world(0, 0));
			Point lmax(map.map2world(map.getMapSizeX() - 1, map.getMapSizeY() - 1));
			// cerr << "CURRENT MAP " << lmin.x << " " << lmin.y << " " << lmax.x << " " << lmax.y << endl;
			// cerr << "BOUNDARY OVERRIDE " << min.x << " " << min.y << " " << max.x << " " << max.y << endl;
			min.x = (min.x >= lmin.x) ? lmin.x : min.x - m_enlargeStep;
			max.x = (max.x <= lmax.x) ? lmax.x : max.x + m_enlargeStep;
			min.y = (min.y >= lmin.y) ? lmin.y : min.y - m_enlargeStep;
			max.y = (max.y <= lmax.y) ? lmax.y : max.y + m_enlargeStep;
			map.resize(min.x, min.y, max.x, max.y);
			// cerr << "RESIZE " << min.x << " " << min.y << " " << max.x << " " << max.y << endl;
		}

		std::chrono::high_resolution_clock::time_point st1 = std::chrono::high_resolution_clock::now();

		HierarchicalArray2D<PointAccumulator>::PointSet activeArea;
		/*allocate the active area*/
		cnt = 0;
		angle = m_laserAngles + m_initialBeamsSkip;
		std::vector<IntPoint> freegridPts;
		int width =map.getMapSizeX();
		int height = map.getMapSizeY();
		std::vector< int8_t > mask;
		mask.resize(width*height); 
		for (const double *r = readings + m_initialBeamsSkip; r < readings + m_laserBeams; r++, angle++)
		{
			cnt++;
			if (m_generateMap)
			{

				float d = *r;
				if (d > m_laserMaxRange || d <= 1e-6 || isnan(d))
					continue;
				if (d > m_usableRange)
					d = m_usableRange;
				Point phit = lp + Point(d * cos(lp.theta + *angle), d * sin(lp.theta + *angle));
				// Point phit=lp+Point(data1[cnt-1],data2[cnt-1]);
				IntPoint p0 = map.world2map(lp);
				IntPoint p1 = map.world2map(phit);

				// IntPoint linePoints[20000] ;
				GridLineTraversalLine line;
				line.points = m_linePoints;
				GridLineTraversal::gridLine(p0, p1, &line);
				/* for (int i = 0; i < line.num_points - 1; i++)
				{
					IntPoint tmp= m_linePoints[i];
					int index = width * tmp.y+ tmp.x;
					if (mask[index]==0)
					{
						mask[index]=1;
						freegridPts.push_back(tmp);
					}

				}  */
				for (int i = 0; i < line.num_points - 1; i++)
				{
					assert(map.isInside(m_linePoints[i]));
					activeArea.insert(map.storage().patchIndexes(m_linePoints[i]));
					assert(m_linePoints[i].x >= 0 && m_linePoints[i].y >= 0);
				} 
				if (d < m_usableRange)
				{
					IntPoint cp = map.storage().patchIndexes(p1);
					assert(cp.x >= 0 && cp.y >= 0);
					activeArea.insert(cp);
				}
			}
			else
			{
				if (*r > m_laserMaxRange || *r > m_usableRange || *r <= 1e-6 || isnan(*r))
					continue;
				Point phit = lp;
				// phit.x+=*r*cos(lp.theta+*angle);
				// phit.y+=*r*sin(lp.theta+*angle);
				phit.x += data1[cnt - 1];
				phit.y += data2[cnt - 1];
				IntPoint p1 = map.world2map(phit);
				assert(p1.x >= 0 && p1.y >= 0);
				IntPoint cp = map.storage().patchIndexes(p1);
				assert(cp.x >= 0 && cp.y >= 0);
				activeArea.insert(cp);
			}
		}
		/* for (int i = 0; i < freegridPts.size(); i++)
		{
			assert(map.isInside(freegridPts[i]));
			activeArea.insert(map.storage().patchIndexes(freegridPts[i]));
			assert(freegridPts[i].x >= 0 && freegridPts[i].y >= 0);
		}	 */ 

		// this allocates the unallocated cells in the active area of the map
		// cout << "activeArea::size() " << activeArea.size() << endl;
		/*
			cerr << "ActiveArea=";
			for (HierarchicalArray2D<PointAccumulator>::PointSet::const_iterator it=activeArea.begin(); it!= activeArea.end(); it++){
				cerr << "(" << it->x <<"," << it->y << ") ";
			}
			cerr << endl;
		*/
		map.storage().setActiveArea(activeArea, true);
		m_activeAreaComputed = true;

		//	std::chrono::high_resolution_clock::time_point st2 = std::chrono::high_resolution_clock::now();
		//	double diff_t0 = std::chrono::duration_cast<std::chrono::duration<double>>(st1 - st0).count() * 1000;
		//	double diff_t1 = std::chrono::duration_cast<std::chrono::duration<double>>(st2 - st1).count() * 1000;
		//	printf("compute area  time  is %f   %f \n",diff_t0,diff_t1);
	}

	double ScanMatcher::registerScan(ScanMatcherMap &map, const OrientedPoint &p, const double *readings)
	{
		if (!m_activeAreaComputed)
			computeActiveArea(map, p, readings);

		//	printf("regis m_generateMap is %d \n",m_generateMap);
		//	std::chrono::high_resolution_clock::time_point st1 = std::chrono::high_resolution_clock::now();
		// this operation replicates the cells that will be changed in the registration operation
		map.storage().allocActiveArea();

		OrientedPoint lp = p;
		lp.x += cos(p.theta) * m_laserPose.x - sin(p.theta) * m_laserPose.y;
		lp.y += sin(p.theta) * m_laserPose.x + cos(p.theta) * m_laserPose.y;
		lp.theta += m_laserPose.theta;
		IntPoint p0 = map.world2map(lp);

		const double *angle = m_laserAngles + m_initialBeamsSkip;
		double esum = 0;
		for (const double *r = readings + m_initialBeamsSkip; r < readings + m_laserBeams; r++, angle++)
			if (m_generateMap)
			{
				double d = *r;
				if (d > m_laserMaxRange || d <= 1e-6 || isnan(d))
					continue;
				// if (d>m_usableRange)
				// d=m_usableRange;
				Point phit = lp + Point(d * cos(lp.theta + *angle), d * sin(lp.theta + *angle));
				IntPoint p1 = map.world2map(phit);
				// IntPoint linePoints[20000] ;
				GridLineTraversalLine line;
				line.points = m_linePoints;
				GridLineTraversal::gridLine(p0, p1, &line);
				for (int i = 0; i < line.num_points - 1; i++)
				{
					PointAccumulator &cell = map.cell(line.points[i]);
					double e = -cell.entropy();
					cell.update(false, Point(0, 0));
					e += cell.entropy();
					esum += e;
				}
				if (d < m_usableRange)
				{
					double e = -map.cell(p1).entropy();
					map.cell(p1).update(true, phit);
					e += map.cell(p1).entropy();
					esum += e;
				}
			}
			else
			{
				if (*r > m_laserMaxRange || *r > m_usableRange || *r <= 1e-6 || isnan(*r))
					continue;
				Point phit = lp;
				phit.x += *r * cos(lp.theta + *angle);
				phit.y += *r * sin(lp.theta + *angle);
				IntPoint p1 = map.world2map(phit);
				assert(p1.x >= 0 && p1.y >= 0);
				map.cell(p1).update(true, phit);
			}
		// cout  << "informationGain=" << -esum << endl;
		//	std::chrono::high_resolution_clock::time_point st2 = std::chrono::high_resolution_clock::now();
		//	double diff_t0 = std::chrono::duration_cast<std::chrono::duration<double>>(st2 - st1).count() * 1000;
		//	printf("regist time  is %f  \n",diff_t0);
		return esum;
	}

	double ScanMatcher::computeActiveAreaAndRegisterScan(ScanMatcherMap &map, const OrientedPoint &p, const double *readings)
	{
		// if (!m_activeAreaComputed)
		//	computeActiveArea(map, p, readings);

		printf("m_laserMaxRange is %f  %f \n",m_laserMaxRange,m_usableRange);
		std::chrono::high_resolution_clock::time_point st0 = std::chrono::high_resolution_clock::now();
		Point min(map.map2world(0, 0));
		Point max(map.map2world(map.getMapSizeX() - 1, map.getMapSizeY() - 1));

		OrientedPoint lp = p;

		if (lp.x < min.x)
			min.x = lp.x;
		if (lp.y < min.y)
			min.y = lp.y;
		if (lp.x > max.x)
			max.x = lp.x;
		if (lp.y > max.y)
			max.y = lp.y;

		/*determine the size of the area*/
		const double *angle = m_laserAngles + m_initialBeamsSkip;
		std::vector<float> data1;
		std::vector<float> data2;
		data1.resize(m_laserBeams);
		data2.resize(m_laserBeams);

		int cnt = 0;
		for (const double *r = readings + m_initialBeamsSkip; r < readings + m_laserBeams; r++, angle++)
		{
			cnt++;
			if (*r >4)
			{
				float d=3.0;
				Point phit = lp + Point(d * cos(lp.theta + *angle), d * sin(lp.theta + *angle));
				if (phit.x < min.x)
					min.x = phit.x;
				if (phit.y < min.y)
					min.y = phit.y;
				if (phit.x > max.x)
					max.x = phit.x;
				if (phit.y > max.y)
					max.y = phit.y;

				continue;
			} 

			if (*r > m_laserMaxRange || *r <= 1e-6 || isnan(*r))
				continue;
			float d = *r > m_usableRange ? m_usableRange : *r;
			Point phit = lp;
			float theta = lp.theta + *angle;
			float d1 = d * cos(theta);
			float d2 = d * sin(theta);
			// data1.push_back(d1);
			// data2.push_back(d2);
			data1[cnt - 1] = d1;
			data2[cnt - 1] = d2;
			phit.x += d1; // d * d1; //cos(lp.theta + *angle);
			phit.y += d2; // d * d2; //sin(lp.theta + *angle);
			if (phit.x < min.x)
				min.x = phit.x;
			if (phit.y < min.y)
				min.y = phit.y;
			if (phit.x > max.x)
				max.x = phit.x;
			if (phit.y > max.y)
				max.y = phit.y;
		}
		// min=min-Point(map.getDelta(),map.getDelta());
		// max=max+Point(map.getDelta(),map.getDelta());

		if (!map.isInside(min) || !map.isInside(max))
		{
			Point lmin(map.map2world(0, 0));
			Point lmax(map.map2world(map.getMapSizeX() - 1, map.getMapSizeY() - 1));
			// cerr << "CURRENT MAP " << lmin.x << " " << lmin.y << " " << lmax.x << " " << lmax.y << endl;
			// cerr << "BOUNDARY OVERRIDE " << min.x << " " << min.y << " " << max.x << " " << max.y << endl;
			min.x = (min.x >= lmin.x) ? lmin.x : min.x - m_enlargeStep;
			max.x = (max.x <= lmax.x) ? lmax.x : max.x + m_enlargeStep;
			min.y = (min.y >= lmin.y) ? lmin.y : min.y - m_enlargeStep;
			max.y = (max.y <= lmax.y) ? lmax.y : max.y + m_enlargeStep;
			map.resize(min.x, min.y, max.x, max.y);
			//	std::cout << "RESIZE " << min.x << " " << min.y << " " << max.x << " " << max.y << endl;
		}

		IntPoint p0 = map.world2map(lp);

		HierarchicalArray2D<PointAccumulator>::PointSet activeArea;
		/*allocate the active area*/
		angle = m_laserAngles + m_initialBeamsSkip;
		// std::vector<std::vector<IntPoint>> allGridLinePts;
		// allGridLinePts.resize(m_laserBeams);
		std::vector<IntPoint> freegridPts;
		std::chrono::high_resolution_clock::time_point st1 = std::chrono::high_resolution_clock::now();
		int width = map.getMapSizeX();
		int height = map.getMapSizeY();
		std::vector<uint8_t> mask;
		mask.resize(width * height, 0);

		// std::chrono::high_resolution_clock::time_point st1 = std::chrono::high_resolution_clock::now();

		cnt = 0;
		for (const double *r = readings + m_initialBeamsSkip; r < readings + m_laserBeams; r++, angle++)
		{
			if (m_generateMap)
			{
				float d = *r;
				cnt++;
				// allGridLinePts[cnt-1].reserve(100);
				if (d >4 )
				{
					d=3.0;
					Point phit = lp + Point(d * cos(lp.theta + *angle), d * sin(lp.theta + *angle));
					// IntPoint p0 = map.world2map(lp);
					IntPoint p1 = map.world2map(phit);
					GridLineTraversalLine line;
					line.points = m_linePoints;
					GridLineTraversal::gridLine(p0, p1, &line);
					for (int i = 0; i < line.num_points - 1; i++)
					{
						IntPoint tmp = m_linePoints[i];
						int index = width * tmp.y + tmp.x;
						if (mask[index] == 0)
						{
							mask[index] = 1;
							freegridPts.push_back(tmp);
						}
						else
							mask[index]++;
					}
					continue;
				} 
				
				if (d > m_laserMaxRange || d <= 1e-6 || isnan(d))
					continue;
				if (d > m_usableRange)
					d = m_usableRange;
				// Point phit = lp + Point(d * cos(lp.theta + *angle), d * sin(lp.theta + *angle));
				Point phit = lp + Point(data1[cnt - 1], data2[cnt - 1]);
				// IntPoint p0 = map.world2map(lp);
				IntPoint p1 = map.world2map(phit);

				// IntPoint linePoints[20000] ;
				GridLineTraversalLine line;
				line.points = m_linePoints;
				GridLineTraversal::gridLine(p0, p1, &line);
				for (int i = 0; i < line.num_points - 1; i++)
				{
					IntPoint tmp = m_linePoints[i];
					int index = width * tmp.y + tmp.x;
					if (mask[index] == 0)
					{
						mask[index] = 1;
						freegridPts.push_back(tmp);
					}
					else
						mask[index]++;
				}

				if (d < m_usableRange)
				{
					IntPoint cp = map.storage().patchIndexes(p1);
					assert(cp.x >= 0 && cp.y >= 0);
					activeArea.insert(cp);
				}
			}
			else
			{
				if (*r > m_laserMaxRange || *r > m_usableRange || *r <= 1e-6 || isnan(*r))
					continue;
				Point phit = lp;
				phit.x += *r * cos(lp.theta + *angle);
				phit.y += *r * sin(lp.theta + *angle);
				IntPoint p1 = map.world2map(phit);
				assert(p1.x >= 0 && p1.y >= 0);
				IntPoint cp = map.storage().patchIndexes(p1);
				assert(cp.x >= 0 && cp.y >= 0);
				activeArea.insert(cp);
			}
		}

		for (int i = 0; i < freegridPts.size(); i++)
		{
			assert(map.isInside(freegridPts[i]));
			activeArea.insert(map.storage().patchIndexes(freegridPts[i]));
			assert(freegridPts[i].x >= 0 && freegridPts[i].y >= 0);
		}

		map.storage().setActiveArea(activeArea, true);
		m_activeAreaComputed = true;

		// registerScan
		// this operation replicates the cells that will be changed in the registration operation
		map.storage().allocActiveArea();

		/* OrientedPoint lp=p;
		lp.x+=cos(p.theta)*m_laserPose.x-sin(p.theta)*m_laserPose.y;
		lp.y+=sin(p.theta)*m_laserPose.x+cos(p.theta)*m_laserPose.y;
		lp.theta+=m_laserPose.theta; */
		// IntPoint p0=map.world2map(lp);
		std::chrono::high_resolution_clock::time_point st2 = std::chrono::high_resolution_clock::now();

		angle = m_laserAngles + m_initialBeamsSkip;
		double esum = 0;
		cnt = 0;
		int GridPtsNum = 0;
		for (const double *r = readings + m_initialBeamsSkip; r < readings + m_laserBeams; r++, angle++)
		{
			if (m_generateMap)
			{
				cnt++;
				float d = *r;
				if (d > m_laserMaxRange || d <= 1e-6 || isnan(d))
					continue;
				if (d > m_usableRange)
					d = m_usableRange;
				Point phit = lp + Point(data1[cnt - 1], data2[cnt - 1]);
				// Point phit = lp + Point(d * cos(lp.theta + *angle), d * sin(lp.theta + *angle));
				IntPoint p1 = map.world2map(phit);

				// std::vector<IntPoint> currGrid;
				// currGrid = allGridLinePts[cnt-1];
				// IntPoint linePoints[20000] ;
				/* GridLineTraversalLine line;
				line.points=m_linePoints;
				GridLineTraversal::gridLine(p0, p1, &line);
				//GridPtsNum++;
				//printf("currGrid.size() is %d \n",currGrid.size());
				//for (int i=0; i<currGrid.size(); i++){
				//	PointAccumulator& cell=map.cell(currGrid[i]);
				for (int i = 0; i < line.num_points - 1; i++)
				{
					PointAccumulator &cell = map.cell(line.points[i]);
					double e=-cell.entropy();
					cell.update(false, Point(0,0));
					e+=cell.entropy();
					esum+=e;
				} */
				if (d < m_usableRange)
				{
					double e = -map.cell(p1).entropy();
					map.cell(p1).update(true, phit);
					e += map.cell(p1).entropy();
					esum += e;
				}
			}
			else
			{
				if (*r > m_laserMaxRange || *r > m_usableRange || *r <= 1e-6 || isnan(*r))
					continue;
				Point phit = lp;
				phit.x += *r * cos(lp.theta + *angle);
				phit.y += *r * sin(lp.theta + *angle);
				IntPoint p1 = map.world2map(phit);
				assert(p1.x >= 0 && p1.y >= 0);
				map.cell(p1).update(true, phit);
			}
		}

		for (int i = 0; i < freegridPts.size(); i++)
		{
			int index = width * freegridPts[i].y + freegridPts[i].x;
			int visits = mask[index];
			/* if (!map.isInside(freegridPts[i]))
			{
				printf("not inside freegridPts is %d %d \n",freegridPts[i].x,freegridPts[i].y);
			}
			 */

			// assert(map.isInside(freegridPts[i]));
			if (visits >= 0)
			{
				if (visits == 0)
				{
					visits = 1;
				}

				PointAccumulator &cell = map.cell(freegridPts[i]);
				// double e=-cell.entropy();
				cell.updateNew(false, visits, Point(0, 0));
				// e+=cell.entropy();
				// esum+=e;
			}
			// assert(freegridPts[i].x >= 0 && freegridPts[i].y >= 0);
		}

		// cout  << "informationGain=" << -esum << endl;
		std::chrono::high_resolution_clock::time_point st3 = std::chrono::high_resolution_clock::now();
		double diff_t0 = std::chrono::duration_cast<std::chrono::duration<double>>(st1 - st0).count() * 1000;
		double diff_t1 = std::chrono::duration_cast<std::chrono::duration<double>>(st2 - st1).count() * 1000;
		double diff_t2 = std::chrono::duration_cast<std::chrono::duration<double>>(st3 - st2).count() * 1000;
		//	printf("regist time  is %d %f  %f %f \n",freegridPts.size(),diff_t0,diff_t1,diff_t2);
		//printf("hitcnt is %d %f %f  ",hitCnt,m_usableRange,m_laserMaxRange);
		return esum;
	}



	void   ScanMatcher::initScanMap(ScanMatcherMap& map, std::vector<int8_t> mapData,int width,int height)
	{
		HierarchicalArray2D<PointAccumulator>::PointSet activeArea;

		for (int x = 0; x < width; x++)
		{
			for (int y = 0; y < height; y++)
			{
				uint8_t pix = mapData[x+y*width] & 7;
				if (pix==1) // occ
				{
					IntPoint p1;
					p1.x = x;
					p1.y = y;

					IntPoint cp = map.storage().patchIndexes(p1);
					assert(cp.x >= 0 && cp.y >= 0);
					activeArea.insert(cp);
				}
				else if (pix==7) //free 
				{
					IntPoint p1;
					p1.x = x;
					p1.y = y;
					activeArea.insert(map.storage().patchIndexes(p1));
				}
			}
		}

		map.storage().setActiveArea(activeArea);
		map.storage().allocActiveArea();


		for (int x = 0; x < width; x++)
		{
			for (int y = 0; y < height; y++)
			{
 				uint8_t pix = mapData[x+y*width] & 7;
				if (pix==1) // occ
				{
					IntPoint p1;
					p1.x = x;
					p1.y = y;
					assert(p1.x >= 0 && p1.y >= 0);

					Point phit = map.map2world(p1);
					map.cell(p1).update(true, phit);
				}
				else if (pix==7) //free 
				{
					IntPoint p1;
					p1.x = x;
					p1.y = y;
					PointAccumulator &cell = map.cell(p1);
					cell.updateNew(false, 1, Point(0, 0));
				}
			}
		}
	}
	/*
	void ScanMatcher::registerScan(ScanMatcherMap& map, const OrientedPoint& p, const double* readings){
		if (!m_activeAreaComputed)
			computeActiveArea(map, p, readings);

		//this operation replicates the cells that will be changed in the registration operation
		map.storage().allocActiveArea();

		OrientedPoint lp=p;
		lp.x+=cos(p.theta)*m_laserPose.x-sin(p.theta)*m_laserPose.y;
		lp.y+=sin(p.theta)*m_laserPose.x+cos(p.theta)*m_laserPose.y;
		lp.theta+=m_laserPose.theta;
		IntPoint p0=map.world2map(lp);
		const double * angle=m_laserAngles;
		for (const double* r=readings; r<readings+m_laserBeams; r++, angle++)
			if (m_generateMap){
				double d=*r;
				if (d>m_laserMaxRange)
					continue;
				if (d>m_usableRange)
					d=m_usableRange;
				Point phit=lp+Point(d*cos(lp.theta+*angle),d*sin(lp.theta+*angle));
				IntPoint p1=map.world2map(phit);

				IntPoint linePoints[20000] ;
				GridLineTraversalLine line;
				line.points=linePoints;
				GridLineTraversal::gridLine(p0, p1, &line);
				for (int i=0; i<line.num_points-1; i++){
					IntPoint ci=map.storage().patchIndexes(line.points[i]);
					if (map.storage().getActiveArea().find(ci)==map.storage().getActiveArea().end())
						cerr << "BIG ERROR" <<endl;
					map.cell(line.points[i]).update(false, Point(0,0));
				}
				if (d<=m_usableRange){

					map.cell(p1).update(true,phit);
				}
			} else {
				if (*r>m_laserMaxRange||*r>m_usableRange) continue;
				Point phit=lp;
				phit.x+=*r*cos(lp.theta+*angle);
				phit.y+=*r*sin(lp.theta+*angle);
				map.cell(phit).update(true,phit);
			}
	}

	*/



	inline void ScanMatcher::getCurrImgBound(cv::Point& start,const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const{
		const double * angle=m_laserAngles+m_initialBeamsSkip;
		OrientedPoint lp=p;
		unsigned int skip=0;
		std::vector<float> xVect;
		std::vector<float> yVect;

		for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++){
			skip++;
			skip=skip>m_likelihoodSkip?0:skip;
			if (skip||*r>m_usableRange||*r==0.0) continue;
			Point phit=lp;

			float costh= cos(lp.theta+*angle);
			float sinth= sin(lp.theta+*angle);
			phit.x+=*r*costh;
			phit.y+=*r*sinth;

			xVect.push_back(phit.x);
			yVect.push_back(phit.y);		
		}

		if (xVect.size()==0||yVect.size()==0)
		{
			Point min(map.map2world(0,0));

			IntPoint ip=map.world2map(min);
			start.x= ip.x;
			start.y= ip.y;
		}
		else 
		{
			float minY = *std::min_element(yVect.begin(), yVect.end())-1;
			float maxY = *std::max_element(yVect.begin(), yVect.end())+1;
			float minX = *std::min_element(xVect.begin(), xVect.end())-1;
			float maxX = *std::max_element(xVect.begin(), xVect.end())+1;

			Point min(map.map2world(0,0));
			Point max(map.map2world(map.getMapSizeX()-1,map.getMapSizeY()-1));
				
			if (minX<min.x) minX=min.x;
			if (minY<min.y) minY=min.y;

			Point pMin(minX,minY);
			
			IntPoint ip=map.world2map(pMin);
			start.x= ip.x;
			start.y= ip.y;
		}
	}		

	double ScanMatcher::icpOptimize(OrientedPoint &pnew, const ScanMatcherMap &map, const OrientedPoint &init, const double *readings) const
	{
		double currentScore;
		double sc = score(map, init, readings);
		;
		OrientedPoint start = init;
		pnew = init;
		int iterations = 0;
		do
		{
			currentScore = sc;
			sc = icpStep(pnew, map, start, readings);
			// cerr << "pstart=" << start.x << " " <<start.y << " " << start.theta << endl;
			// cerr << "pret=" << pnew.x << " " <<pnew.y << " " << pnew.theta << endl;
			start = pnew;
			iterations++;
		} while (sc > currentScore);
		cerr << "i=" << iterations << endl;
		return currentScore;
	}

	double ScanMatcher::optimize(OrientedPoint &pnew, const ScanMatcherMap &map, const OrientedPoint &init, const double *readings) const
	{
		double bestScore = -1;
		OrientedPoint currentPose = init;
		double currentScore = score(map, currentPose, readings);
		double adelta = m_optAngularDelta, ldelta = m_optLinearDelta;
		unsigned int refinement = 0;
	//	printf("init score is %f \n",currentScore);
	//	cv::Mat img_map = cv::Mat(cv::Size(map.getMapSizeX(), map.getMapSizeY()), CV_32FC3, cv::Scalar(0,0,0)); 
	
		for (size_t j = 0; j < img_map.rows; j++)
		{
			for (size_t i = 0; i < img_map.cols; i++)
			{
				img_map.at<cv::Vec3f>(j, i)[0]= 0;
				img_map.at<cv::Vec3f>(j, i)[1]= 0;
				img_map.at<cv::Vec3f>(j, i)[2]= 0;
			}	
		}

		cv::Point start;
		getCurrImgBound(start,map,currentPose,readings);

		enum Move
		{
			Front,
			Back,
			Left,
			Right,
			TurnLeft,
			TurnRight,
			Done
		};
		/*	cout << __PRETTY_FUNCTION__<<  " readings: ";
			for (int i=0; i<m_laserBeams; i++){
				cout << readings[i] << " ";
			}
			cout << endl;
		*/
		int c_iterations = 0;
		do
		{
			if (bestScore >= currentScore)
			{
				refinement++;
				adelta *= .5;
				ldelta *= .5;
			}
			bestScore = currentScore;
		//	cout <<"score="<< currentScore << " refinement=" << refinement<<"  iter  "<<c_iterations<<"  pose "<< currentPose.x  << " " << currentPose.y << " "<<std::endl;
			//		cout <<  "pose=" << currentPose.x  << " " << currentPose.y << " " << currentPose.theta << endl;
			OrientedPoint bestLocalPose = currentPose;
			OrientedPoint localPose = currentPose;

			Move move = Front;
			do
			{
				localPose = currentPose;
				switch (move)
				{
				case Front:
					localPose.x += ldelta;
					move = Back;
					break;
				case Back:
					localPose.x -= ldelta;
					move = Left;
					break;
				case Left:
					localPose.y -= ldelta;
					move = Right;
					break;
				case Right:
					localPose.y += ldelta;
					move = Done;
					break; 
				/* case Right:
					localPose.y += ldelta;
					move = TurnLeft;
					break;
				case TurnLeft:
					localPose.theta += adelta;
					move = TurnRight;
					break;
				case TurnRight:
					localPose.theta -= adelta;
					move = Done;
					break;  */
				default:;
				}

				double odo_gain = 1;
				if (m_angularOdometryReliability > 0.)
				{
					double dth = init.theta - localPose.theta;
					dth = atan2(sin(dth), cos(dth));
					dth *= dth;
					odo_gain *= exp(-m_angularOdometryReliability * dth);
				}
				if (m_linearOdometryReliability > 0.)
				{
					double dx = init.x - localPose.x;
					double dy = init.y - localPose.y;
					double drho = dx * dx + dy * dy;
					odo_gain *= exp(-m_linearOdometryReliability * drho);
				}
				
				//double localScore = odo_gain * score(map, localPose, readings);
				double localScore = odo_gain * scoreNew(start,img_map,map, localPose, readings);

				//printf("gain %f %f %f -",m_linearOdometryReliability,odo_gain,localScore);

				if (localScore > currentScore)
				{
					currentScore = localScore;
					bestLocalPose = localPose;
				}
				c_iterations++;
			} while (move != Done);
			currentPose = bestLocalPose;
			//		cout << "currentScore=" << currentScore<< endl;
			// here we look for the best move;
		} while ((currentScore > bestScore || refinement < m_optRecursiveIterations)&&c_iterations<100);
		// cout << __PRETTY_FUNCTION__ << "bestScore=" << bestScore<< endl;
		// cout << __PRETTY_FUNCTION__ << "iterations=" << c_iterations<< endl;
		pnew = currentPose;
		//cout <<"score="<< currentScore << " refinement=" << refinement<<"  iter  "<<c_iterations<<"  pose "<< currentPose.x  << " " << currentPose.y << " "<<std::endl;
			
		return bestScore;
	}


	double ScanMatcher::optimize1(OrientedPoint &pnew, const ScanMatcherMap &map, const OrientedPoint &init, const double *readings) const
	{
		double bestScore = -1;
		OrientedPoint currentPose = init;
		double currentScore = score(map, currentPose, readings);
		double adelta = m_optAngularDelta, ldelta = m_optLinearDelta;
		unsigned int refinement = 0;
	//	cv::Mat img_map = cv::Mat(cv::Size(map.getMapSizeX(), map.getMapSizeY()), CV_32FC3, cv::Scalar(0,0,0)); 
	
		for (size_t j = 0; j < img_map.rows; j++)
		{
			for (size_t i = 0; i < img_map.cols; i++)
			{
				img_map.at<cv::Vec3f>(j, i)[0]= 0;
				img_map.at<cv::Vec3f>(j, i)[1]= 0;
				img_map.at<cv::Vec3f>(j, i)[2]= 0;
			}	
		}

		cv::Point start;
		getCurrImgBound(start,map,currentPose,readings);

		enum Move
		{
			Front,
			Back,
			Left,
			Right,
			TurnLeft,
			TurnRight,
			Done
		};
		/*	cout << __PRETTY_FUNCTION__<<  " readings: ";
			for (int i=0; i<m_laserBeams; i++){
				cout << readings[i] << " ";
			}
			cout << endl;
		*/
		int c_iterations = 0;
		do
		{
			if (bestScore >= currentScore)
			{
				refinement++;
				adelta *= .5;
				ldelta *= .5;
			}
			bestScore = currentScore;
			cout <<"score="<< currentScore << " refinement=" << refinement<<"  iter  "<<c_iterations<<"  pose "<< currentPose.x  << " " << currentPose.y << " "<<std::endl;
			//		cout <<  "pose=" << currentPose.x  << " " << currentPose.y << " " << currentPose.theta << endl;
			OrientedPoint bestLocalPose = currentPose;
			OrientedPoint localPose = currentPose;

			Move move = Front;
			do
			{
				localPose = currentPose;
				switch (move)
				{
				case Front:
					localPose.x += ldelta;
					move = Back;
					break;
				case Back:
					localPose.x -= ldelta;
					move = Left;
					break;
				case Left:
					localPose.y -= ldelta;
					move = Right;
					break;
				case Right:
					localPose.y += ldelta;
					move = TurnLeft;
					break;
				case TurnLeft:
					localPose.theta += adelta;
					move = TurnRight;
					break;
				case TurnRight:
					localPose.theta -= adelta;
					move = Done;
					break;
				default:;
				}

				double odo_gain = 1;
				if (m_angularOdometryReliability > 0.)
				{
					double dth = init.theta - localPose.theta;
					dth = atan2(sin(dth), cos(dth));
					dth *= dth;
					odo_gain *= exp(-m_angularOdometryReliability * dth);
				}
				if (m_linearOdometryReliability > 0.)
				{
					double dx = init.x - localPose.x;
					double dy = init.y - localPose.y;
					double drho = dx * dx + dy * dy;
					odo_gain *= exp(-m_linearOdometryReliability * drho);
				}
				double localScore = odo_gain * score(map, localPose, readings);
				//double localScore = odo_gain * scoreNew(start,img_map,map, localPose, readings);

				if (localScore > currentScore)
				{
					currentScore = localScore;
					bestLocalPose = localPose;
				}
				c_iterations++;
			} while (move != Done);
			currentPose = bestLocalPose;
			//		cout << "currentScore=" << currentScore<< endl;
			// here we look for the best move;
		} while ((currentScore > bestScore || refinement < m_optRecursiveIterations)&&c_iterations<100);
		// cout << __PRETTY_FUNCTION__ << "bestScore=" << bestScore<< endl;
		// cout << __PRETTY_FUNCTION__ << "iterations=" << c_iterations<< endl;
		pnew = currentPose;
		//cout <<"score="<< currentScore << " refinement=" << refinement<<"  iter  "<<c_iterations<<"  pose "<< currentPose.x  << " " << currentPose.y << " "<<std::endl;
			
		return bestScore;
	}

	struct ScoredMove
	{
		OrientedPoint pose;
		double score;
		double likelihood;
	};

	typedef std::list<ScoredMove> ScoredMoveList;

	double ScanMatcher::optimize(OrientedPoint &_mean, ScanMatcher::CovarianceMatrix &_cov, const ScanMatcherMap &map, const OrientedPoint &init, const double *readings) const
	{
		ScoredMoveList moveList;
		double bestScore = -1;
		OrientedPoint currentPose = init;
		ScoredMove sm = {currentPose, 0, 0};
		unsigned int matched = likelihoodAndScore(sm.score, sm.likelihood, map, currentPose, readings);
		double currentScore = sm.score;
		moveList.push_back(sm);
		double adelta = m_optAngularDelta, ldelta = m_optLinearDelta;
		unsigned int refinement = 0;
		int count = 0;
		cv::Mat img_map = cv::Mat(cv::Size(map.getMapSizeX(), map.getMapSizeY()), CV_32FC3, cv::Scalar(0,0,0)); 
	
		enum Move
		{
			Front,
			Back,
			Left,
			Right,
			TurnLeft,
			TurnRight,
			Done
		};
		do
		{
			if (bestScore >= currentScore)
			{
				refinement++;
				adelta *= .5;
				ldelta *= .5;
			}
			bestScore = currentScore;
			//		cout <<"score="<< currentScore << " refinement=" << refinement;
			//		cout <<  "pose=" << currentPose.x  << " " << currentPose.y << " " << currentPose.theta << endl;
			OrientedPoint bestLocalPose = currentPose;
			OrientedPoint localPose = currentPose;

			Move move = Front;
			do
			{
				localPose = currentPose;
				switch (move)
				{
				case Front:
					localPose.x += ldelta;
					move = Back;
					break;
				case Back:
					localPose.x -= ldelta;
					move = Left;
					break;
				case Left:
					localPose.y -= ldelta;
					move = Right;
					break;
				case Right:
					localPose.y += ldelta;
					move = TurnLeft;
					break;
				case TurnLeft:
					localPose.theta += adelta;
					move = TurnRight;
					break;
				case TurnRight:
					localPose.theta -= adelta;
					move = Done;
					break;
				default:;
				}
				double localScore, localLikelihood;

				double odo_gain = 1;
				if (m_angularOdometryReliability > 0.)
				{
					double dth = init.theta - localPose.theta;
					dth = atan2(sin(dth), cos(dth));
					dth *= dth;
					odo_gain *= exp(-m_angularOdometryReliability * dth);
				}
				if (m_linearOdometryReliability > 0.)
				{
					double dx = init.x - localPose.x;
					double dy = init.y - localPose.y;
					double drho = dx * dx + dy * dy;
					odo_gain *= exp(-m_linearOdometryReliability * drho);
				}
				localScore = odo_gain * score(map, localPose, readings);
				//localScore = odo_gain * scoreNew(img_map,map, localPose, readings);
				// update the score
				count++;
				matched = likelihoodAndScore(localScore, localLikelihood, map, localPose, readings);
				if (localScore > currentScore)
				{
					currentScore = localScore;
					bestLocalPose = localPose;
				}
				sm.score = localScore;
				sm.likelihood = localLikelihood; //+log(odo_gain);
				sm.pose = localPose;
				moveList.push_back(sm);
				// update the move list
			} while (move != Done);
			currentPose = bestLocalPose;
			// cout << __PRETTY_FUNCTION__ << "currentScore=" << currentScore<< endl;
			// here we look for the best move;
		} while (currentScore > bestScore || refinement < m_optRecursiveIterations);
		// cout << __PRETTY_FUNCTION__ << "bestScore=" << bestScore<< endl;
		// cout << __PRETTY_FUNCTION__ << "iterations=" << count<< endl;

		// normalize the likelihood
		double lmin = 1e9;
		double lmax = -1e9;
		for (ScoredMoveList::const_iterator it = moveList.begin(); it != moveList.end(); it++)
		{
			lmin = it->likelihood < lmin ? it->likelihood : lmin;
			lmax = it->likelihood > lmax ? it->likelihood : lmax;
		}
		// cout << "lmin=" << lmin << " lmax=" << lmax<< endl;
		for (ScoredMoveList::iterator it = moveList.begin(); it != moveList.end(); it++)
		{
			it->likelihood = exp(it->likelihood - lmax);
			// cout << "l=" << it->likelihood << endl;
		}
		// compute the mean
		OrientedPoint mean(0, 0, 0);
		double lacc = 0;
		for (ScoredMoveList::const_iterator it = moveList.begin(); it != moveList.end(); it++)
		{
			mean = mean + it->pose * it->likelihood;
			lacc += it->likelihood;
		}
		mean = mean * (1. / lacc);
		// OrientedPoint delta=mean-currentPose;
		// cout << "delta.x=" << delta.x << " delta.y=" << delta.y << " delta.theta=" << delta.theta << endl;
		CovarianceMatrix cov = {0., 0., 0., 0., 0., 0.};
		for (ScoredMoveList::const_iterator it = moveList.begin(); it != moveList.end(); it++)
		{
			OrientedPoint delta = it->pose - mean;
			delta.theta = atan2(sin(delta.theta), cos(delta.theta));
			cov.xx += delta.x * delta.x * it->likelihood;
			cov.yy += delta.y * delta.y * it->likelihood;
			cov.tt += delta.theta * delta.theta * it->likelihood;
			cov.xy += delta.x * delta.y * it->likelihood;
			cov.xt += delta.x * delta.theta * it->likelihood;
			cov.yt += delta.y * delta.theta * it->likelihood;
		}
		cov.xx /= lacc, cov.xy /= lacc, cov.xt /= lacc, cov.yy /= lacc, cov.yt /= lacc, cov.tt /= lacc;

		_mean = currentPose;
		_cov = cov;
		return bestScore;
	}

	void ScanMatcher::setLaserParameters(unsigned int beams, double *angles, const OrientedPoint &lpose)
	{
		/*if (m_laserAngles)
			delete [] m_laserAngles;
		*/
		assert(beams < LASER_MAXBEAMS);
		m_laserPose = lpose;
		m_laserBeams = beams;
		// m_laserAngles=new double[beams];
		memcpy(m_laserAngles, angles, sizeof(double) * m_laserBeams);
	}

	double ScanMatcher::likelihood(double &_lmax, OrientedPoint &_mean, CovarianceMatrix &_cov, const ScanMatcherMap &map, const OrientedPoint &p, const double *readings)
	{
		ScoredMoveList moveList;

		for (double xx = -m_llsamplerange; xx <= m_llsamplerange; xx += m_llsamplestep)
			for (double yy = -m_llsamplerange; yy <= m_llsamplerange; yy += m_llsamplestep)
				for (double tt = -m_lasamplerange; tt <= m_lasamplerange; tt += m_lasamplestep)
				{

					OrientedPoint rp = p;
					rp.x += xx;
					rp.y += yy;
					rp.theta += tt;

					ScoredMove sm;
					sm.pose = rp;

					likelihoodAndScore(sm.score, sm.likelihood, map, rp, readings);
					moveList.push_back(sm);
				}

		// OrientedPoint delta=mean-currentPose;
		// cout << "delta.x=" << delta.x << " delta.y=" << delta.y << " delta.theta=" << delta.theta << endl;
		// normalize the likelihood
		double lmax = -1e9;
		double lcum = 0;
		for (ScoredMoveList::const_iterator it = moveList.begin(); it != moveList.end(); it++)
		{
			lmax = it->likelihood > lmax ? it->likelihood : lmax;
		}
		for (ScoredMoveList::iterator it = moveList.begin(); it != moveList.end(); it++)
		{
			// it->likelihood=exp(it->likelihood-lmax);
			lcum += exp(it->likelihood - lmax);
			it->likelihood = exp(it->likelihood - lmax);
			// cout << "l=" << it->likelihood << endl;
		}

		OrientedPoint mean(0, 0, 0);
		double s = 0, c = 0;
		for (ScoredMoveList::const_iterator it = moveList.begin(); it != moveList.end(); it++)
		{
			mean = mean + it->pose * it->likelihood;
			s += it->likelihood * sin(it->pose.theta);
			c += it->likelihood * cos(it->pose.theta);
		}
		mean = mean * (1. / lcum);
		s /= lcum;
		c /= lcum;
		mean.theta = atan2(s, c);

		CovarianceMatrix cov = {0., 0., 0., 0., 0., 0.};
		for (ScoredMoveList::const_iterator it = moveList.begin(); it != moveList.end(); it++)
		{
			OrientedPoint delta = it->pose - mean;
			delta.theta = atan2(sin(delta.theta), cos(delta.theta));
			cov.xx += delta.x * delta.x * it->likelihood;
			cov.yy += delta.y * delta.y * it->likelihood;
			cov.tt += delta.theta * delta.theta * it->likelihood;
			cov.xy += delta.x * delta.y * it->likelihood;
			cov.xt += delta.x * delta.theta * it->likelihood;
			cov.yt += delta.y * delta.theta * it->likelihood;
		}
		cov.xx /= lcum, cov.xy /= lcum, cov.xt /= lcum, cov.yy /= lcum, cov.yt /= lcum, cov.tt /= lcum;

		_mean = mean;
		_cov = cov;
		_lmax = lmax;
		return log(lcum) + lmax;
	}

	double ScanMatcher::likelihood(double &_lmax, OrientedPoint &_mean, CovarianceMatrix &_cov, const ScanMatcherMap &map, const OrientedPoint &p,
								   Gaussian3 &odometry, const double *readings, double gain)
	{
		ScoredMoveList moveList;

		for (double xx = -m_llsamplerange; xx <= m_llsamplerange; xx += m_llsamplestep)
			for (double yy = -m_llsamplerange; yy <= m_llsamplerange; yy += m_llsamplestep)
				for (double tt = -m_lasamplerange; tt <= m_lasamplerange; tt += m_lasamplestep)
				{

					OrientedPoint rp = p;
					rp.x += xx;
					rp.y += yy;
					rp.theta += tt;

					ScoredMove sm;
					sm.pose = rp;

					likelihoodAndScore(sm.score, sm.likelihood, map, rp, readings);
					sm.likelihood += odometry.eval(rp) / gain;
					assert(!isnan(sm.likelihood));
					moveList.push_back(sm);
				}

		// OrientedPoint delta=mean-currentPose;
		// cout << "delta.x=" << delta.x << " delta.y=" << delta.y << " delta.theta=" << delta.theta << endl;
		// normalize the likelihood
		double lmax = -std::numeric_limits<double>::max();
		double lcum = 0;
		for (ScoredMoveList::const_iterator it = moveList.begin(); it != moveList.end(); it++)
		{
			lmax = it->likelihood > lmax ? it->likelihood : lmax;
		}
		for (ScoredMoveList::iterator it = moveList.begin(); it != moveList.end(); it++)
		{
			// it->likelihood=exp(it->likelihood-lmax);
			lcum += exp(it->likelihood - lmax);
			it->likelihood = exp(it->likelihood - lmax);
			// cout << "l=" << it->likelihood << endl;
		}

		OrientedPoint mean(0, 0, 0);
		double s = 0, c = 0;
		for (ScoredMoveList::const_iterator it = moveList.begin(); it != moveList.end(); it++)
		{
			mean = mean + it->pose * it->likelihood;
			s += it->likelihood * sin(it->pose.theta);
			c += it->likelihood * cos(it->pose.theta);
		}
		mean = mean * (1. / lcum);
		s /= lcum;
		c /= lcum;
		mean.theta = atan2(s, c);

		CovarianceMatrix cov = {0., 0., 0., 0., 0., 0.};
		for (ScoredMoveList::const_iterator it = moveList.begin(); it != moveList.end(); it++)
		{
			OrientedPoint delta = it->pose - mean;
			delta.theta = atan2(sin(delta.theta), cos(delta.theta));
			cov.xx += delta.x * delta.x * it->likelihood;
			cov.yy += delta.y * delta.y * it->likelihood;
			cov.tt += delta.theta * delta.theta * it->likelihood;
			cov.xy += delta.x * delta.y * it->likelihood;
			cov.xt += delta.x * delta.theta * it->likelihood;
			cov.yt += delta.y * delta.theta * it->likelihood;
		}
		cov.xx /= lcum, cov.xy /= lcum, cov.xt /= lcum, cov.yy /= lcum, cov.yt /= lcum, cov.tt /= lcum;

		_mean = mean;
		_cov = cov;
		_lmax = lmax;
		double v = log(lcum) + lmax;
		assert(!isnan(v));
		return v;
	}

	void ScanMatcher::setMatchingParameters(double urange, double range, double sigma, int kernsize, double lopt, double aopt, int iterations, double likelihoodSigma, unsigned int likelihoodSkip)
	{
		m_usableRange = urange;
		m_laserMaxRange = range;
		m_kernelSize = kernsize;
		m_optLinearDelta = lopt;
		m_optAngularDelta = aopt;
		m_optRecursiveIterations = iterations;
		m_gaussianSigma = sigma;
		m_likelihoodSigma = likelihoodSigma;
		m_likelihoodSkip = likelihoodSkip;
	}

};
