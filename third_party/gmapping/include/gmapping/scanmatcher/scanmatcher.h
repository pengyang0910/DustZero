#ifndef SCANMATCHER_H
#define SCANMATCHER_H

#include "gmapping/scanmatcher/icp.h"
#include "gmapping/scanmatcher/smmap.h"
#include <gmapping/utils/macro_params.h>
#include <gmapping/utils/stat.h>
#include <iostream>
#include <gmapping/utils/gvalues.h>
#include "opencv2/opencv.hpp"
#define LASER_MAXBEAMS 2048

namespace GMapping {

class ScanMatcher{
	public:
		typedef Covariance3 CovarianceMatrix;
		
		ScanMatcher();
		~ScanMatcher();
		double icpOptimize(OrientedPoint& pnew, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;
		double optimize(OrientedPoint& pnew, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;
		double optimize(OrientedPoint& mean, CovarianceMatrix& cov, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;
		double optimize1(OrientedPoint& pnew, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;
		
		double   registerScan(ScanMatcherMap& map, const OrientedPoint& p, const double* readings);
		double   computeActiveAreaAndRegisterScan(ScanMatcherMap& map, const OrientedPoint& p, const double* readings);
		void    initScanMap(ScanMatcherMap& map, std::vector<int8_t> mapData,int width,int height);
		void setLaserParameters
			(unsigned int beams, double* angles, const OrientedPoint& lpose);
		void setMatchingParameters
			(double urange, double range, double sigma, int kernsize, double lopt, double aopt, int iterations, double likelihoodSigma=1, unsigned int likelihoodSkip=0 );
		void invalidateActiveArea();
		void computeActiveArea(ScanMatcherMap& map, const OrientedPoint& p, const double* readings);

		inline double icpStep(OrientedPoint & pret, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;
		inline double score(const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;
		inline double scoreNew(cv::Point start,cv::Mat& img_map,const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;
		inline double scoreBest(const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;
		inline unsigned int likelihoodAndScore(double& s, double& l, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;
		inline unsigned int likelihoodAndScore1(double& s, double& l, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;
		
		double likelihood(double& lmax, OrientedPoint& mean, CovarianceMatrix& cov, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings);
		double likelihood(double& _lmax, OrientedPoint& _mean, CovarianceMatrix& _cov, const ScanMatcherMap& map, const OrientedPoint& p, Gaussian3& odometry, const double* readings, double gain=180.);
		inline const double* laserAngles() const { return m_laserAngles; }
		inline unsigned int laserBeams() const { return m_laserBeams; }
		inline void getCurrImgBound(cv::Point& start,const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;
		void setLinearPara(float reliability);
		void setAnglePara(float reliability);
		inline double getUnknownPt(int& unkownCnt,const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;
		static const double nullLikelihood;
		//inline bool getOptimizePose(int& unkownCnt,const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;
		
	protected:
		//state of the matcher
		bool m_activeAreaComputed;
		
		/**laser parameters*/
		unsigned int m_laserBeams;
		double       m_laserAngles[LASER_MAXBEAMS];
		//OrientedPoint m_laserPose;
		PARAM_SET_GET(OrientedPoint, laserPose, protected, public, public)
		PARAM_SET_GET(double, laserMaxRange, protected, public, public)
		/**scan_matcher parameters*/
		PARAM_SET_GET(double, usableRange, protected, public, public)
		PARAM_SET_GET(double, gaussianSigma, protected, public, public)
		PARAM_SET_GET(double, likelihoodSigma, protected, public, public)
		PARAM_SET_GET(int,    kernelSize, protected, public, public)
		PARAM_SET_GET(double, optAngularDelta, protected, public, public)
		PARAM_SET_GET(double, optLinearDelta, protected, public, public)
		PARAM_SET_GET(unsigned int, optRecursiveIterations, protected, public, public)
		PARAM_SET_GET(unsigned int, likelihoodSkip, protected, public, public)
		PARAM_SET_GET(double, llsamplerange, protected, public, public)
		PARAM_SET_GET(double, llsamplestep, protected, public, public)
		PARAM_SET_GET(double, lasamplerange, protected, public, public)
		PARAM_SET_GET(double, lasamplestep, protected, public, public)
		PARAM_SET_GET(bool, generateMap, protected, public, public)
		PARAM_SET_GET(double, enlargeStep, protected, public, public)
		PARAM_SET_GET(double, fullnessThreshold, protected, public, public)
		PARAM_SET_GET(double, angularOdometryReliability, protected, public, public)
		PARAM_SET_GET(double, linearOdometryReliability, protected, public, public)
		PARAM_SET_GET(double, freeCellRatio, protected, public, public)
		PARAM_SET_GET(unsigned int, initialBeamsSkip, protected, public, public)

		// allocate this large array only once
		IntPoint* m_linePoints;
};

inline double ScanMatcher::icpStep(OrientedPoint & pret, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const{
	const double * angle=m_laserAngles+m_initialBeamsSkip;
	OrientedPoint lp=p;
	lp.x+=cos(p.theta)*m_laserPose.x-sin(p.theta)*m_laserPose.y;
	lp.y+=sin(p.theta)*m_laserPose.x+cos(p.theta)*m_laserPose.y;
	lp.theta+=m_laserPose.theta;
	unsigned int skip=0;
	double freeDelta=map.getDelta()*m_freeCellRatio;
	std::list<PointPair> pairs;
	
	for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++){
		skip++;
		skip=skip>m_likelihoodSkip?0:skip;
		if (*r>m_usableRange||*r==0.0) continue;
		if (skip) continue;
		Point phit=lp;
		phit.x+=*r*cos(lp.theta+*angle);
		phit.y+=*r*sin(lp.theta+*angle);
		IntPoint iphit=map.world2map(phit);
		Point pfree=lp;
		pfree.x+=(*r-map.getDelta()*freeDelta)*cos(lp.theta+*angle);
		pfree.y+=(*r-map.getDelta()*freeDelta)*sin(lp.theta+*angle);
 		pfree=pfree-phit;
		IntPoint ipfree=map.world2map(pfree);
		bool found=false;
		Point bestMu(0.,0.);
		Point bestCell(0.,0.);
		for (int xx=-m_kernelSize; xx<=m_kernelSize; xx++)
		for (int yy=-m_kernelSize; yy<=m_kernelSize; yy++){
			IntPoint pr=iphit+IntPoint(xx,yy);
			IntPoint pf=pr+ipfree;
			//AccessibilityState s=map.storage().cellState(pr);
			//if (s&Inside && s&Allocated){
				const PointAccumulator& cell=map.cell(pr);
				const PointAccumulator& fcell=map.cell(pf);
				if (((double)cell )> m_fullnessThreshold && ((double)fcell )<m_fullnessThreshold){
					Point mu=phit-cell.mean();
					if (!found){
						bestMu=mu;
						bestCell=cell.mean();
						found=true;
					}else
						if((mu*mu)<(bestMu*bestMu)){
							bestMu=mu;
							bestCell=cell.mean();
						} 
						
				}
			//}
		}
		if (found){
			pairs.push_back(std::make_pair(phit, bestCell));
			//std::cerr << "(" << phit.x-bestCell.x << "," << phit.y-bestCell.y << ") ";
		}
		//std::cerr << std::endl;
	}
	
	OrientedPoint result(0,0,0);
	//double icpError=icpNonlinearStep(result,pairs);
	std::cerr << "result(" << pairs.size() << ")=" << result.x << " " << result.y << " " << result.theta << std::endl;
	pret.x=p.x+result.x;
	pret.y=p.y+result.y;
	pret.theta=p.theta+result.theta;
	pret.theta=atan2(sin(pret.theta), cos(pret.theta));
	return score(map, p, readings);
}

inline double ScanMatcher::score(const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const{
	double s=0;
	const double * angle=m_laserAngles+m_initialBeamsSkip;
	OrientedPoint lp=p;
	lp.x+=cos(p.theta)*m_laserPose.x-sin(p.theta)*m_laserPose.y;
	lp.y+=sin(p.theta)*m_laserPose.x+cos(p.theta)*m_laserPose.y;
	lp.theta+=m_laserPose.theta;
	unsigned int skip=0;
	double freeDelta=map.getDelta()*m_freeCellRatio;
	int cnt=0;
	for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++){
		skip++;
		skip=skip>m_likelihoodSkip?0:skip;
		if (skip||*r>m_usableRange||*r==0.0) continue;
		Point phit=lp;
		phit.x+=*r*cos(lp.theta+*angle);
		phit.y+=*r*sin(lp.theta+*angle);
		IntPoint iphit=map.world2map(phit);
		Point pfree=lp;
		pfree.x+=(*r-map.getDelta()*freeDelta)*cos(lp.theta+*angle);
		pfree.y+=(*r-map.getDelta()*freeDelta)*sin(lp.theta+*angle);
 		pfree=pfree-phit;
		IntPoint ipfree=map.world2map(pfree);
		bool found=false;
		Point bestMu(0.,0.);
		
		for (int xx=-m_kernelSize; xx<=m_kernelSize; xx++)
		for (int yy=-m_kernelSize; yy<=m_kernelSize; yy++){
			IntPoint pr=iphit+IntPoint(xx,yy);
			IntPoint pf=pr+ipfree;
			//AccessibilityState s=map.storage().cellState(pr);
			//if (s&Inside && s&Allocated){
				const PointAccumulator& cell=map.cell(pr);
				const PointAccumulator& fcell=map.cell(pf);
				if (((double)cell )> m_fullnessThreshold && ((double)fcell )<m_fullnessThreshold){
					Point mu=phit-cell.mean();
					if (!found){
						bestMu=mu;
						found=true;
						cnt++;
					}else
						bestMu=(mu*mu)<(bestMu*bestMu)?mu:bestMu;
				}
			//}
		}
		if (found)
			s+=exp(-1./m_gaussianSigma*bestMu*bestMu);
	}
	//printf("cnt is %d ",cnt);
	return s;
}

/* inline double ScanMatcher::score(const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const{
	double s=0;
	const double * angle=m_laserAngles+m_initialBeamsSkip;
	OrientedPoint lp=p;
	lp.x+=cos(p.theta)*m_laserPose.x-sin(p.theta)*m_laserPose.y;
	lp.y+=sin(p.theta)*m_laserPose.x+cos(p.theta)*m_laserPose.y;
	lp.theta+=m_laserPose.theta;
	unsigned int skip=0;
	double freeDelta=map.getDelta()*m_freeCellRatio;
	for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++){
		skip++;
		skip=skip>m_likelihoodSkip?0:skip;
		if (skip||*r>m_usableRange||*r==0.0) continue;
		Point phit=lp;
		phit.x+=*r*cos(lp.theta+*angle);
		phit.y+=*r*sin(lp.theta+*angle);
		IntPoint iphit=map.world2map(phit);
		Point pfree=lp;
		pfree.x+=(*r-map.getDelta()*freeDelta)*cos(lp.theta+*angle);
		pfree.y+=(*r-map.getDelta()*freeDelta)*sin(lp.theta+*angle);
 		pfree=pfree-phit;
		IntPoint ipfree=map.world2map(pfree);
		bool found=false;
		Point bestMu(0.,0.);
		for (int xx=-m_kernelSize; xx<=m_kernelSize; xx++)
		for (int yy=-m_kernelSize; yy<=m_kernelSize; yy++){
			IntPoint pr=iphit+IntPoint(xx,yy);
			IntPoint pf=pr+ipfree;
			//AccessibilityState s=map.storage().cellState(pr);
			//if (s&Inside && s&Allocated){
				const PointAccumulator& cell=map.cell(pr);
				//const PointAccumulator& fcell=map.cell(pf);
				if (((double)cell )> m_fullnessThreshold){
				//if (((double)cell )> m_fullnessThreshold && ((double)fcell )<m_fullnessThreshold){
					Point mu=phit-cell.mean();
					if (!found){
						bestMu=mu;
						found=true;
					}else
						bestMu=(mu*mu)<(bestMu*bestMu)?mu:bestMu;
				}
			//}
		}
		if (found)
			s+=exp(-1./m_gaussianSigma*bestMu*bestMu);
	}
	return s;
} */


inline double ScanMatcher::scoreNew(cv::Point start,cv::Mat & img_map,const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const{
	double s=0;
	const double * angle=m_laserAngles+m_initialBeamsSkip;
	OrientedPoint lp=p;
	lp.x+=cos(p.theta)*m_laserPose.x-sin(p.theta)*m_laserPose.y;
	lp.y+=sin(p.theta)*m_laserPose.x+cos(p.theta)*m_laserPose.y;
	lp.theta+=m_laserPose.theta;
	unsigned int skip=0;
	int cnt=0;
	double freeDelta=map.getDelta()*m_freeCellRatio;
	for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++){
		skip++;
		skip=skip>m_likelihoodSkip?0:skip;
		if (skip||*r>m_usableRange||*r==0.0) continue;
		Point phit=lp;
		phit.x+=*r*cos(lp.theta+*angle);
		phit.y+=*r*sin(lp.theta+*angle);
		IntPoint iphit=map.world2map(phit);
		Point pfree=lp;
		pfree.x+=(*r-map.getDelta()*freeDelta)*cos(lp.theta+*angle);
		pfree.y+=(*r-map.getDelta()*freeDelta)*sin(lp.theta+*angle);
 		pfree=pfree-phit;
		IntPoint ipfree=map.world2map(pfree);
		bool found=false;
		Point bestMu(0.,0.);
		
		// IntPoint pf=iphit+ipfree;

		int ipx = iphit.x- start.x;
		int ipy = iphit.y- start.y;

		if (ipy<0||ipy>=img_map.rows||ipx<0||ipx>=img_map.cols)
		{
			//printf(" out  %d  %d  %d  %d  %d  %d ",iphit.x,iphit.y,ipx,ipy,start.x,start.y);
			continue;
		}
		
		if(fabs(img_map.at<cv::Vec3f>(ipy, ipx)[0])>0.0001)
		{
			if (img_map.at<cv::Vec3f>(ipy, ipx)[0] > m_fullnessThreshold)
			{		
				bestMu=phit-Point(img_map.at<cv::Vec3f>(ipy, ipx)[1],img_map.at<cv::Vec3f>(ipy, ipx)[2]);
				found=true;
			}
			/* 	else if (freePix==0)
				{
					const PointAccumulator& fcell=map.cell(pf);
					double tmp =(double)fcell;
					img_map.at<cv::Vec3f>(pf.y, pf.x)[0]= tmp;
					if (tmp < m_fullnessThreshold)
					{
						bestMu=phit-Point(img_map.at<cv::Vec3f>(iphit.y, iphit.x)[1],img_map.at<cv::Vec3f>(iphit.y, iphit.x)[2]);
						found=true;
					}
					
				}
				 */
			
		}
		else
		{	
			const PointAccumulator& cell=map.cell(iphit);
			double tmp =(double)cell;

			Point mu=cell.mean();
			img_map.at<cv::Vec3f>(ipy, ipx)[0]= tmp;
			img_map.at<cv::Vec3f>(ipy, ipx)[1]= mu.x;
			img_map.at<cv::Vec3f>(ipy, ipx)[2]= mu.y;

			if(tmp>m_fullnessThreshold)
			{
				bestMu=phit-mu;
				found=true;
			}

			/* float freePix = img_map.at<cv::Vec3f>(pf.y, pf.x)[0];
			if(freePix < m_fullnessThreshold&&freePix>0)
			{
				bestMu=phit-mu;
				found=true;
			}
			else if (freePix==0)
			{
				const PointAccumulator& fcell=map.cell(pf);
				double tmp =(double)fcell;
				img_map.at<cv::Vec3f>(pf.y, pf.x)[0]= tmp;
				if (tmp < m_fullnessThreshold)
				{
					bestMu=phit-mu;
					found=true;
				}	
			} */
		}

		if (found)
		{
			cnt++;
			s+=exp(-1./m_gaussianSigma*bestMu*bestMu);
		}	
		
		continue; 




		if(img_map.at<cv::Vec3f>(iphit.y, iphit.x)[0]>m_fullnessThreshold)
		{
			bestMu=phit-Point(img_map.at<cv::Vec3f>(iphit.y, iphit.x)[1],img_map.at<cv::Vec3f>(iphit.y, iphit.x)[2]);
			found=true;
		}
		else
		{	
			IntPoint pr=iphit;
			IntPoint pf=pr+ipfree;
			const PointAccumulator& cell=map.cell(pr);
			const PointAccumulator& fcell=map.cell(pf);

			Point mu=cell.mean();
			img_map.at<cv::Vec3f>(iphit.y, iphit.x)[0]= (double)cell;
			img_map.at<cv::Vec3f>(iphit.y, iphit.x)[1]= mu.x;
			img_map.at<cv::Vec3f>(iphit.y, iphit.x)[2]= mu.y;

			if (((double)cell )> m_fullnessThreshold && ((double)fcell )<m_fullnessThreshold)
			{
				bestMu=phit-mu;
				found=true;
			}
		}

		if (found)
			s+=exp(-1./m_gaussianSigma*bestMu*bestMu);

		
		continue;



		for (int xx=-m_kernelSize; xx<=m_kernelSize; xx++)
		for (int yy=-m_kernelSize; yy<=m_kernelSize; yy++){
			IntPoint pr=iphit+IntPoint(xx,yy);
			IntPoint pf=pr+ipfree;
			//AccessibilityState s=map.storage().cellState(pr);
			//if (s&Inside && s&Allocated){
				const PointAccumulator& cell=map.cell(pr);
				const PointAccumulator& fcell=map.cell(pf);
				if (((double)cell )> m_fullnessThreshold && ((double)fcell )<m_fullnessThreshold){
					Point mu=phit-cell.mean();
					if (!found){
						bestMu=mu;
						found=true;
					}else
						bestMu=(mu*mu)<(bestMu*bestMu)?mu:bestMu;
				}
			//}
		}
		if (found)
			s+=exp(-1./m_gaussianSigma*bestMu*bestMu);
	}
	//printf("cnt is %d ",cnt);
	return s;
}




inline unsigned int ScanMatcher::likelihoodAndScore(double& s, double& l, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const{
	using namespace std;
	l=0;
	s=0;
	const double * angle=m_laserAngles+m_initialBeamsSkip;
	OrientedPoint lp=p;
	lp.x+=cos(p.theta)*m_laserPose.x-sin(p.theta)*m_laserPose.y;
	lp.y+=sin(p.theta)*m_laserPose.x+cos(p.theta)*m_laserPose.y;
	lp.theta+=m_laserPose.theta;
	double noHit=nullLikelihood/(m_likelihoodSigma);
	unsigned int skip=0;
	unsigned int c=0;
	double freeDelta=map.getDelta()*m_freeCellRatio;
	for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++){
		skip++;
		skip=skip>m_likelihoodSkip?0:skip;
		if (*r>m_usableRange) continue;
		if (skip) continue;
		Point phit=lp;
		phit.x+=*r*cos(lp.theta+*angle);
		phit.y+=*r*sin(lp.theta+*angle);
		IntPoint iphit=map.world2map(phit);
		Point pfree=lp;
		pfree.x+=(*r-freeDelta)*cos(lp.theta+*angle);
		pfree.y+=(*r-freeDelta)*sin(lp.theta+*angle);
		pfree=pfree-phit;
		IntPoint ipfree=map.world2map(pfree);
		bool found=false;
		Point bestMu(0.,0.);
		for (int xx=-m_kernelSize; xx<=m_kernelSize; xx++)
		for (int yy=-m_kernelSize; yy<=m_kernelSize; yy++){
			IntPoint pr=iphit+IntPoint(xx,yy);
			IntPoint pf=pr+ipfree;
			//AccessibilityState s=map.storage().cellState(pr);
			//if (s&Inside && s&Allocated){
				const PointAccumulator& cell=map.cell(pr);
				const PointAccumulator& fcell=map.cell(pf);
				if (((double)cell )>m_fullnessThreshold && ((double)fcell )<m_fullnessThreshold){
					Point mu=phit-cell.mean();
					if (!found){
						bestMu=mu;
						found=true;
					}else
						bestMu=(mu*mu)<(bestMu*bestMu)?mu:bestMu;
				}
			//}	
		}
		if (found){
			s+=exp(-1./m_gaussianSigma*bestMu*bestMu);
			c++;
		}
		if (!skip){
			double f=(-1./m_likelihoodSigma)*(bestMu*bestMu);
			l+=(found)?f:noHit;
		}
	}
	return c;
}


inline unsigned int ScanMatcher::likelihoodAndScore1(double& s, double& l, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const{
	using namespace std;
	l=0;
	s=0;
	const double * angle=m_laserAngles+m_initialBeamsSkip;
	OrientedPoint lp=p;
	lp.x+=cos(p.theta)*m_laserPose.x-sin(p.theta)*m_laserPose.y;
	lp.y+=sin(p.theta)*m_laserPose.x+cos(p.theta)*m_laserPose.y;
	lp.theta+=m_laserPose.theta;
	double noHit=nullLikelihood/(m_likelihoodSigma);
	unsigned int skip=0;
	unsigned int c=0;
	double freeDelta=map.getDelta()*m_freeCellRatio;
	for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++){
		skip++;
		skip=skip>m_likelihoodSkip?0:skip;
		if (*r>m_usableRange) continue;
		if (skip) continue;
		Point phit=lp;
		phit.x+=*r*cos(lp.theta+*angle);
		phit.y+=*r*sin(lp.theta+*angle);
		IntPoint iphit=map.world2map(phit);
		Point pfree=lp;
		pfree.x+=(*r-freeDelta)*cos(lp.theta+*angle);
		pfree.y+=(*r-freeDelta)*sin(lp.theta+*angle);
		//pfree=pfree-phit;
		IntPoint ipfree=map.world2map(pfree);
		ipfree = ipfree - iphit;
		bool found=false;
		Point bestMu(0.,0.);
		for (int xx=-1; xx<=1; xx++)
		for (int yy=-1; yy<=1; yy++){
			IntPoint pr=iphit+IntPoint(xx,yy);
			IntPoint pf=pr+ipfree;
			//AccessibilityState s=map.storage().cellState(pr);
			//if (s&Inside && s&Allocated){
				const PointAccumulator& cell=map.cell(pr);
				const PointAccumulator& fcell=map.cell(pf);
				if (((double)cell )>m_fullnessThreshold && ((double)fcell )<m_fullnessThreshold){
					Point mu=phit-cell.mean();
					if (!found){
						bestMu=mu;
						found=true;
					}else
						bestMu=(mu*mu)<(bestMu*bestMu)?mu:bestMu;
				}
			//}	
		}
		if (found){
			s+=exp(-1./m_gaussianSigma*bestMu*bestMu);
			c++;
		}
		if (!skip){
			double f=(-1./m_likelihoodSigma)*(bestMu*bestMu);
			l+=(found)?f:noHit;
		}
	}
	return c;
}


inline double ScanMatcher::scoreBest(const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const{
	double s=0;
	const double * angle=m_laserAngles+m_initialBeamsSkip;
	OrientedPoint lp=p;
	unsigned int skip=0;
	double freeDelta=map.getDelta()*m_freeCellRatio;  // freeDela 0.0707107 map.getDelta()  0.05
	//hitCnt=0;

	for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++){
		skip++;
		skip=skip>m_likelihoodSkip?0:skip;
		if (skip||*r>m_usableRange||*r==0.0) continue;
		Point phit=lp;

		float costh= cos(lp.theta+*angle);
		float sinth= sin(lp.theta+*angle);
		phit.x+=*r*costh;
		phit.y+=*r*sinth;
		IntPoint iphit=map.world2map(phit);
		Point pfree=lp;
		pfree.x+=(*r-map.getDelta()*m_freeCellRatio)*costh;
		pfree.y+=(*r-map.getDelta()*m_freeCellRatio)*sinth;
 		//pfree=pfree-phit;
		IntPoint ipfree=map.world2map(pfree);
		ipfree= ipfree -iphit;
		bool found=false;
		Point bestMu(0.,0.);

		// float tmp = map.getDelta()*freeDelta;
		//printf("tmp is %f ",tmp);
		
		bool isunKownFlag=true;

		//std::cout<<","<<ipfree.x<<" "<<ipfree.y;

		for (int xx=-2; xx<=2; xx++)
		for (int yy=-2; yy<=2; yy++){
			IntPoint pr=iphit+IntPoint(xx,yy);
			IntPoint pf=pr+ipfree;
			//AccessibilityState s=map.storage().cellState(pr);
			//if (s&Inside && s&Allocated){
			 	const PointAccumulator& cell=map.cell(pr);
				
				const PointAccumulator& fcell=map.cell(pf);

				if (((double)cell )>= 0){
					isunKownFlag =false;
				}
				
				if (((double)cell )> m_fullnessThreshold&& ((double)fcell )< m_fullnessThreshold){
				//if (((double)cell )> m_fullnessThreshold){
					Point mu=phit-cell.mean();
					
					if (!found){
						bestMu=mu;
						found=true;
					}else
						bestMu=(mu*mu)<(bestMu*bestMu)?mu:bestMu;
				} 
			//}
		}
		if (found)
		{
			s+=exp(-1./m_gaussianSigma*bestMu*bestMu);
			//hitCnt++;
	//		printf(" %d: %d %d ",hitCnt,iphit.x,iphit.y);
		}

		if (isunKownFlag==false)
		{
			//hitCnt++;
		}
		
			
	}
	//printf("hitCnt is %d ",hitCnt);
	return s;
}

inline double ScanMatcher::getUnknownPt(int& unkownCnt,const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const{
double s=0;
	const double * angle=m_laserAngles+m_initialBeamsSkip;
	OrientedPoint lp=p;
	// printf("x y is %f %f  %f  %f %f  %f  \n",lp.x,lp.y,lp.theta,m_laserPose.x,m_laserPose.y,m_laserPose.theta);
	/*lp.x+=cos(p.theta)*m_laserPose.x-sin(p.theta)*m_laserPose.y;
	lp.y+=sin(p.theta)*m_laserPose.x+cos(p.theta)*m_laserPose.y;
	lp.theta+=m_laserPose.theta; */
	unsigned int skip=0;
	double freeDelta=map.getDelta()*m_freeCellRatio;
	unkownCnt=0;

	for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++){
		skip++;
		skip=skip>m_likelihoodSkip?0:skip;
		if (skip||*r>m_usableRange||*r==0.0) continue;
		Point phit=lp;

		float costh= cos(lp.theta+*angle);
		float sinth= sin(lp.theta+*angle);
		phit.x+=*r*costh;
		phit.y+=*r*sinth;
		IntPoint iphit=map.world2map(phit);
		Point pfree=lp;
		pfree.x+=(*r-map.getDelta()*freeDelta)*costh;
		pfree.y+=(*r-map.getDelta()*freeDelta)*sinth;
 		pfree=pfree-phit;
		IntPoint ipfree=map.world2map(pfree);
		bool found=false;
		Point bestMu(0.,0.);

		// float tmp = map.getDelta()*freeDelta;
		//printf("tmp is %f ",tmp);
		
		bool isunKownFlag=true;

		for (int xx=-2; xx<=2; xx++)
		for (int yy=-2; yy<=2; yy++){
			IntPoint pr=iphit+IntPoint(xx,yy);
			IntPoint pf=pr+ipfree;
			//AccessibilityState s=map.storage().cellState(pr);
			//if (s&Inside && s&Allocated){
			 	const PointAccumulator& cell=map.cell(pr);
				
				const PointAccumulator& fcell=map.cell(pf);

				if (((double)cell )>= 0){
					isunKownFlag =false;
				}
				
				if (((double)cell )> m_fullnessThreshold){
				//if (((double)cell )> m_fullnessThreshold){
					Point mu=phit-cell.mean();
					//float tmp1 = (double)fcell;
					/* if (tmp1>0)
					{
						printf(" out2  %d  %d  %d  %d  %f",iphit.x,iphit.y,pf.x,pf.y);
					} */
					
					if (!found){
						bestMu=mu;
						found=true;
					}else
						bestMu=(mu*mu)<(bestMu*bestMu)?mu:bestMu;
				} 
			//}
		}
		if (found)
		{
			s+=exp(-1./m_gaussianSigma*bestMu*bestMu);
			//hitCnt++;
	//		printf(" %d: %d %d ",hitCnt,iphit.x,iphit.y);
		}

		if (isunKownFlag==true)
		{
			unkownCnt++;
		}
		
			
	}
	//printf("hitCnt is %d ",hitCnt);
	return s;
}


};

#endif
