/*
 * @Descripttion: 
 * @version: 
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2019-10-09 16:05:09
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2020-09-21 15:34:01
 */
#include <string>
#include <deque>
#include <list>
#include <map>
#include <set>
#include <fstream>
#include <iomanip>
#include <time.h>
#include <gmapping/utils/stat.h>
#include "gmapping/gridfastslam/gridslamprocessor.h"
#include "xutils.h"
//#define MAP_CONSISTENCY_CHECK
//#define GENERATE_TRAJECTORIES

namespace GMapping {

const double m_distanceThresholdCheck = 20;
 
using namespace std;

  GridSlamProcessor::GridSlamProcessor(): m_infoStream(cout){
    
    period_ = 5.0;
    m_obsSigmaGain=1;
    m_resampleThreshold=0.5;//test
    m_minimumScore=0.;
  }
  
  GridSlamProcessor::GridSlamProcessor(const GridSlamProcessor& gsp) 
    :last_update_time_(0.0), m_particles(gsp.m_particles), m_infoStream(cout){
    
    period_ = 5.0;

    m_obsSigmaGain=gsp.m_obsSigmaGain;
    m_resampleThreshold=gsp.m_resampleThreshold;
    m_minimumScore=gsp.m_minimumScore;
    
    m_beams=gsp.m_beams;
    m_indexes=gsp.m_indexes;
    m_motionModel=gsp.m_motionModel;
    m_resampleThreshold=gsp.m_resampleThreshold;
    m_matcher=gsp.m_matcher;
    
    m_count=gsp.m_count;
    m_readingCount=gsp.m_readingCount;
    m_lastPartPose=gsp.m_lastPartPose;
    m_pose=gsp.m_pose;
    m_odoPose=gsp.m_odoPose;
    m_linearDistance=gsp.m_linearDistance;
    m_angularDistance=gsp.m_angularDistance;
    m_neff=gsp.m_neff;
	
    cerr << "FILTER COPY CONSTRUCTOR" << endl;
    cerr << "m_odoPose=" << m_odoPose.x << " " <<m_odoPose.y << " " << m_odoPose.theta << endl;
    cerr << "m_lastPartPose=" << m_lastPartPose.x << " " <<m_lastPartPose.y << " " << m_lastPartPose.theta << endl;
    cerr << "m_linearDistance=" << m_linearDistance << endl;
    cerr << "m_angularDistance=" << m_linearDistance << endl;
    
		
    m_xmin=gsp.m_xmin;
    m_ymin=gsp.m_ymin;
    m_xmax=gsp.m_xmax;
    m_ymax=gsp.m_ymax;
    m_delta=gsp.m_delta;
    
    m_regScore=gsp.m_regScore;
    m_critScore=gsp.m_critScore;
    m_maxMove=gsp.m_maxMove;
    
    m_linearThresholdDistance=gsp.m_linearThresholdDistance;
    m_angularThresholdDistance=gsp.m_angularThresholdDistance;
    m_obsSigmaGain=gsp.m_obsSigmaGain;
    
#ifdef MAP_CONSISTENCY_CHECK
    cerr << __PRETTY_FUNCTION__ <<  ": trajectories copy.... ";
#endif
    TNodeVector v=gsp.getTrajectories();
    for (unsigned int i=0; i<v.size(); i++){
		m_particles[i].node=v[i];
    }
#ifdef MAP_CONSISTENCY_CHECK
    cerr <<  "end" << endl;
#endif


    cerr  << "Tree: normalizing, resetting and propagating weights within copy construction/cloneing ..." ;
    updateTreeWeights(false);
    cerr  << ".done!" <<endl;
  }
  
  GridSlamProcessor::GridSlamProcessor(std::ostream& infoS): m_infoStream(infoS){
    period_ = 5.0;
    m_obsSigmaGain=1;
    m_resampleThreshold=0.5;//test
    m_minimumScore=0.;

    xlog.Enable(true, true); //test
    xlog.Initialise("gmapping.log");
    xlog.SetThreshold(XLOG_LEVEL_INFO);

    xlog.Info("xxxxxxqqqqqqq, period_:%.4f", period_);
  }

  GridSlamProcessor* GridSlamProcessor::clone() const {
# ifdef MAP_CONSISTENCY_CHECK
    cerr << __PRETTY_FUNCTION__ << ": performing preclone_fit_test" << endl;
    typedef std::map<autoptr< Array2D<PointAccumulator> >::reference* const, int> PointerMap;
    PointerMap pmap;
	for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++){
	  const ScanMatcherMap& m1(it->map);
	  const HierarchicalArray2D<PointAccumulator>& h1(m1.storage());
 	  for (int x=0; x<h1.getXSize(); x++){
	    for (int y=0; y<h1.getYSize(); y++){
	      const autoptr< Array2D<PointAccumulator> >& a1(h1.m_cells[x][y]);
	      if (a1.m_reference){
		PointerMap::iterator f=pmap.find(a1.m_reference);
		if (f==pmap.end())
		  pmap.insert(make_pair(a1.m_reference, 1));
		else
		  f->second++;
	      }
	    }
	  }
	}
	cerr << __PRETTY_FUNCTION__ <<  ": Number of allocated chunks" << pmap.size() << endl;
	for(PointerMap::const_iterator it=pmap.begin(); it!=pmap.end(); it++)
	  assert(it->first->shares==(unsigned int)it->second);

	cerr << __PRETTY_FUNCTION__ <<  ": SUCCESS, the error is somewhere else" << endl;
# endif
	GridSlamProcessor* cloned=new GridSlamProcessor(*this);
	
# ifdef MAP_CONSISTENCY_CHECK
	cerr << __PRETTY_FUNCTION__ <<  ": trajectories end" << endl;
	cerr << __PRETTY_FUNCTION__ << ": performing afterclone_fit_test" << endl;
	ParticleVector::const_iterator jt=cloned->m_particles.begin();
	for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++){
	  const ScanMatcherMap& m1(it->map);
	  const ScanMatcherMap& m2(jt->map);
	  const HierarchicalArray2D<PointAccumulator>& h1(m1.storage());
	  const HierarchicalArray2D<PointAccumulator>& h2(m2.storage());
	  jt++;
 	  for (int x=0; x<h1.getXSize(); x++){
	    for (int y=0; y<h1.getYSize(); y++){
	      const autoptr< Array2D<PointAccumulator> >& a1(h1.m_cells[x][y]);
	      const autoptr< Array2D<PointAccumulator> >& a2(h2.m_cells[x][y]);
	      assert(a1.m_reference==a2.m_reference);
	      assert((!a1.m_reference) || !(a1.m_reference->shares%2));
	    }
	  }
	}
	cerr << __PRETTY_FUNCTION__ <<  ": SUCCESS, the error is somewhere else" << endl;
# endif
	return cloned;
}
  
  GridSlamProcessor::~GridSlamProcessor(){
    cerr << __PRETTY_FUNCTION__ << ": Start" << endl;
    cerr << __PRETTY_FUNCTION__ << ": Deleting tree" << endl;
    for (std::vector<Particle>::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
#ifdef TREE_CONSISTENCY_CHECK		
      TNode* node=it->node;
      while(node)
	node=node->parent;
      cerr << "@" << endl;
#endif
      if (it->node)
	delete it->node;
      //cout << "l=" << it->weight<< endl;
    }
    
# ifdef MAP_CONSISTENCY_CHECK
    cerr << __PRETTY_FUNCTION__ << ": performing predestruction_fit_test" << endl;
    typedef std::map<autoptr< Array2D<PointAccumulator> >::reference* const, int> PointerMap;
    PointerMap pmap;
    for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++){
      const ScanMatcherMap& m1(it->map);
      const HierarchicalArray2D<PointAccumulator>& h1(m1.storage());
      for (int x=0; x<h1.getXSize(); x++){
	for (int y=0; y<h1.getYSize(); y++){
	  const autoptr< Array2D<PointAccumulator> >& a1(h1.m_cells[x][y]);
	  if (a1.m_reference){
	    PointerMap::iterator f=pmap.find(a1.m_reference);
	    if (f==pmap.end())
	      pmap.insert(make_pair(a1.m_reference, 1));
	    else
	      f->second++;
	  }
	}
      }
    }
    cerr << __PRETTY_FUNCTION__ << ": Number of allocated chunks" << pmap.size() << endl;
    for(PointerMap::const_iterator it=pmap.begin(); it!=pmap.end(); it++)
      assert(it->first->shares>=(unsigned int)it->second);
    cerr << __PRETTY_FUNCTION__ << ": SUCCESS, the error is somewhere else" << endl;
# endif
  }


		
  void GridSlamProcessor::setMatchingParameters (double urange, double range, double sigma, int kernsize, double lopt, double aopt, 
						 int iterations, double likelihoodSigma, double likelihoodGain, unsigned int likelihoodSkip){
    m_obsSigmaGain=likelihoodGain;
    m_matcher.setMatchingParameters(urange, range, sigma, kernsize, lopt, aopt, iterations, likelihoodSigma, likelihoodSkip);
    //m_matcher2.setMatchingParameters(urange, range, sigma, kernsize, lopt, aopt, iterations, likelihoodSigma, likelihoodSkip);
    //m_matcher3.setMatchingParameters(urange, range, sigma, kernsize, lopt, aopt, iterations, likelihoodSigma, likelihoodSkip);
    if (m_infoStream)
      m_infoStream << " -maxUrange "<< urange
		   << " -maxUrange "<< range
		   << " -sigma     "<< sigma
		   << " -kernelSize "<< kernsize
		   << " -lstep "    << lopt
		   << " -lobsGain " << m_obsSigmaGain
		   << " -astep "    << aopt << endl;
    
    
  }
  
void GridSlamProcessor::setMotionModelParameters
(double srr, double srt, double str, double stt){
  m_motionModel.srr=srr;
  m_motionModel.srt=srt;
  m_motionModel.str=str;
  m_motionModel.stt=stt;	
  
  if (m_infoStream)
    m_infoStream << " -srr "<< srr 	<< " -srt "<< srt  
		 << " -str "<< str 	<< " -stt "<< stt << endl;
  
}
  
  void GridSlamProcessor::setUpdateDistances(double linear, double angular, double resampleThreshold){
    m_linearThresholdDistance=linear; 
    m_angularThresholdDistance=angular;
    m_resampleThreshold=resampleThreshold;	
    if (m_infoStream)
      m_infoStream << " -linearUpdate " << linear
		   << " -angularUpdate "<< angular
		   << " -resampleThreshold " << m_resampleThreshold << endl;
  }
  
  //HERE STARTS THE BEEF

  GridSlamProcessor::Particle::Particle(const ScanMatcherMap& m):
    map(m), pose(0,0,0), weight(0), weightSum(0), gweight(0), previousIndex(0){
    node=0;
  }
  
  
  void GridSlamProcessor::setSensorMap(const SensorMap& smap){
    
    /*
      Construct the angle table for the sensor
      
      FIXME For now detect the readings of only the front laser, and assume its pose is in the center of the robot 
    */
    
    SensorMap::const_iterator laser_it=smap.find(std::string("FLASER"));
    if (laser_it==smap.end()){
      cerr << "Attempting to load the new carmen log format" << endl;
      laser_it=smap.find(std::string("ROBOTLASER1"));
      assert(laser_it!=smap.end());
    }
    const RangeSensor* rangeSensor=dynamic_cast<const RangeSensor*>((laser_it->second));
    assert(rangeSensor && rangeSensor->beams().size());
    unsigned int tempNum = rangeSensor->beams().size();
    m_beams=static_cast<unsigned int>(rangeSensor->beams().size());
    double* angles=new double[rangeSensor->beams().size()];
    for (unsigned int i=0; i<tempNum; i++){
      angles[i]=rangeSensor->beams()[i].pose.theta;
    }
    m_matcher.setLaserParameters(tempNum, angles, rangeSensor->getPose());
    delete [] angles;
  }
  
  void GridSlamProcessor::init(unsigned int size, double xmin, double ymin, double xmax, double ymax, double delta, OrientedPoint initialPose){
    m_xmin=xmin;
    m_ymin=ymin;
    m_xmax=xmax;
    m_ymax=ymax;
    m_delta=delta;

    //xlog.Info("gridslamprocess init odom_x:%.8lf, odom_y:%.8lf, odom_pa:%.8lf", initialPose.x, initialPose.y, initialPose.theta);//test
    if (m_infoStream)
      m_infoStream 
	<< " -xmin "<< m_xmin
	<< " -xmax "<< m_xmax
	<< " -ymin "<< m_ymin
	<< " -ymax "<< m_ymax
	<< " -delta "<< m_delta
	<< " -particles "<< size << endl;

    

    m_particles.clear();
    TNode* node=new TNode(initialPose, 0, 0, 0);
    ScanMatcherMap lmap(Point(xmin+xmax, ymin+ymax)*.5, xmax-xmin, ymax-ymin, delta);
    for (unsigned int i=0; i<size; i++){
      m_particles.push_back(Particle(lmap));
      m_particles.back().pose=initialPose;
      m_particles.back().previousPose=initialPose;
      m_particles.back().setWeight(0);
      m_particles.back().previousIndex=0;
      
		// this is not needed
		//		m_particles.back().node=new TNode(initialPose, 0, node, 0);

		// we use the root directly
		m_particles.back().node= node;
    }
    m_neff=(double)size;
    m_count=0;
    m_readingCount=0;
    m_linearDistance=m_angularDistance=0;
  }


  // void GridSlamProcessor::initWithMap( ScanMatcherMap lmap,unsigned int size, double xmin, double ymin, double xmax, double ymax, double delta, OrientedPoint initialPose)
  // {
      //  m_xmin=xmin;
//  m_ymin=ymin;
//  m_xmax=xmax;
//  m_ymax=ymax;
//  m_delta=delta;
//  m_particles.clear();
//  TNode* node=new TNode(initialPose, 0, 0, 0);
//  ScanMatcherMap lmap(Point(xmin+xmax, ymin+ymax)*.5, xmax-xmin, ymax-ymin, delta);
//  for (unsigned int i=0; i<size; i++){
  //  m_particles.push_back(Particle(lmap));
  //  m_particles.back().pose=initialPose;
  //  m_particles.back().previousPose=initialPose;
  //  m_particles.back().setWeight(0);
  //  m_particles.back().previousIndex=0;
  //  node=new TNode(initialPose, 0, node, 0);n
	// m_particles.back().node= node;
//  }
//  m_neff=(double)size;
//  m_count=0;
//  m_readingCount=0;
//  m_linearDistance=m_angularDistance=0;
  // }


  void GridSlamProcessor::processTruePos(const OdometryReading& o){
    const OdometrySensor* os=dynamic_cast<const OdometrySensor*>(o.getSensor());
    if (os && os->isIdeal() && m_outputStream){
      m_outputStream << setiosflags(ios::fixed) << setprecision(3);
      m_outputStream <<  "SIMULATOR_POS " <<  o.getPose().x << " " << o.getPose().y << " " ;
      m_outputStream << setiosflags(ios::fixed) << setprecision(6) << o.getPose().theta << " " <<  o.getTime() << endl;
    }
  }
  
  bool GridSlamProcessor::processScan(const RangeReading & reading, int adaptParticles,bool odomErr){
    
    /**retireve the position from the reading, and compute the odometry*/
    OrientedPoint relPose=reading.getPose();
    if (!m_count){
      m_lastPartPose=m_odoPose=relPose;
    }
    

    float ratio=0;
    int partIndex =0;
    //write the state of the reading and update all the particles using the motion model
    for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
      OrientedPoint& pose(it->pose);
    //  std::cout <<"motion particle "<<partIndex<< ":" << it->pose.x<<" "<<it->pose.y<<" "<<it->pose.theta;	
    
      pose=m_motionModel.drawFromMotion(it->pose, relPose, m_odoPose);
      ratio= it->amclScore;
      partIndex++;
    //  std::cout <<"- "<< it->pose.x<<" "<<it->pose.y<<" "<<it->pose.theta << std::endl;	
    }

    // update the output file
    if (m_outputStream.is_open()){
      m_outputStream << setiosflags(ios::fixed) << setprecision(6);
      m_outputStream << "ODOM ";
      m_outputStream << setiosflags(ios::fixed) << setprecision(3) << m_odoPose.x << " " << m_odoPose.y << " ";
      m_outputStream << setiosflags(ios::fixed) << setprecision(6) << m_odoPose.theta << " ";
      m_outputStream << reading.getTime();
      m_outputStream << endl;
    }
    if (m_outputStream.is_open()){
      m_outputStream << setiosflags(ios::fixed) << setprecision(6);
      m_outputStream << "ODO_UPDATE "<< m_particles.size() << " ";
      for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
	OrientedPoint& pose(it->pose);
	m_outputStream << setiosflags(ios::fixed) << setprecision(3) << pose.x << " " << pose.y << " ";
	m_outputStream << setiosflags(ios::fixed) << setprecision(6) << pose.theta << " " << it-> weight << " ";
      }
      m_outputStream << reading.getTime();
      m_outputStream << endl;
    }
    
    //invoke the callback
    onOdometryUpdate();
    

    // accumulate the robot translation and rotation
    OrientedPoint move=relPose-m_odoPose;
    move.theta=atan2(sin(move.theta), cos(move.theta));
    m_linearDistance+=sqrt(move*move);
    m_angularDistance+=fabs(move.theta);
    
    // if the robot jumps throw a warning
    if (m_linearDistance>m_distanceThresholdCheck){
      xlog.Error("***********************************************************************");
      xlog.Error("********** Error: m_distanceThresholdCheck overridden!!!! *************");
      xlog.Error("m_distanceThresholdCheck=%.3f", m_distanceThresholdCheck );
      xlog.Error("Old Odometry Pose= %.3f %.3f %.3f", m_odoPose.x, m_odoPose.y, m_odoPose.theta);
      xlog.Error("New Odometry Pose (reported from observation)= %.ef %.3f %.3f", relPose.x, relPose.y, relPose.theta);
      xlog.Error("***********************************************************************");
      xlog.Error("** The Odometry has a big jump here. This is probably a bug in the   **");
      xlog.Error("** odometry/laser input. We continue now, but the result is probably **");
      xlog.Error("** crap or can lead to a core dump since the map doesn't fit.... C&G **");
      xlog.Error("***********************************************************************");
    }
    
    m_odoPose=relPose;
    
    bool processed=false;
    if (odomErr&&(reading.getTime() - last_update_time_) <10&&m_linearDistance>0.05)
    {
       printf("------------amcl score is low! don't match-------\n");
       return processed;
    }

    if (m_count<20)
    {
      m_matcher.setLinearPara(3);
    }
    /* else if (m_count<100)
    {
      m_matcher.setLinearPara(1);
    } */
    else m_matcher.setLinearPara(1);
    
    int unkownCnt = 0;
    Particle best = getParticles()[getBestParticleIndex()];
    OrientedPoint corrected;
    double score, l, s;
    // matcher.computeActiveAreaAndRegisterScan(smap, n->pose, &((*n->reading)[0]));
    assert(reading.size() == m_beams);
    double *plainReading = new double[m_beams];
    int validPts = 0;
    bool isLaserValid = true;
    for (unsigned int i = 0; i < m_beams; i++)
    {
      plainReading[i] = reading[i];
      if (plainReading[i] > 0.2 && plainReading[i] < 4)
      {
        validPts++;
      }
    }

    if (m_count > 0)
    {
      float Newscore = m_matcher.getUnknownPt(unkownCnt, best.map, best.pose, plainReading);
      float percent= 0;
      if(validPts>0)  percent = 100*unkownCnt / validPts;
      xlog.Info("percent is %f score is %f   %d  %d",percent,Newscore,unkownCnt,validPts);
      if (percent > 50)
      {
        isLaserValid = false;
      }
    }

    /*xlog.Debug("m_count:%d, m_linearDistance:%f, m_linearThresholdDistance:%f, m_angularDistance:%f, m_angularThresholdDistance:%f, reading.getTime():%f, last_update_time_:%f, period_:%.4f, reading.st:%f",\
                    m_count, m_linearDistance, m_linearThresholdDistance, m_angularDistance, m_angularThresholdDistance,\
                      reading.getTime(), last_update_time_, period_, reading.getTime());
    */
    // process a scan only if the robot has traveled a given distance or a certain amount of time has elapsed
    if (! m_count || m_linearDistance>=m_linearThresholdDistance || m_angularDistance>=m_angularThresholdDistance
    || (period_ >= 0.0 && (reading.getTime() - last_update_time_) > period_))
    {
      //xlog.Info("update condition");
      last_update_time_ = reading.getTime();      

      if (m_outputStream.is_open())
      {
        m_outputStream << setiosflags(ios::fixed) << setprecision(6);
        m_outputStream << "FRAME " <<  m_readingCount;
        m_outputStream << " " << m_linearDistance;
        m_outputStream << " " << m_angularDistance << endl;
      }
      
      if (m_infoStream)
	        m_infoStream << "update frame " <<  m_readingCount << endl
		     << "update ld=" << m_linearDistance << " ad=" << m_angularDistance << endl;
      
      xlog.Debug("Laser Pose= %.3f %.3f %.3f", 
          reading.getPose().x, reading.getPose().y, reading.getPose().theta);
      
      
      //this is for converting the reading in a scan-matcher feedable form
      assert(reading.size()==m_beams);
      double * plainReading = new double[m_beams];
      int tmpCnt=0;
      for(unsigned int i=0; i<m_beams; i++)
      {
	      plainReading[i]=reading[i];
        if(reading[i]<0.2)
           tmpCnt++;
      }
      // %d  %d ",tmpCnt,m_beams);
      m_infoStream << "m_count " << m_count << endl;

      RangeReading* reading_copy = new RangeReading(reading.size(),
                               &(reading[0]),
                               static_cast<const RangeSensor*>(reading.getSensor()),
                               reading.getTime());
      if (isLaserValid == false)
      {
        int index = 0;
        xlog.Error("isLaserValid true! Registering Scans:");
        // TNodeVector::iterator node_it=oldGeneration.begin();
        for (ParticleVector::iterator it = m_particles.begin(); it != m_particles.end(); it++)
        {
          // create a new node in the particle tree and add it to the old tree
          // BEGIN: BUILDING TREE
          TNode *node = 0;
          node = new TNode(it->pose, 0.0, it->node, 0);

          // node->reading=0;
          node->reading = reading_copy;
          it->node = node;

          // END: BUILDING TREE
          m_matcher.invalidateActiveArea();
          m_matcher.computeActiveArea(it->map, it->pose, plainReading);
          m_matcher.registerScan(it->map, it->pose, plainReading);
          it->previousIndex = index;
          index++;
          // it++;
        }
      }
      else if (m_count>0)
      {
        static uint64_t maxTimeConsume_us = 0;
        static uint64_t maxResampleTime_us = 0;
        //xlog.Info("before scanMatch");       
        uint64_t startF_us, endF_us;  
		startF_us = getTimeStap_us();
        scanMatch(plainReading);     
		endF_us = getTimeStap_us();
        uint64_t match_time= (endF_us - startF_us);
        xlog.Info("scanmatch time is %lld",match_time);
        if(maxTimeConsume_us < (endF_us - startF_us))
          maxTimeConsume_us = endF_us - startF_us;

        //xlog.Info("after scanMatch, maxTime=%.4f", maxTimeConsume);
        if (m_outputStream.is_open())
        {
          m_outputStream << "LASER_READING "<< reading.size() << " ";
          m_outputStream << setiosflags(ios::fixed) << setprecision(2);
	        for (RangeReading::const_iterator b=reading.begin(); b!=reading.end(); b++)
          {
	          m_outputStream << *b << " ";
	        }
          OrientedPoint p=reading.getPose();
          m_outputStream << setiosflags(ios::fixed) << setprecision(6);
          m_outputStream << p.x << " " << p.y << " " << p.theta << " " << reading.getTime()<< endl;
          m_outputStream << "SM_UPDATE "<< m_particles.size() << " ";
          for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++)
          {
            const OrientedPoint& pose=it->pose;
            m_outputStream << setiosflags(ios::fixed) << setprecision(3) <<  pose.x << " " << pose.y << " ";
            m_outputStream << setiosflags(ios::fixed) << setprecision(6) <<  pose.theta << " " << it-> weight << " ";
          }
	        m_outputStream << endl;
	      }
	      onScanmatchUpdate();
	      updateTreeWeights(false);	
        if (m_infoStream)
        {
          m_infoStream << "neff= " << m_neff  << endl;
        }
        if (m_outputStream.is_open())
        {
          m_outputStream << setiosflags(ios::fixed) << setprecision(6);
          m_outputStream << "NEFF " << m_neff << endl;
        }
        //xlog.Info("before resample");g
		startF_us = getTimeStap_us();
 	      resample(plainReading, adaptParticles, reading_copy);
		  endF_us = getTimeStap_us();
        if(maxResampleTime_us < (endF_us - startF_us))
          maxResampleTime_us = endF_us - startF_us;
        //xlog.Info("after resample maxResampleTime=%.2f", maxResampleTime);
      } 
      else 
      {
        m_infoStream << "Registering First Scan"<< endl;
        for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++)
        {	
          m_matcher.invalidateActiveArea();
          m_matcher.computeActiveArea(it->map, it->pose, plainReading);
          m_matcher.registerScan(it->map, it->pose, plainReading);
          // cyr: not needed anymore, particles refer to the root in the beginning!
          TNode* node=new	TNode(it->pose, 0., it->node,  0);
          //node->reading=0;
            node->reading = reading_copy;
          it->node=node;
	  
	      }
      }
      //		cerr  << "Tree: normalizing, resetting and propagating weights at the end..." ;
      updateTreeWeights(false);
      //		cerr  << ".done!" <<endl;   
      delete [] plainReading;
      m_lastPartPose=m_odoPose; //update the past pose for the next iteration
      m_linearDistance=0;
      m_angularDistance=0;
      m_count++;
      processed=true;
      
      //keep ready for the next step
      int partIndex=0;
      for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++)
      {
	      it->previousPose=it->pose;
        partIndex++;
        //if (m_infoStream)
 //       std::cout <<partIndex<< ":" << it->pose.x<<" "<<it->pose.y<<" "<<it->pose.theta << std::endl;	
      }
      
    }
    if (m_outputStream.is_open())
      m_outputStream << flush;
    m_readingCount++;
    return processed;
  }
  

   bool GridSlamProcessor::initScan(const RangeReading & reading, int adaptParticles){
    
    /**retireve the position from the reading, and compute the odometry*/
    OrientedPoint relPose=reading.getPose();
    if (!m_count){
      m_lastPartPose=m_odoPose=relPose;
    }
    
    for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
      //it->pose =relPose;
      OrientedPoint& pose(it->pose);
      pose=m_motionModel.drawFromMotionNew(it->pose, relPose, m_odoPose);
    }

    //invoke the callback
    onOdometryUpdate();

    // accumulate the robot translation and rotation
    OrientedPoint move=relPose-m_odoPose;
    move.theta=atan2(sin(move.theta), cos(move.theta));
    m_linearDistance+=sqrt(move*move);
    m_angularDistance+=fabs(move.theta);
    
    m_odoPose=relPose;
    
    bool processed=false;
    return processed;

    xlog.Debug("m_count:%d, m_linearDistance:%f, m_linearThresholdDistance:%f, m_angularDistance:%f, m_angularThresholdDistance:%f, reading.getTime():%f, last_update_time_:%f, period_:%.4f, reading.st:%f",\
                    m_count, m_linearDistance, m_linearThresholdDistance, m_angularDistance, m_angularThresholdDistance,\
                      reading.getTime(), last_update_time_, period_, reading.getTime());

    // process a scan only if the robot has traveled a given distance or a certain amount of time has elapsed
    {
      //xlog.Info("update condition");
      last_update_time_ = reading.getTime();      

      if (m_outputStream.is_open())
      {
        m_outputStream << setiosflags(ios::fixed) << setprecision(6);
        m_outputStream << "FRAME " <<  m_readingCount;
        m_outputStream << " " << m_linearDistance;
        m_outputStream << " " << m_angularDistance << endl;
      }
      
      if (m_infoStream)
	        m_infoStream << "update frame " <<  m_readingCount << endl
		     << "update ld=" << m_linearDistance << " ad=" << m_angularDistance << endl;
      
      xlog.Debug("Laser Pose= %.3f %.3f %.3f", 
          reading.getPose().x, reading.getPose().y, reading.getPose().theta);
      
      
      //this is for converting the reading in a scan-matcher feedable form
      assert(reading.size()==m_beams);
      double * plainReading = new double[m_beams];
      int tmpCnt=0;
      for(unsigned int i=0; i<m_beams; i++)
      {
	      plainReading[i]=reading[i];
        if(reading[i]<0.2)
           tmpCnt++;
      }
      //printf("tmpCnt is %d  %d ",tmpCnt,m_beams);
      m_infoStream << "m_count " << m_count << endl;

      RangeReading* reading_copy = new RangeReading(reading.size(),
                               &(reading[0]),
                               static_cast<const RangeSensor*>(reading.getSensor()),
                               reading.getTime());
  
        scanMatch(plainReading);     
	      onScanmatchUpdate();
	      updateTreeWeights(false);	
      
        //xlog.Info("before resample");g
		
 	      resample(plainReading, adaptParticles, reading_copy);
	
      //		cerr  << "Tree: normalizing, resetting and propagating weights at the end..." ;
      updateTreeWeights(false);
      //		cerr  << ".done!" <<endl;   
      delete [] plainReading;
      m_lastPartPose=m_odoPose; //update the past pose for the next iteration
      m_linearDistance=0;
      m_angularDistance=0;
      m_count++;
      processed=true;
      
      //keep ready for the next step
      for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++)
      {
	      it->previousPose=it->pose;
        printf("particle pose is  %f  %f  %f \n",it->pose.x,it->pose.y,it->pose.theta);
      }
      
    }
    if (m_outputStream.is_open())
      m_outputStream << flush;
    m_readingCount++;
    return processed;
  }
  
  
  std::ofstream& GridSlamProcessor::outputStream(){
    return m_outputStream;
  }
  
  std::ostream& GridSlamProcessor::infoStream(){
    return m_infoStream;
  }
  
  
  int GridSlamProcessor::getBestParticleIndex() const{
    unsigned int bi=0;
    double bw=-std::numeric_limits<double>::max();
    for (unsigned int i=0; i<m_particles.size(); i++)
      if (bw<m_particles[i].weightSum){
	bw=m_particles[i].weightSum;
	bi=i;
      }
    return (int) bi;
  }

  // void GridSlamProcessor::updateParticles(float angle)
  // {
  //   float r1 = cos(angle);
  //   float r2 = sin(angle);

  //   // write the state of the reading and update all the particles using the motion model
  //   for (ParticleVector::iterator it = m_particles.begin(); it != m_particles.end(); it++)
  //   {
  //     OrientedPoint &pose(it->pose);

  //     float y0 = pose.y;
  //     float x0 = pose.x;
  //     it->pose.x = (x0 * r1 - y0 * r2);
  //     it->pose.y = (x0 * r2 + y0 * r1);
  //     it->pose.theta += angle;   

  //     printf("particile pose is   %f %f %f \n",it->pose.x,it->pose.y,it->pose.theta);

  //     GMapping::GridSlamProcessor::TNode *n = it->node;

  //     while (n){
			
  //     if (!n->reading)
  //     {
  //         printf("no reading \n");
  //         n=n->parent;
  //         continue;
  //     }
  //     n->pose.theta = n->pose.theta +angle;
  //     float y0 =  n->pose.y;
  //     float x0 =  n->pose.x;
  //     n->pose.x= (x0 * r1 - y0 * r2 );
  //     n->pose.y= (x0 * r2 + y0 * r1 );

  //     printf("trai pose is   %f %f %f \n",n->pose.x,n->pose.y,n->pose.theta);

  //     n=n->parent;
	// 	}
      
  //     /*for (GMapping::GridSlamProcessor::TNode *n = it->node;
  //        n;
  //        n = n->parent)
  //       {
  //           if (!n->reading)
  //           {
  //               continue;
  //           }
  //         n->pose.theta = n->pose.theta +angle;
  //         float y0 =  n->pose.y;
  //         float x0 =  n->pose.x;
  //         n->pose.x= (x0 * r1 - y0 * r2 );
  //         n->pose.y= (x0 * r2 + y0 * r1 );

  //         printf("trai pose is   %f %f %f \n",n->pose.x,n->pose.y,n->pose.theta);
  //       }*/
  //   }

  //   float y0 =  m_odoPose.y;
  //   float x0 =  m_odoPose.x;
  //   m_odoPose.x= (x0 * r1 - y0 * r2 );
  //   m_odoPose.y= (x0 * r2 + y0 * r1 );
  //   m_odoPose.theta += angle;        
  // }

bool GridSlamProcessor::rotateNode(TNode* n, float angle)
{
    if (!n||!n->reading)
    {
      return true;
    }
     
    float r1 = cos(angle);
    float r2 = sin(angle);

    //printf("pre trai pose is   %f %f %f \n",n->pose.x,n->pose.y,n->pose.theta);
    float z= n->pose.theta;
    n->pose.theta = z +angle;
    float y0 =  n->pose.y;
    float x0 =  n->pose.x;
    n->pose.x= (x0 * r1 - y0 * r2 );
    n->pose.y= (x0 * r2 + y0 * r1 );

    printf("trai pose is  %f %f %f,  %f %f %f \n",x0,y0,z,n->pose.x,n->pose.y,n->pose.theta);

    rotateNode(n->parent,angle);

    return false;
}

void GridSlamProcessor::updateParticles(float angle)
{
  float r1 = cos(angle);
  float r2 = sin(angle);

  // write the state of the reading and update all the particles using the motion model
  int nodeIndex= 0;
  for (ParticleVector::iterator it = m_particles.begin(); it != m_particles.end(); it++)
  {
    OrientedPoint &pose(it->pose);

    float y0 = pose.y;
    float x0 = pose.x;
    it->pose.x = (x0 * r1 - y0 * r2);
    it->pose.y = (x0 * r2 + y0 * r1);
    it->pose.theta += angle;   

    printf("particle  pose is  %f %f %f \n",it->pose.x,it->pose.y,it->pose.theta);
    for (GMapping::GridSlamProcessor::TNode *n = it->node;n;n = n->parent)
    {
      if (!n->reading||n->rotateFlag)
      {
          continue;
      }
      n->pose.theta = n->pose.theta +angle;
      float y0 =  n->pose.y;
      float x0 =  n->pose.x;
      n->pose.x= (x0 * r1 - y0 * r2 );
      n->pose.y= (x0 * r2 + y0 * r1 );

      n->rotateFlag=1;

      //printf(" %d trai pose is   %f %f %f \n",nodeIndex,n->pose.x,n->pose.y,n->pose.theta);
    }

    if (abs(angle)>0.1)
    {
      for (size_t x = 0; x < it->map.getMapSizeX(); x++)
      {
        for (size_t y = 0; y < it->map.getMapSizeY(); y++)
        {
          GMapping::IntPoint p(x, y);
          double occ = it->map.cell(p);
          //if (occ >= 0)
          {
            PointAccumulator& cell=it->map.cell(p);
            double e=-cell.entropy();
            cell.clear(false, Point(0,0));
          }
        }    
      } 
    }

    for (GMapping::GridSlamProcessor::TNode *n = it->node;n;n = n->parent)
    {
      if (!n->reading)
      {
          continue;
      }
       
      m_matcher.invalidateActiveArea();
      m_matcher.computeActiveArea(it->map, n->pose, &((*n->reading)[0]));
      m_matcher.registerScan(it->map, n->pose, &((*n->reading)[0]));
    } 

    nodeIndex++;
  }

  float y0 =  m_odoPose.y;
  float x0 =  m_odoPose.x;
  m_odoPose.x= (x0 * r1 - y0 * r2 );
  m_odoPose.y= (x0 * r2 + y0 * r1 );
  m_odoPose.theta += angle;       
}



  void GridSlamProcessor::updateParticles(OrientedPoint initialPose)
  {
    // write the state of the reading and update all the particles using the motion model
    for (ParticleVector::iterator it = m_particles.begin(); it != m_particles.end(); it++)
    {
      it->pose.x = initialPose.x;
      it->pose.y = initialPose.y;
      it->pose.theta = initialPose.theta;     
    }
  }

  void GridSlamProcessor::updateParticleScore(float score)
  {
    for (ParticleVector::iterator it = m_particles.begin(); it != m_particles.end(); it++)
    {
      it->amclScore= score;  
    }
  }
  


  void GridSlamProcessor::onScanmatchUpdate(){}
  void GridSlamProcessor::onResampleUpdate(){}
  void GridSlamProcessor::onOdometryUpdate(){}

  
};// end namespace




