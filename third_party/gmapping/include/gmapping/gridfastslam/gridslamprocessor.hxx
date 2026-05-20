
#ifdef MACOSX
// This is to overcome a possible bug in Apple's GCC.
#define isnan(x) (x==FP_NAN)
#endif


inline void GridSlamProcessor::parallel(ScanMatcher &matcher, const double* plainReading, ParticleVector::iterator it, int ParticlesNum, double &sumScore)
{
  //double sumScore=0;
  for (int i=0; i<ParticlesNum; i++)
  {
    OrientedPoint corrected;
    double score, l, s;
    score=matcher.optimize(corrected, it->map, it->pose, plainReading);
    //    it->pose=corrected;
    //xlog.Info("score:%.8lf, minimumscore:%.8lf", score, m_minimumScore);
    if (score>m_minimumScore )
    {      //test gmapping
      it->pose=corrected;
    } 
    else {
      if (m_infoStream){
        //m_infoStream << "lp:" << m_lastPartPose.x << " "  << m_lastPartPose.y << " "<< m_lastPartPose.theta <<std::endl;
        //m_infoStream << "op:" << m_odoPose.x << " " << m_odoPose.y << " "<< m_odoPose.theta <<std::endl;
      }
    }

    matcher.likelihoodAndScore(s, l, it->map, it->pose, plainReading);
    sumScore+=score;
    it->weight+=l;
    it->weightSum+=l;

    //set up the selective copy of the active area
    //by detaching the areas that will be updated
    matcher.invalidateActiveArea();
    matcher.computeActiveArea(it->map, it->pose, plainReading);
    it++;
  }
}


/**Just scan match every single particle.
If the scan matching fails, the particle gets a default likelihood.*/

inline void GridSlamProcessor::scanMatch(const double* plainReading){
#if 0
#define THREAD_NUM 2
  //int threadNum = 2;
  int particlesNum = m_particles.size();
  ParticleVector::iterator it=m_particles.begin(); 
  std::thread threads[THREAD_NUM];
  double sumScore=0;
  double sumScores[THREAD_NUM]={0};
  for (int i = 0; i < THREAD_NUM; i++)
  {
    threads[i] = std::thread(&GridSlamProcessor::parallel, this, std::ref(m_matcher), plainReading, it+particlesNum/THREAD_NUM * i, particlesNum/THREAD_NUM, std::ref(*(sumScores+i)));
    sumScore += sumScores[i];
  }
  
  for (auto & th:threads)
  {
    if(th.joinable())
      th.join();
    
  }
        

  if (m_infoStream)
    m_infoStream << "Average Scan Matching Score=" << (sumScore)/m_particles.size() << std::endl;	
#else
  double sumScore=0;
  // float mean;
  float dev;
  float sum_weight;
  std::vector<double> all_particle_weights;
  float bestScore=0;
  for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++)
  {
    OrientedPoint corrected;
    // OrientedPoint prePose= it->pose;
    double score, l, s;
    uint64_t startF_us, startF_us1,endF_us;  
		startF_us = getTimeStap_us();
    score=m_matcher.optimize(corrected, it->map, it->pose, plainReading);
    //double score1 = m_matcher.scoreBest(it->map, it->pose, plainReading);
    //double score2 = m_matcher.scoreBest(it->map, corrected, plainReading);
    //    it->pose=corrected;
    //xlog.Info("score:%.8lf, minimumscore:%.8lf", score, m_minimumScore);
    //printf("corrected is %lf  %lf  %lf  %lf ",corrected.x,corrected.y,it->pose.x.it->pose.y);
   // xlog.Info("corrected is %lf %lf  %lf - %lf  %lf %lf , %lf  %lf %lf",score,score1,m_minimumScore,corrected.x,corrected.y,corrected.theta,it->pose.x,it->pose.y,it->pose.theta);
    //printf("corrected is %lf %lf %lf   %lf  %lf   %lf  %lf ",score,score1,score2,corrected.x,corrected.y,it->pose.x,it->pose.y);
   
    float dist =sqrt((corrected.x-it->pose.x)*(corrected.x-it->pose.x)+(corrected.y-it->pose.y)*(corrected.y-it->pose.y));
    
    if (score>m_minimumScore)
    {      //test gmapping
      it->pose=corrected;
      //xlog.Info("score:%.8lf, minimumscore:%.8lf", score, m_minimumScore);
    } 
    else {
      if (m_infoStream){ 
        //m_infoStream << "lp:" << m_lastPartPose.x << " "  << m_lastPartPose.y << " "<< m_lastPartPose.theta <<std::endl;
        //m_infoStream << "op:" << m_odoPose.x << " " << m_odoPose.y << " "<< m_odoPose.theta <<std::endl;
      }
    }

    if (dist>0.25) 
    {
      
      /* xlog.Error("err is %f corrected is %lf - %lf  %lf   %lf  %lf ",dist,score,corrected.x,corrected.y,it->pose.x,it->pose.y);
      score=m_matcher.optimize1(corrected, it->map, prePose, plainReading);
      xlog.Info("corrected is %lf - %lf  %lf ",score,corrected.x,corrected.y);

      //xlog.Error("////////////////////////err is %f ///////////////////// ",dist);
       cv::Mat original_img = cv::Mat(cv::Size(it->map.getMapSizeX(), it->map.getMapSizeY()), CV_8UC1, cv::Scalar(0)); 
      for (int x = 0; x < it->map.getMapSizeX(); x++)
      {
          for (int y = 0; y < it->map.getMapSizeY(); y++)
          {
              /// @todo Sort out the unknown vs. free vs. obstacle thresholding
              GMapping::IntPoint p(x, y);
              double occ = it->map.cell(p);
              assert(occ <= 1.0);

              if (occ > 0.1)
              {
                  std::cout<<  x <<"  "<<y<<std::endl; 
                  original_img.at<uchar>(y,x)= 100;
              }
          }
      } 
      cv::imwrite("original_img.jpg",original_img);
      cv::Mat original_img1 =original_img.clone();
      int i=0;
      int hitCnt=0;
      double s1=0;
      double s2=0;
      for (const double* r=plainReading; r<plainReading+400; r++)
      {
        i++;
        double d=*r;
        if (d>3.8)
          continue;
       
        float  angle = -M_PI/2+M_PI*i/400;
        Point phit=it->pose;

        float costh= cos(it->pose.theta+angle);
        float sinth= sin(it->pose.theta+angle);
        phit.x+=*r*costh;
        phit.y+=*r*sinth;
        IntPoint iphit=it->map.world2map(phit);
        original_img1.at<uchar>(iphit.y,iphit.x)= 150;

        Point bestMu(0.,0.);
        bool found =false;
			 	const PointAccumulator& cell=it->map.cell(iphit);
        //printf("%d  %d  %d  %d  %f \n",i,hitCnt,iphit.x,iphit.y,(double)cell);
        //printf(" %d  %d  %f \n",iphit.x,iphit.y,(double)cell);
				if (((double)cell )> 0.1){
				//if (((double)cell )> m_fullnessThreshold){
					Point mu=phit-cell.mean();
          //printf("mu is %f  %f",mu.x,mu.y);

					bestMu=mu;
					found=true;
				} 
		
        if (found)
        {
          s1+=exp(-1./0.075*bestMu*bestMu);
          hitCnt++;
      //		printf(" %d: %d %d ",hitCnt,iphit.x,iphit.y);
        }

  
      }

      //printf("1111111111111111111111\n");


      cv::Mat original_img2 =original_img.clone();
      i=0;
      int hitCnt1=0;
      for (const double* r=plainReading; r<plainReading+400; r++)
      {
        i++;
        double d=*r;
        if (d>3.8)
          continue;
       
        float  angle = -M_PI/2+M_PI*i/400;
        Point phit=corrected;

        float costh= cos(corrected.theta+angle);
        float sinth= sin(corrected.theta+angle);
        phit.x+=*r*costh;
        phit.y+=*r*sinth;
        IntPoint iphit=it->map.world2map(phit);
        original_img2.at<uchar>(iphit.y,iphit.x)= 200;

        Point bestMu(0.,0.);
        bool found =false;
        const PointAccumulator& cell=it->map.cell(iphit);
        //printf("%d  %d  %d  %d  %f \n",i,hitCnt1,iphit.x,iphit.y,(double)cell);
        printf(" %d  %d  %f \n",iphit.x,iphit.y,(double)cell);
				if (((double)cell )> 0.1){
				//if (((double)cell )> m_fullnessThreshold){
					Point mu=phit-cell.mean();


					bestMu=mu;
					found=true;
				} 
		
        if (found)
        {
          s2+=exp(-1./0.075*bestMu*bestMu);
          hitCnt1++;
      //		printf(" %d: %d %d ",hitCnt,iphit.x,iphit.y);
        }
  
      }

      printf(" %d %d %f  %f\n",hitCnt,hitCnt1,s1,s2);
      
      cv::imwrite("original_img1.jpg",original_img1);
      cv::imwrite("original_img2.jpg",original_img2);

      exit(0);  */ 
    }

    if (score>bestScore)
    {
      bestScore = score;
    }

    startF_us1 = getTimeStap_us();

    m_matcher.likelihoodAndScore1(s, l, it->map, it->pose, plainReading);

    //double l1;
    //m_matcher.likelihoodAndScore(s, l1, it->map, it->pose, plainReading);

    endF_us = getTimeStap_us();
    // uint64_t diff1 = startF_us1 -startF_us;
    // uint64_t diff2 = endF_us -startF_us1;

    //printf(" %lf  %lf\n",l,l1);

    sumScore+=score;
    it->weight+=l;
    it->weightSum+=l;
    all_particle_weights.push_back(l);
    sum_weight+=l;

    //set up the selective copy of the active area
    //by detaching the areas that will be updated
    m_matcher.invalidateActiveArea();
    m_matcher.computeActiveArea(it->map, it->pose, plainReading);
  }
  
  for (size_t i = 0; i < all_particle_weights.size(); i++)
  {
    all_particle_weights[i]=all_particle_weights[i]/sum_weight;
  }
  float aver_weight=1/m_particles.size();
  for (size_t i = 0; i < all_particle_weights.size(); i++)
  {
    dev+=(all_particle_weights[i]-aver_weight)*(all_particle_weights[i]-aver_weight);
  }
  xlog.Info("weight is %f  dev is %f  aver score is %f  bestScore is %f\n",sum_weight,dev,sumScore/m_particles.size(),bestScore);
  

  if (m_infoStream)
    m_infoStream << "Average Scan Matching Score=" << sumScore/m_particles.size() << std::endl;	
#endif

}

/*
inline void GridSlamProcessor::scanMatch(const double* plainReading){
  double sumScore=0;
  for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
    OrientedPoint corrected;
    double score, l, s;
    score=m_matcher.optimize(corrected, it->map, it->pose, plainReading);
    //    it->pose=corrected;
    //xlog.Info("score:%.8lf, minimumscore:%.8lf", score, m_minimumScore);
    if (score>m_minimumScore){      //test gmapping
      it->pose=corrected;
    } else {
	if (m_infoStream){
	  //m_infoStream << "lp:" << m_lastPartPose.x << " "  << m_lastPartPose.y << " "<< m_lastPartPose.theta <<std::endl;
	  //m_infoStream << "op:" << m_odoPose.x << " " << m_odoPose.y << " "<< m_odoPose.theta <<std::endl;
	}
    }

    m_matcher.likelihoodAndScore(s, l, it->map, it->pose, plainReading);
    sumScore+=score;
    it->weight+=l;
    it->weightSum+=l;

    //set up the selective copy of the active area
    //by detaching the areas that will be updated
    m_matcher.invalidateActiveArea();
    m_matcher.computeActiveArea(it->map, it->pose, plainReading);
  }
  if (m_infoStream)
    m_infoStream << "Average Scan Matching Score=" << sumScore/m_particles.size() << std::endl;	
}
*/

inline void GridSlamProcessor::normalize(){
  //normalize the log m_weights
  double gain=1./(m_obsSigmaGain*m_particles.size());
  double lmax= -std::numeric_limits<double>::max();
  for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
    lmax=it->weight>lmax?it->weight:lmax;
  }
  //cout << "!!!!!!!!!!! maxwaight= "<< lmax << endl;
  
  m_weights.clear();
  double wcum=0;
  m_neff=0;
  for (std::vector<Particle>::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
    m_weights.push_back(exp(gain*(it->weight-lmax)));
    wcum+=m_weights.back();
    //cout << "l=" << it->weight<< endl;
  }
  
  m_neff=0;
  for (std::vector<double>::iterator it=m_weights.begin(); it!=m_weights.end(); it++){
    *it=*it/wcum;
    double w=*it;
    m_neff+=w*w;
  }
  m_neff=1./m_neff;
  
}

inline bool GridSlamProcessor::resample(const double* plainReading, int adaptSize, const RangeReading* reading){
  
  bool hasResampled = false;
  
  TNodeVector oldGeneration;
  for (unsigned int i=0; i<m_particles.size(); i++){
    oldGeneration.push_back(m_particles[i].node);
  }
  
  if (m_neff<m_resampleThreshold*m_particles.size()){		
    
    if (m_infoStream)
      m_infoStream  << "*************RESAMPLE***************" << std::endl;
    
    uniform_resampler<double, double> resampler;
    m_indexes=resampler.resampleIndexes(m_weights, adaptSize);
    
    if (m_outputStream.is_open()){
      m_outputStream << "RESAMPLE "<< m_indexes.size() << " ";
      for (std::vector<unsigned int>::const_iterator it=m_indexes.begin(); it!=m_indexes.end(); it++){
	m_outputStream << *it <<  " ";
      }
      m_outputStream << std::endl;
    }
    
    onResampleUpdate();
    //BEGIN: BUILDING TREE
    ParticleVector temp;
    unsigned int j=0;
    std::vector<unsigned int> deletedParticles;  		//this is for deleteing the particles which have been resampled away.
    //		cerr << "Existing Nodes:" ;
    for (unsigned int i=0; i<m_indexes.size(); i++){
      //			cerr << " " << m_indexes[i];
      while(j<m_indexes[i]){
	deletedParticles.push_back(j);
	j++;
			}
      if (j==m_indexes[i])
	j++;
      Particle & p=m_particles[m_indexes[i]];
      TNode* node=0;
      TNode* oldNode=oldGeneration[m_indexes[i]];
      //			cerr << i << "->" << m_indexes[i] << "B("<<oldNode->childs <<") ";
      node=new	TNode(p.pose, 0, oldNode, 0);
      //node->reading=0;
      node->reading=reading;
      //			cerr << "A("<<node->parent->childs <<") " <<endl;
      
      temp.push_back(p);
      temp.back().node=node;
      temp.back().previousIndex=m_indexes[i];
    }
    while(j<m_indexes.size()){
      deletedParticles.push_back(j);
      j++;
    }
    //		cerr << endl;
    //xlog.Error("Deleting Nodes:");
    for (unsigned int i=0; i<deletedParticles.size(); i++){
      //xlog.WriteRaw(" %d", deletedParticles[i]);
      delete m_particles[deletedParticles[i]].node;
      m_particles[deletedParticles[i]].node=0;
    }
    
    xlog.WriteRaw("\nDone");
    xlog.Flush();
    //END: BUILDING TREE
    xlog.Error("Deleting old particles...") ;
    m_particles.clear();
    xlog.Error("Done");
    xlog.Error("Copying Particles and  Registering  scans...");
    for (ParticleVector::iterator it=temp.begin(); it!=temp.end(); it++){
      it->setWeight(0);
      m_matcher.invalidateActiveArea();
      m_matcher.registerScan(it->map, it->pose, plainReading);
      m_particles.push_back(*it);
    }
    xlog.Error("Done");
    hasResampled = true;
  } else {
    int index=0;
    xlog.Error("Registering Scans:");
    TNodeVector::iterator node_it=oldGeneration.begin();
    for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
      //create a new node in the particle tree and add it to the old tree
      //BEGIN: BUILDING TREE  
      TNode* node=0;
      node=new TNode(it->pose, 0.0, *node_it, 0);
      
      //node->reading=0;
      node->reading=reading;
      it->node=node;

      //END: BUILDING TREE
      m_matcher.invalidateActiveArea();
      m_matcher.registerScan(it->map, it->pose, plainReading);
      it->previousIndex=index;
      index++;
      node_it++;
      
    }
    xlog.Error("Done");
    
  }
  //END: BUILDING TREE
  return hasResampled;
}
