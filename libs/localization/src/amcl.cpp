/*
 * @Description: 
 * @version: 
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2020-08-19 20:56:56
 * @LastEditors: Jephy Zhang
 * @LastEditTime: 2022-02-09 16:08:57
 */

#include "amcl.h"
#include "global.h"
#include <algorithm>
#include "NavUtils/NavUtils.h"

#include "iomanip"



//#define TESTAMCL

std::vector<std::pair<int,int> > Amcl_t::free_space_indices;

static double normalize(double z)
{
  return atan2(sin(z),cos(z));
}
static double angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

void Amcl_t::pubAmclRet()
{
    odomSub = lcm_p.subscribe(LCM_CHANNEL_Odom, &Amcl_t::odomMsgUpdate, this);
    odomSub->setQueueCapacity(1);

    bindCpuCore(BIND_CPU_ID_AMCL);
    xlog->Info("publish amcl pose and odom!\r\n");
    while(isRunning)
    {
        lcmHandle(&lcm_p);
        pubAmclPose();
        sleep_us(20000);
    }
}

void Amcl_t::lcmHandle(lcm::LCM *lcmPt)
{
    while (1)
    {
        int ret = lcmPt->handleTimeout(1);
        if (ret <= 0)
            break;
    }
}

void Amcl_t::Main()
{
    bindCpuCore(BIND_CPU_ID_AMCL);
    // int tick = 0;

#ifdef TESTAMCL
    unsigned int reqMapTick = 0; //for test localCostmap
    //requestMap();
#endif

    while(isRunning)
    {
		uint64_t start_ts_us = getTimeStap_us();
#ifdef TESTAMCL
        reqMapTick++;
        if(1200 == reqMapTick)
        {
            //testUpdateMapBySelf();
            requestMap();
            //reqMapTick = 0;
        }
#endif    
        if(slamMapTs > curMapTs + 1000000)
        {
            requestMap();
            slamMapTs = curMapTs;
        }
            
        lcmHandle(&lcm_m);
        //pubAmclPose();

		uint64_t end_ts_us = getTimeStap_us();
        xlog->Info("amcl main cost:%.6f s\n", (end_ts_us - start_ts_us) * 0.000001);
        if(20000 > (end_ts_us - start_ts_us))
            sleep_us(20000 - (end_ts_us - start_ts_us));     
    }
}

void Amcl_t::testUpdateMapBySelf()
{
    std::lock_guard<std::recursive_mutex> clk(mtxConfig);
    // Re-initialize the filter
    pf_vector_t pf_init_pose_mean = pf_vector_zero();
    pf_init_pose_mean.v[0] = amclPose.px;
    pf_init_pose_mean.v[1] = amclPose.py;
    pf_init_pose_mean.v[2] = amclPose.pa;
    pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
    // Copy in the covariance, converting from 6-D to 3-D

    pf_init_pose_cov.m[0][0] = 0.001; //msg.cov[0];
    //pf_init_pose_cov.m[0][1] = msg.cov[3];
    //pf_init_pose_cov.m[0][2] = msg.cov[5];
    //pf_init_pose_cov.m[1][0] = msg.cov[3];
    pf_init_pose_cov.m[1][1] = 0.001; //msg.cov[1];
    //pf_init_pose_cov.m[1][2] = msg.cov[4];
    //pf_init_pose_cov.m[2][0] = msg.cov[5];
    //pf_init_pose_cov.m[2][1] = msg.cov[4];
    pf_init_pose_cov.m[2][2] = 0.001; //msg.cov[2];

    delete initial_pose_hyp_;
    initial_pose_hyp_ = new amcl_hyp_t();
    initial_pose_hyp_->pf_pose_mean = pf_init_pose_mean;
    initial_pose_hyp_->pf_pose_cov = pf_init_pose_cov;

    applyInitialPose();
}

void Amcl_t::Start()
{
    lastOdom.timestamp_us = 0;
    lastOdom.px = 0.0;
    lastOdom.py = 0.0;
    lastOdom.pa = 0.0;

    amclPose.px = 0.0;
    amclPose.py = 0.0;
    amclPose.pa = 0.0;

    odomData.px = odomData.py = odomData.pa = 0.0;
    
    setupMsgCb();
    gotInitPose = false;
    isRunning = true;
   
    xlog->Info("Amcl has synchronized timer\r\n");
    mThread = std::thread(&Amcl_t::Main, this);
    pThread = std::thread(&Amcl_t::pubAmclRet, this);  
}

void Amcl_t::Stop()
{
    //saveCurPose();
    first_map_received_ = false;
    isRunning = false;

    lcm_m.unsubscribe(laserSub);
    lcm_m.unsubscribe(initPoseCmdSub);
    lcm_m.unsubscribe(gridmapSub);
    lcm_m.unsubscribe(rotationPoseSub);
    //lcm_m.unsubscribe(odomSub);
    lcm_m.unsubscribe(robotStatusSub);
    lcm_m.unsubscribe(rpcSub);
    lcm_p.unsubscribe(odomSub);

    if(pThread.joinable())
    {
        pThread.join();
    }
    if(mThread.joinable())
        mThread.join();
}

void Amcl_t::resetPf()
{
#if 0
    // Clear queued laser objects so that their parameters get updated
    lasers_.clear();
    lasers_update_.clear();

    if( pf_ != NULL )
    {
        pf_free( pf_ );
        pf_ = NULL;
    }	
    pf_ = pf_alloc(min_particles_, max_particles_,
                    alpha_slow_, alpha_fast_,
                    (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                    (void *)map_);
    pf_set_selective_resampling(pf_, selective_resampling_);
    pf_err_ = config.kld_err; 
    pf_z_ = config.kld_z; 
    pf_->pop_err = pf_err_;
    pf_->pop_z = pf_z_;

    // Initialize the filter
    pf_vector_t pf_init_pose_mean = pf_vector_zero();
    pf_init_pose_mean.v[0] = last_published_pose.pose.pose.position.x;
    pf_init_pose_mean.v[1] = last_published_pose.pose.pose.position.y;
    pf_init_pose_mean.v[2] = tf2::getYaw(last_published_pose.pose.pose.orientation);
    pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
    pf_init_pose_cov.m[0][0] = last_published_pose.pose.covariance[6*0+0];
    pf_init_pose_cov.m[1][1] = last_published_pose.pose.covariance[6*1+1];
    pf_init_pose_cov.m[2][2] = last_published_pose.pose.covariance[6*5+5];
    pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
    pf_init_ = false;

    // Instantiate the sensor objects
    // Odometry
    delete odom_;
    odom_ = new AMCLOdom();
    ROS_ASSERT(odom_);
    odom_->SetModel( odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_ );
    // Laser
    delete laser_;
    laser_ = new AMCLLaser(max_beams_, map_);
    ROS_ASSERT(laser_);
    if(laser_model_type_ == LASER_MODEL_BEAM)
        laser_->SetModelBeam(z_hit_, z_short_, z_max_, z_rand_,
                            sigma_hit_, lambda_short_, 0.0);
    else if(laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD_PROB){
        ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
        laser_->SetModelLikelihoodFieldProb(z_hit_, z_rand_, sigma_hit_,
                        laser_likelihood_max_dist_, 
                        do_beamskip_, beam_skip_distance_, 
                        beam_skip_threshold_, beam_skip_error_threshold_);
        ROS_INFO("Done initializing likelihood field model with probabilities.");
    }
    else if(laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD){
        ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
        laser_->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_,
                                        laser_likelihood_max_dist_);
        ROS_INFO("Done initializing likelihood field model.");
    }
#endif
}

void Amcl_t::freeMapDependentMemory()
{
    if (map_ != NULL)
    {
        map_free(map_);
        map_ = NULL;
    }

    if (!first_map_received_)
    {
        if (pf_ != NULL)
        {
            pf_free(pf_);
            pf_ = NULL;
        }
        delete odom_;
        odom_ = NULL;
    }

    delete laser_;
    laser_ = NULL;
}

map_t* Amcl_t::convertMap( const NavMsg::GridMap_t &msg )
{
    map_t* map = map_alloc();

    map->size_x = msg.width;
    map->size_y = msg.height;
    map->scale = msg.resolution;
    map->origin_x = msg.originPx + (map->size_x / 2) * map->scale;
    map->origin_y = msg.originPy + (map->size_y / 2) * map->scale;

    // Convert to player format
    map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
    int obsCnt=0;
    for(int i=0;i<map->size_x * map->size_y;i++)
    {
        if((msg.data[i]& 7) == 7)  //free
            map->cells[i].occ_state = -1;
        else if((msg.data[i] & 7) == 1)  //obs
        {
            map->cells[i].occ_state = +1;
            obsCnt++;
        }
        else     //unkown
            map->cells[i].occ_state = 0;
    }
    xlog->Info("obsCnt is %d \n",obsCnt);
    xlog->Info("cell  is %d  %d \n",msg.data[7514],msg.data[7515]);
    return map;
}

void Amcl_t::requestMap()
{
    xlog->Info("--------- amcl request for map! -------------------\r\n");
    NavMsg::SlamMapRequest_t mapReq;
    mapReq.name = MapReqAmcl;
    mapReq.requestMap = true;
	mapReq.timestamp_us = getTimeStap_us();
    lcm_m.publish(LCM_CHANNEL_MapRequest, &mapReq);
}

void Amcl_t::gridMapInfoUpdate(const lcm::ReceiveBuffer* rbuf,
		const std::string &channel,
		const NavMsg::GridMapInfo_t *msg)
{
    slamMapTs = msg->updateTsUs;
}


void Amcl_t::gridMapUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const NavMsg::GridMap_t *msg)
{
    xlog->Info("Amcl gridMapUdate mapCaller = %s!\r\n", msg->caller.c_str());
    std::lock_guard<std::mutex> lock(mtx);
    if( first_map_only_ && first_map_received_ ) 
    {
        return;
    }
    xlog->Info("amcl gridmapUpdate! map.width:%d, map.datacount:%d \n", msg->width, msg->data.size());
    if (msg->caller == std::string("Amcl"))
    {
        if (!checkTsError(msg->timestamp_us, 100000))
        {
            //xlog->Info("Warning: Acml Map msg delayed! ts err>0.1 \r\n");
            xlog->Warn("Warning: Acml Map msg delayed! ts err>0.1 ");
        }

        NavMsg::GridMap_t tmpMap = *msg;

        handleMapMessage(tmpMap);
    
        first_map_received_ = true;
        curMapTs = msg->timestamp_us;
        //xlog->Info("mapUpdateTs: %.2f map.width:%d, map.height:%d, map.datacount:%d\n", 
        //   curMapTs, msg->width, msg->height, msg->data.size());
    }
}

void Amcl_t::setUniformDistributionPose(unsigned int particlesNum)
{
    // int i;
    pf_sample_set_t *set;
    pf_sample_t *sample;
    // pf_pdf_gaussian_t *pdf;

    set = pf_->sets + pf_->current_set;

    // Create the kd tree for adaptive sampling
    pf_kdtree_clear(set->kdtree);

    set->sample_count = particlesNum;

    // Compute the new sample poses
    for (int i = 0; i < set->sample_count; i++)
    {
        sample = set->samples + i;
        sample->weight = 1.0 / particlesNum;
        sample->pose = uniformPoseGenerator(map_);
        pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
    }

    pf_->w_slow = pf_->w_fast = 0.0;

    // Re-compute cluster statistics
    pf_cluster_stats(pf_, set);

    //set converged to 0
    pf_init_converged(pf_);
}

void Amcl_t::handleMapMessage(NavMsg::GridMap_t &msg)
{
    std::lock_guard<std::recursive_mutex> lck(mtxConfig);
    freeMapDependentMemory();
    // Clear queued laser objects because they hold pointers to the existing
    // map, #5202.

    lasers_.clear(); //to be completed
    lasers_update_.clear();
    frame_to_laser_.clear();
    map_ = convertMap(msg);

#ifdef NEW_UNIFORM_SAMPLING
    // Index of free space
    //free_space_indices.resize(0);
    free_space_indices.clear();
    for (int i = 0; i < map_->size_x; i++)
        for (int j = 0; j < map_->size_y; j++)
            if (map_->cells[MAP_INDEX(map_, i, j)].occ_state == -1)
                free_space_indices.push_back(std::make_pair(i, j));
#endif

    if (!first_map_received_)
    {
        if (Uniform_Distribution == pdf_type)
        {
            // Create the particle filter
            pf_ = pf_alloc(min_particles_, max_relocalization_particles_,
                           alpha_slow_, alpha_fast_,
                           (pf_init_model_fn_t)Amcl_t::uniformPoseGenerator,
                           (void *)map_);
        }
        else
        {
            // Create the particle filter
            pf_ = pf_alloc(min_particles_, max_particles_,
                           alpha_slow_, alpha_fast_,
                           (pf_init_model_fn_t)Amcl_t::uniformPoseGenerator,
                           (void *)map_);
        }

        pf_set_selective_resampling(pf_, selective_resampling_);
        pf_->pop_err = pf_err_;
        pf_->pop_z = pf_z_;

        // Initialize the filter

        pf_vector_t pf_init_pose_mean = pf_vector_zero();
        if (amclPoseUpdate)
        {
            xlog->Info("handleMapMessage amclPose(%.2f, %.2f, %.2f)\n", amclPose.px, amclPose.py, amclPose.pa);
            pf_init_pose_mean.v[0] = amclPose.px;
            pf_init_pose_mean.v[1] = amclPose.py;
            pf_init_pose_mean.v[2] = amclPose.pa;
        }
        else
        {
            xlog->Info("handleMapMessage odomPose(%.2f, %.2f, %.2f)\n", odomData.px, odomData.py, odomData.pa);
            pf_init_pose_mean.v[0] = odomData.px; 
            pf_init_pose_mean.v[1] = odomData.py;
            pf_init_pose_mean.v[2] = odomData.pa;
        }

        initPose.px = pf_init_pose_mean.v[0];
        initPose.py = pf_init_pose_mean.v[1];
        initPose.pa = pf_init_pose_mean.v[2];

        xlog->Info("initPose is %f  %f  %f \n",initPose.px,initPose.py,initPose.pa);

        pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
        pf_init_pose_cov.m[0][0] = init_cov_[0]; //tmpMsg.cov[0];// init_cov_[0];
        pf_init_pose_cov.m[1][1] = init_cov_[1]; //tmpMsg.cov[1];// init_cov_[1];
        pf_init_pose_cov.m[2][2] = init_cov_[2]; //tmpMsg.cov[2];// init_cov_[2];

        if (Uniform_Distribution == pdf_type)
        {
            xlog->Info("-------uniform  \n ");
            //setUniformDistributionPose(max_relocalization_particles_);
            pdf_type = Gaussion_Distribution;
            //return;
        }
        else
        {
            xlog->Info("-------handleMapMessage \n ");
            pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
            xlog->Info("initPose is %f  %f  %f \n",initPose.px,initPose.py,initPose.pa);
        }

        pf_init_ = false;

        // Instantiate the sensor objects
        // Odometry
        if (NULL != odom_)
        {
            delete odom_;
            odom_ = NULL;
        }

        //xlog->Info("handle map, initPose(%.3f, %.3f, %.3f) \r\n", pf_init_pose_mean.v[0], pf_init_pose_mean.v[1], pf_init_pose_mean.v[2]);
        odom_ = new amcl::AMCLOdom();
        odom_->SetModel(odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_);
    }
    
    // Laser
    if (NULL != laser_)
    {
        delete laser_;
        laser_ = NULL;
    }
    //xlog->Info("new amclLaser \r\n");
    laser_ = new amcl::AMCLLaser(max_beams_, map_);

    if (laser_model_type_ == amcl::laser_model_t::LASER_MODEL_BEAM)
        laser_->SetModelBeam(z_hit_, z_short_, z_max_, z_rand_,
                             sigma_hit_, lambda_short_, 0.0);
    else if (laser_model_type_ == amcl::laser_model_t::LASER_MODEL_LIKELIHOOD_FIELD_PROB)
    {
        //xlog->Info("Initializing field model; this can take some time on large maps...\r\n");
        laser_->SetModelLikelihoodFieldProb(z_hit_, z_rand_, sigma_hit_,
                                            laser_likelihood_max_dist_,
                                            do_beamskip_, beam_skip_distance_,
                                            beam_skip_threshold_, beam_skip_error_threshold_);
        //xlog->Info("Done initializing likelihood field model.\r\n");
    }
    else
    {
        //xlog->Info("Initializing likelihood field model; this can take some time on large maps...\r\n");
        laser_->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_,
                                        laser_likelihood_max_dist_);

        //xlog->Info("Done initializing likelihood field model.\r\n");
    }

    // In case the initial pose message arrived before the first map,
    // try to apply the initial pose now that the map has arrived.
    //xlog->Info("before apply intial pose\r\n");
    if (!first_map_received_)
        applyInitialPose();
    //xlog->Info("handleMapMessage exit\r\n");
}

void Amcl_t::applyInitialPose()
{
    xlog->Info("-----   applyInit \r\n");
    std::lock_guard<std::recursive_mutex> cfl(mtxConfig);
    if( initial_pose_hyp_ != NULL && map_ != NULL ) {
        xlog->Info("-----   1111111111111111   applyInit \r\n");
        pf_init(pf_, initial_pose_hyp_->pf_pose_mean, initial_pose_hyp_->pf_pose_cov);
        pf_init_ = false;

        delete initial_pose_hyp_;
        initial_pose_hyp_ = NULL;
    }
}

int Amcl_t::getValidLaserPoiintCount(SensorMsg::LaserData_t *ldataPt)
{
    int ret = 0;
    for (auto point: ldataPt->ranges)
    {
        if (point > 0.2)
        {
            ret++;
        }
        
    }
    
    return ret;
}

void Amcl_t::laserMsgUpdate(const lcm::ReceiveBuffer *rbuf,
                            const std::string &channel,
                            const SensorMsg::LaserData_t *msg)
{
    std::lock_guard<std::mutex> lock(mtx);
    laser = *msg;

    if ((map_ == NULL) )
    {
        return;
    }

    if(pitchInvaild)
    {
        xlog->Debug("pitch invalid \n");
        return;
    }

    if(laserCorrection)
    {
        for(int i = 0; i < laser.range_num; ++i)
        {
        // laser.bearing[i] = msg->min_bearing + msg->resolution * (i * 3);
        // laser.ranges[i] = msg->ranges[i * 3];
            
            //if(laserCorrection)
            {
                //for test laser left and right data interpolation
                if(laser.ranges[i] > 2.0 && (i > (laser.range_num / 2)))
                {
                    laser.ranges[i] -= log(laser.ranges[i]) / log(28.5);
                }
            }

            //laserData.bearing.push_back(msg->min_bearing + msg->resolution * (i * 3));
            //laserData.ranges.push_back(msg->ranges[i * 3]);
        }
    }
    laserUpdate = true;

    std::lock_guard<std::recursive_mutex> clk(mtxConfig);
    int laser_index = -1;

    // Do we have the base->base_laser Tx yet?
    if (frame_to_laser_.find(msg->name) == frame_to_laser_.end())
    {
        lasers_.push_back(new amcl::AMCLLaser(*laser_));
        lasers_update_.push_back(true);
        laser_index = frame_to_laser_.size();
        xlog->Info("register laser, index:%d\n", laser_index);

        pf_vector_t laser_pose_v;
        laser_pose_v.v[0] = laserMountPose.px;
        laser_pose_v.v[1] = laserMountPose.py;
        // laser mounting angle gets computed later -> set to 0 here!
        laser_pose_v.v[2] = laserMountPose.pa;
        lasers_[laser_index]->SetLaserPose(laser_pose_v);
        xlog->Debug("Received laser's pose wrt robot: %.3f %.3f %.3f",
               laser_pose_v.v[0],
               laser_pose_v.v[1],
               laser_pose_v.v[2]);

        frame_to_laser_[msg->name] = laser_index;
    }
    else
    {
        // we have the laser pose, retrieve laser index
        laser_index = frame_to_laser_[msg->name];
    }

    // Where was the robot when this scan was taken?
    pf_vector_t pose;
    double poseTs;
    uint32_t tsErr_us = 0;
    uint64_t msg_ts_us = (msg->timestamp_us);
    NavMsg::Odom_t odom;
    {
        std::lock_guard<std::mutex> lock(mtxOdom);
        if (odomFilter.empty())
        {
            return;
        }
        odom = odomFilter.lookup(msg_ts_us, tsErr_us);
        if (tsErr_us > 100000)
        {
            xlog->Error("Amcl Error: look up odomPose error, tsErr>0.1");
            return;
        }
        pose.v[0] = odom.px;
        pose.v[1] = odom.py;
        pose.v[2] = odom.pa;
        poseTs = odom.timestamp_us * 0.000001;
    }
    pf_vector_t delta = pf_vector_zero();
    
    if (pf_init_)
    {
        // Compute change in pose
        //delta = pf_vector_coord_sub(pose, pf_odom_pose_);
        delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
        delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
        delta.v[2] = angle_diff(pose.v[2], pf_odom_pose_.v[2]);

        // See if we should update the filter
        bool update = fabs(delta.v[0]) > d_thresh_ ||
                      fabs(delta.v[1]) > d_thresh_ ||
                      fabs(delta.v[2]) > a_thresh_;
        update = update || m_force_update;
        m_force_update = false;

        // Set the laser update flags

        //outfile1 << poseTs <<","<< pose.v[0]<<"," << pose.v[1]<<"," << pose.v[2]<<"," << curMapTs<<"," <<  msg_ts_us/1000000.0<<"," << laser.ranges[600]<<"," << std::endl;
        if (update)
        {

            // xlog->Info("laser odom ts diff:%f \n",tsErr_us*0.000001);
            // xlog->Info("last odom ts:%f,current odom ts:%f,ts diff:%f \n",pf_odom_pose_ts_,poseTs,poseTs-pf_odom_pose_ts_);
            // xlog->Info("before odom update \n pf_odom_pose_  x:%.4f,y:%.4f,theta:%.4f \n odom pose       x:%.4f,y:%.4f,theta:%.4f \n odom delta pose x:%.4f,y:%.4f,theta:%.4f \n",
            //         pf_odom_pose_.v[0],pf_odom_pose_.v[1],pf_odom_pose_.v[2],pose.v[0],pose.v[1],pose.v[2],delta.v[0],delta.v[1],delta.v[2]);
            for (size_t i = 0; i < lasers_update_.size(); i++)
            {
                lasers_update_[i] = true;
            }   
            if(fabs(delta.v[0]) > 0.3 || fabs(delta.v[1]) > 0.3 || fabs(delta.v[2]) > 1.0)
            {
                xlog->Info("odom diff change too fast!skip current data! \n");
                xlog->Info("laser odom ts diff:%f \n",tsErr_us*0.000001);
                xlog->Info("last odom ts:%f,current odom ts:%f,ts diff:%f \n",pf_odom_pose_ts_,poseTs,poseTs-pf_odom_pose_ts_);
                xlog->Info("before odom update \n pf_odom_pose_  x:%.4f,y:%.4f,theta:%.4f \n odom pose       x:%.4f,y:%.4f,theta:%.4f \n odom delta pose x:%.4f,y:%.4f,theta:%.4f \n",
                        pf_odom_pose_.v[0],pf_odom_pose_.v[1],pf_odom_pose_.v[2],pose.v[0],pose.v[1],pose.v[2],delta.v[0],delta.v[1],delta.v[2]);
                xlog->Debug("before odom update \n pf_odom_pose_  x:%.4f,y:%.4f,theta:%.4f \n odom pose       x:%.4f,y:%.4f,theta:%.4f \n odom delta pose x:%.4f,y:%.4f,theta:%.4f \n",
                        pf_odom_pose_.v[0],pf_odom_pose_.v[1],pf_odom_pose_.v[2],pose.v[0],pose.v[1],pose.v[2],delta.v[0],delta.v[1],delta.v[2]);                        
                //exit(0);
                // for (unsigned int i = 0; i < lasers_update_.size(); i++)
                // {
                //     lasers_update_[i] = false;
                // }
                // pf_odom_pose_ = pose;
                // pf_odom_pose_ts_ = poseTs;                
            }            
        }
        else
        {
            for (size_t i = 0; i < lasers_update_.size(); i++)
            {
                lasers_update_[i] = false;
            }
            // xlog->Info("skip current laser! \n");
        }
    }

    bool force_publication = false;
    if (!pf_init_)
    {
        // Pose at last filter update
        pf_odom_pose_.v[0] = initPose.px;
        pf_odom_pose_.v[1] = initPose.py;
        pf_odom_pose_.v[2] = initPose.pa;
        pf_odom_pose_ts_ = initPoseTs;
        //xlog->Info("initPose(%.3f, %.3f, %.3f)\n", pf_odom_pose_.v[0], pf_odom_pose_.v[1], pf_odom_pose_.v[2]);

        // Filter is now initialized
        pf_init_ = true;

        // Should update sensor data
        for (size_t i = 0; i < lasers_update_.size(); i++)
        {
            lasers_update_[i] = true;
        }
            
        force_publication = true;

        resample_count_ = 0;
    }
    // If the robot has moved, update the filter
    else if (pf_init_ && lasers_update_[laser_index])
    {
        amcl::AMCLOdomData odata;
        odata.pose = pose;
        // HACK
        // Modify the delta in the action data so the filter gets
        // updated correctly
        odata.delta = delta;

        // Use the action data to update the filter
        odom_->UpdateAction(pf_, (amcl::AMCLSensorData *)&odata);
    }

    bool resampled = false;
    // If the robot has moved, update the filter   
    if (lasers_update_[laser_index])
    {
        amcl::AMCLLaserData ldata;
        ldata.sensor = lasers_[laser_index];
        ldata.range_count = laser.range_num;

        // To account for lasers that are mounted upside-down, we determine the
        // min, max, and increment angles of the laser in the base frame.
        //
        // Construct min and max angles of laser, in the base_link frame.

        double angle_min = laser.min_bearing;
        double angle_increment = laser.resolution;

        // wrapping angle to [-pi .. pi]
        angle_increment = fmod(angle_increment + 5 * M_PI, 2 * M_PI) - M_PI;

        // Apply range min/max thresholds, if the user supplied them
        if (laser_max_range_ > 0.0)
            ldata.range_max = std::min((double)laser.max_range, laser_max_range_);
        else
            ldata.range_max = laser.max_range;
        double range_min;
        if (laser_min_range_ > 0.0)
            range_min = std::max((double)laser.min_range, laser_min_range_);
        else
            range_min = laser.min_range;
        // The AMCLLaserData destructor will free this memory
        ldata.ranges = new double[ldata.range_count][2];

        for (int i = 0; i < ldata.range_count; i++)
        {
            // amcl doesn't (yet) have a concept of min range.  So we'll map short
            // readings to max range.
            if (laser.ranges[i] <= range_min)
                ldata.ranges[i][0] = ldata.range_max;
            else
                ldata.ranges[i][0] = laser.ranges[i];
            // Compute bearing
            ldata.ranges[i][1] = angle_min +
                                (i * angle_increment);
        }
        lasers_[laser_index]->UpdateSensor(pf_, (amcl::AMCLSensorData *)&ldata);
        total_score = pf_->total_score;
        laserCount = getValidLaserPoiintCount(&laser);
        lasers_update_[laser_index] = false;

        pf_odom_pose_ = pose;
        pf_odom_pose_ts_ = poseTs;
        // Resample the particles
        //xlog->Info("pf w_fast: %.4f  w_slow: %.4f converged=%d clusterNum=%d\r\n", 
            //pf_->w_fast, pf_->w_slow, pf_->converged, (pf_->current_set+pf_->sets)->cluster_count);
        if ( !(++resample_count_ % resample_interval_) && ( pf_->w_fast < 3))
        {
            pf_update_resample(pf_);
            resampled = true;
        }
        else
        {
            pf_sample_set_t* set=pf_->sets + pf_->current_set;
            pf_sample_t *sample;
            // for (int i = 0; i < set->sample_count; i++)
            // {
            //     sample = set->samples + i;
            //     xlog->Info("sample id:%d,weight:%f,x:%f,y:%f,theta:%f \n",i,sample->weight,sample->pose.v[0],sample->pose.v[1],sample->pose.v[2]);
            // }
        }

        pf_sample_set_t *set = pf_->sets + pf_->current_set;

        // Publish the resulting cloud
        // TODO: set maximum rate for publishing
        NavMsg::PoseArray_t PoseArray;
        NavMsg::Pose_t pose;
        pf_vector_t curr_pose_mean;
        curr_pose_mean.v[0]=0;
        curr_pose_mean.v[1]=0;
        curr_pose_mean.v[2]=0;
        if (!m_force_update)
        {
			PoseArray.timestamp_us = getTimeStap_us();
            PoseArray.name = "poseArray";
            PoseArray.poseNum = set->sample_count;
            PoseArray.poses.resize(set->sample_count);
            float dev=0;
            float mean=0;
            std::vector<float> pose_arrays;
            for (int i = 0; i < set->sample_count; i++)
            {

                pose.px = set->samples[i].pose.v[0];
                pose.py = set->samples[i].pose.v[1];
                pose.pa = set->samples[i].pose.v[2];
                curr_pose_mean.v[0]+=pose.px;
                curr_pose_mean.v[1]+=pose.py;
                curr_pose_mean.v[2]+=pose.pa;

                PoseArray.poses[i] = pose;

                float pos =sqrt(pose.px*pose.px+pose.py*pose.py);
                pose_arrays.push_back(pos);
                mean+= pos;
            }
            mean=mean/set->sample_count;
            curr_pose_mean.v[0]=curr_pose_mean.v[0]/set->sample_count;
            curr_pose_mean.v[1]=curr_pose_mean.v[1]/set->sample_count;
            curr_pose_mean.v[2]=curr_pose_mean.v[2]/set->sample_count;

            for (int i = 0; i < pose_arrays.size(); i++)
            {
                float err= (pose_arrays[i]-mean)*(pose_arrays[i]-mean);
                dev+=err;
            }
            dev=dev/pose_arrays.size();
            //xlog->Info("poseArray size:%d  dev is %f \n", PoseArray.poseNum,dev);
            static int amclGoodCnt=0;
            devGood.clear();
            if (dev<0.005)
            {
                amclGoodCnt++;
            }
            else amclGoodCnt=0;

            if (amclGoodCnt>=10)
            {
                devGood+="devGood";
                if (amclGoodCnt%10==0)
                {
                    RobotMsg::HackCmd_t hackCmd;
                    hackCmd.data = 10;
                    lcm_m.publish("AmclToSlamCmd", &hackCmd);
                    //xlog->Info("send to slam \n");
                } 
                if (amclGoodCnt>=1000)
                {
                    amclGoodCnt=0;
                }        
            }

            // timeval StartTs,endTs;
            // gettimeofday(&StartTs,NULL);
            // xlog->Info("cal hit function \n");
            pf_vector_t hitStartPose;
            hitStartPose.v[0]=amclPose.px;
            hitStartPose.v[1]=amclPose.py;
            hitStartPose.v[2]=ClipAngle(amclPose.pa);
            amcl::AMCLLaserData* hitlaser=&ldata; 
            hitScore=lasers_[laser_index]->laserHitThroughScore((amcl::AMCLLaserData *)hitlaser,hitStartPose);
            xlog->Info("hit score:%f \n",hitScore);
            xlog->Info("do clean:%d,send end clean cmd:%d \n",int(doClean),int(sendEndCmd));
            //xlog->Info("useParamCorrect:%d,usePfCorrect:%d \n",useParamCorrect,usePfCorrect);

            if(!doClean)
            {
                if(useParamCorrect)
                {
                    if(hitScore < 60 && !odomModelHasSet)
                    {
                        if (NULL != odom_)
                        {
                            delete odom_;
                            odom_ = NULL;
                        }
                        odom_ = new amcl::AMCLOdom();
                        odom_->SetModel(odom_model_type_, 0.3,0.3,0.3,0.3, alpha5_);
                        odomModelHasSet = true;
                        xlog->Info("low hitscore! odom model parameters changed! \n");                 
                    }
                    else if(hitScore > 80 && odomModelHasSet)
                    {
                        if (NULL != odom_)
                        {
                            delete odom_;
                            odom_ = NULL;
                        }
                        odom_ = new amcl::AMCLOdom();
                        odom_->SetModel(odom_model_type_, alpha1_,alpha2_,alpha3_,alpha4_ ,alpha5_);
                        odomModelHasSet = false;
                        xlog->Info("high hitscore! odom model parameters recover! \n");  
                    }
                }
                if(usePfCorrect)
                {
                    if(fabs(delta.v[2]) > 0.06)
                    {
                        xlog->Info("skip pf correct when rotate! \n");
                    }
                    else
                    {
                        forwardHitScore = lasers_[laser_index]->fowardHitScore((amcl::AMCLLaserData *)hitlaser,hitStartPose,500,700,2.0);
                        float forwardSigma=0.1;
                        //xlog->Info("forwardPfSet:%d \n\n",int(forwardPfSet));
                        if(fabs(forwardHitScore) > 1.0)
                        {
                            if(!forwardPfSet)
                            {
                                float sigma=forwardSigma*(forwardHitScore*map_->scale*forwardHitScore*map_->scale);
                                float forwardGaussianNoise=pf_ran_gaussian(sigma);
                                xlog->Info("sigma:%f,forwardGaussianNoise:%f \n\n\n",sigma,forwardGaussianNoise);

                                pf_sample_set_t* set = pf_->sets + pf_->current_set;
                                for (int i = 0; i < set->sample_count; i++)
                                {
                                    pf_sample_t* sample = set->samples + i;
                                    float forwardGaussianNoise=pf_ran_gaussian(sigma);
                                    sample->pose.v[0] += forwardGaussianNoise * cos(sample->pose.v[2]);
                                    sample->pose.v[1] += forwardGaussianNoise * sin(sample->pose.v[2]);        
                                }   
                                //forwardPfSet=true;
                            }
                        }
                        // else
                        // {
                        //     forwardPfSet=false;
                        // }
                    }
                }
            }
            else
            {
                if(doCleanChange)
                {
                    if (NULL != odom_)
                    {
                        delete odom_;
                        odom_ = NULL;
                    }
                    odom_ = new amcl::AMCLOdom();
                    odom_->SetModel(odom_model_type_, alpha1_,alpha2_,alpha3_,alpha4_ ,alpha5_);    
                    doCleanChange = false;
                }  
            }


            // float ts=getTimeStap_ms()*0.001;
            //outfile1 <<ts<<","<<hitScore<<","<<forwardHitScore<<std::endl;
            //xlog->Info("amcl pose,x:%f,y:%f,theta:%f \n",hitStartPose.v[0],hitStartPose.v[1],hitStartPose.v[2]);
            //xlog->Info("cur  pose,x:%f,y:%f,theta;%f \n",curr_pose_mean.v[0],curr_pose_mean.v[1],curr_pose_mean.v[2]);
            // gettimeofday(&endTs,NULL);
            // xlog->Info("hit score cost time:%lf \n",(endTs.tv_sec-StartTs.tv_sec)+(endTs.tv_usec-StartTs.tv_usec)*0.000001);

            lcm_m.publish(LCM_CHANNEL_AmclParticleCloud, &PoseArray);
        }
    }

    if (resampled || force_publication)
    {
        // Read out the current hypotheses
        double max_weight = 0.0;
        int max_weight_hyp = -1;
        std::vector<amcl_hyp_t> hyps;
        hyps.resize(pf_->sets[pf_->current_set].cluster_count);
        xlog->Debug("hyps size = %d", hyps.size());

        for (int hyp_count = 0;
             hyp_count < pf_->sets[pf_->current_set].cluster_count; hyp_count++)
        {
            double weight;
            pf_vector_t pose_mean;
            pf_matrix_t pose_cov;
            if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov))
            {
                xlog->Error("Couldn't get stats on cluster %d", hyp_count);
                break;
            }
            xlog->Info("cluster seq:%d,weight:%f,mean x:%f,y:%f,theta:%f \n",
                    hyp_count,weight,pose_mean.v[0],pose_mean.v[1],pose_mean.v[2]);

            hyps[hyp_count].weight = weight;
            hyps[hyp_count].pf_pose_mean = pose_mean;
            hyps[hyp_count].pf_pose_cov = pose_cov;

            if (hyps[hyp_count].weight > max_weight)
            {
                max_weight = hyps[hyp_count].weight;
                max_weight_hyp = hyp_count;
            }
        }
        static bool clusterDiverge = false;
        static int clusterCount = 0;
        if(pf_->sets[pf_->current_set].cluster_count > 2)
        {
            xlog->Info("several cluster! stop! \n");
            clusterDiverge = true;
        }
        if(clusterDiverge == true)
        {
            clusterCount++;
        }
        if(clusterCount > 2)
        {
            xlog->Info("cluster recorded! stop! \n");
            clusterCount=0;            
            //exit(0);
        }

        if (max_weight > 0.0)
        {
            xlog->Debug("max_weight:%.4f Max weight pose: %.3f %.3f %.3f",
                   max_weight,
                   hyps[max_weight_hyp].pf_pose_mean.v[0],
                   hyps[max_weight_hyp].pf_pose_mean.v[1],
                   hyps[max_weight_hyp].pf_pose_mean.v[2]);

            std::lock_guard<std::mutex> lock(mtxAmclPose);
            lastOdom = odom; //for amcl pose interpolation
            amclPoseUpdate = true;

            amcl_hyp_t hyps_ = hyps[max_weight_hyp];
            amclPose.px = hyps_.pf_pose_mean.v[0];
            amclPose.py = hyps_.pf_pose_mean.v[1];
            amclPose.pa = hyps_.pf_pose_mean.v[2];

            pf_sample_set_t* set = pf_->sets + pf_->current_set;
            amclPose.covLen = sizeof(set->cov.m) / sizeof(set->cov.m[0][0]);
            amclPose.cov.resize(amclPose.covLen);

            for(unsigned int i = 0; i < 2; i++)
            {
                for(unsigned int j = 0 ; j < 2; j++)
                {
                    amclPose.cov[3 * i + j] = set->cov.m[i][j];
                }
            }
            amclPose.cov[3 * 2 + 2] = set->cov.m[2][2];

            // (multi-line comment removed to avoid backslash warning)

        }
    }
}

void Amcl_t::pubAmclPose()
{
    //xlog->Info("in pubAmclPose\n");
    if(odomFilter.empty())
    {
        //xlog->Info("odomFilter is enpty or amclPose does not update!\n");
        return;
    }
    NavMsg::Odom_t cur_odom = odomFilter.back();
    if(!amclPoseUpdate)
    {
        std::lock_guard<std::mutex> lock(mtxAmclPose);
        amclOdom.name = "amclOdom";
		amclOdom.timestamp_us = getTimeStap_us();
        amclOdom.px = cur_odom.px;
        amclOdom.py = cur_odom.py;
        amclOdom.pa = cur_odom.pa;
        amclOdom.score = cur_odom.score;
    
        lcm_p.publish(LCM_CHANNEL_AmclOdom, &amclOdom);
        return;
    }

    float tsErr = 0;
    //xlog->Info("originAmclPose(%.2f, %.2f, %.2f) \r\n", amclPose.px, amclPose.py, amclPose.pa);

    std::lock_guard<std::mutex> lock(mtxAmclPose);
    if(cur_odom.timestamp_us - lastOdom.timestamp_us > 20000)
    {      
        float err_pa = cur_odom.pa - lastOdom.pa;
        float err_x = (cur_odom.px - lastOdom.px);
        float err_y = (cur_odom.py - lastOdom.py);
        float err_dis = sqrt(err_x * err_x + err_y * err_y);
        float forwardDir = atan2(err_y, err_x);

        while (err_pa >= M_PI)
            err_pa -= 2 * M_PI;
        while (err_pa < -M_PI)
            err_pa += 2 * M_PI;

        if(fabs(ClipAngle(forwardDir - lastOdom.pa)) > M_PI/2.0)
            err_dis *= -1;
        amclPose.pa += err_pa;
        amclPose.pa = ClipAngle(amclPose.pa);
        amclPose.px = amclPose.px + err_dis * cos(amclPose.pa);
        amclPose.py = amclPose.py + err_dis * sin(amclPose.pa);
        
        lastOdom = cur_odom;
    }
    amclPose.weight = total_score;
    amclPose.name = "amclHyps"; 
	amclPose.timestamp_us = getTimeStap_us();
    //amclPose.covLen = amclPose.cov.size();
    if (amclPoseUpdate)
    {
        //lcm.publish(LCM_CHANNEL_AmclPose, &amclPose);
        lcm_p.publish(LCM_CHANNEL_AmclPose, &amclPose);
    }

    amclOdom.name = "amclOdom";
	amclOdom.timestamp_us = getTimeStap_us();
    float angle=0*3.1415/180;
    float r1 = cos(angle);
    float r2 = sin(angle);
    
    float px= cur_odom.px;
    float py= cur_odom.py;
    float pa= cur_odom.pa;

    amclOdom.px = amclPose.px;
    amclOdom.py = amclPose.py;
    amclOdom.pa = amclPose.pa;

    float x0=amclOdom.px;
    float y0=amclOdom.py;
    amclOdom.px=(x0 * r1 - y0 * r2 );//+initx;
    amclOdom.py=(x0 * r2 + y0 * r1 );//+inity;
    amclOdom.pa+= angle;
    
    if (laserCount!=0)
    {
        amclOdom.score = amclPose.weight/laserCount;
    }
    else amclOdom.score = -1;

    NavMsg::SlamPoseInfo_t amclPoseInfo;
    amclPoseInfo.timestamp_us=getTimeStap_us();
    amclPoseInfo.score= hitScore;
    amclPoseInfo.laserCnt=laserCount;
    amclPoseInfo.ratio=amclOdom.score;
    amclPoseInfo.name=devGood;
    // int devbool=1;
    if(devGood.empty())
    {
        // devbool=0;
    }
    // xlog->Info("pf current set:%d,converged:%d,sample count:%d,slow:%f,fast:%f \n",
    //         pf_->current_set,pf_->sets[pf_->current_set].converged,pf_->sets[pf_->current_set].sample_count,pf_->w_slow,pf_->w_fast);
    // xlog->Info("hit score:%f,laser count:%d,ratio:%f \n",hitScore,laserCount,amclOdom.score);
    xlog->Debug("amclpose, %f, %f, %f,  %f, %f, %f,   %f, %f, %lld", px,py,pa,amclOdom.px, amclOdom.py, amclOdom.pa, amclPose.weight,amclOdom.score,amclOdom.timestamp_us);
    //lcm.publish(LCM_CHANNEL_AmclOdom, &amclOdom);
    lcm_p.publish(LCM_CHANNEL_AmclOdom, &amclOdom);
    //xlog->Info("amclPose.weight:%f\n", amclPose.weight);
    lcm_p.publish(LCM_CHANNEL_AmclPoseInfo, &amclPoseInfo);
    //xlog->Info("amcl pose is %f %d ",amclPoseInfo.score,laserCount);
    /*static float lastTs = getTs();
    xlog->Info("Amcl_t pubAmclPose(%.4f, %.4f, %.4f), tserr:%4f\n", amclOdom.px, amclOdom.py, amclOdom.pa, getTs() - lastTs);
    lastTs = getTs();*/

}

void Amcl_t::odomMsgUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const NavMsg::Odom_t *msg)
{
    std::lock_guard<std::mutex> lock(mtxOdom);
    if(resetOdom)
    {
        // Instantiate the sensor objects
        if (NULL != odom_)
        {
            delete odom_;
            odom_ = NULL;
        }
        odom_ = new amcl::AMCLOdom();
        odom_->SetModel(odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_);
        //xlog->Info("a1:%f,a2:%f,a3:%f,a4:%f,a5:%f \n\n\n\n",alpha1_,alpha2_,alpha3_,alpha4_,alpha5_);
        xlog->Info("recover alpha1 ~~ alpha4 parameters!  \n");
        resetOdom=false;
    }
    //xlog->Info("amcl odomupdate!\n");
    if(!checkTsError(msg->timestamp_us, 100000))
    {
       // xlog->Info("Warning: Acml Odom msg over-delayed! ts err>0.1 \n ");
    }
   
   // odomFilter.push(*msg, msg->timestamp_us);
   // odomData = *msg;

    float angle=rotationPose.pa;
    float r1 = cos(angle);
    float r2 = sin(angle);
    
    NavMsg::Odom_t amclOdom = *msg;
    amclOdom.px = msg->px;
    amclOdom.py = msg->py;
    amclOdom.pa = msg->pa;


    float x0=amclOdom.px;
    float y0=amclOdom.py;
    amclOdom.px=(x0 * r1 - y0 * r2 );//+initx;
    amclOdom.py=(x0 * r2 + y0 * r1 );//+inity;
    amclOdom.pa+= angle;
    amclOdom.pa = ClipAngle(amclOdom.pa);

    if(odomVect.size()<3)
        odomVect.push_back(amclOdom);
    else
    {
        odomVect.erase(odomVect.begin());
        odomVect.push_back(amclOdom);
    }

    NavMsg::Odom_t tmpOdom = amclOdom;

    if(odomVect.size() == 3)
    {
        int maxIndex = 0, minIndex = 0, mediuIndex;
        float maxPa = -10;
        float minPa = 1000;
        for(size_t i=0; i<odomVect.size(); i++)
        {
            if(maxPa < odomVect[i].pa)
            {
                maxPa = odomVect[i].pa;
                maxIndex = i;
            }

            if(minPa > odomVect[i].pa)
            {
                minPa = odomVect[i].pa;
                minIndex = i;
            }
        }

        if(maxIndex == minIndex )
            mediuIndex = minIndex;
        else 
            mediuIndex = 3-minIndex - maxIndex;

        tmpOdom = odomVect[mediuIndex];    
        //xlog->Info("(max, min, medius) = (%d, %d, %d) \n",maxIndex, minIndex,mediuIndex);
    }

    //xlog->Info("odom %f  %f  %f  %lld\n",amclOdom.px,amclOdom.py,amclOdom.pa,amclOdom.timestamp_us);
    odomFilter.push(tmpOdom, msg->timestamp_us); 
    odomData = amclOdom;

    odomUpdate = true;

    if(!pf_init_)
        gotInitPose = true;
}

void Amcl_t::initPoseCmdUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const NavMsg::InitialPoseCmd_t *msg)
{
    xlog->Info("amcl initpose(%.4f, %.4f, %.4f)!\n", msg->pose.px, msg->pose.py, msg->pose.pa);
    if(!checkTsError(msg->timestamp_us, 100000))
    {
        xlog->Info("Warning: Acml initPose msg over-delayed! ts err>0.1 \n");
    }
    tmpMsg = *msg;
    if(!first_map_received_)
    {
        if(Uniform_Distribution == msg->pdfType)
            pdf_type = Uniform_Distribution;
        requestMap();
    }
    handleInitalPoseMessage(tmpMsg);
}

void Amcl_t::rotatePoseCmdUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const NavMsg::Pose_t *msg)
{
    rotationPose=*msg;
    xlog->Info("-- -- ---  --- ratation is %f \n",rotationPose.pa);
    xlog->Debug("-----------ratation --------- ");
}

void Amcl_t::handleInitalPoseMessage(const NavMsg::InitialPoseCmd_t &msg)
{
    std::lock_guard<std::recursive_mutex> clk(mtxConfig);
    

    initPose = msg.pose;
    initPoseTs = msg.timestamp_us * 0.000001;
    // Re-initialize the filter
    pf_vector_t pf_init_pose_mean = pf_vector_zero();
    pf_init_pose_mean.v[0] = initPose.px;
    pf_init_pose_mean.v[1] = initPose.py;
    pf_init_pose_mean.v[2] = initPose.pa;
    xlog->Info("relocalization reset pariticles,set init pose x:%f,y:%f,theta:%f \n",initPose.px,initPose.py,initPose.pa);
    pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
    // Copy in the covariance, converting from 6-D to 3-D

    xlog->Info("covLen:%d, (%.4f, %.4f, %.4f, %.4f, %.4f, %.4f)\n",\
        msg.covLen, msg.cov[0], msg.cov[1], msg.cov[2], msg.cov[3], msg.cov[4], msg.cov[5]);

    if(6 == msg.covLen)
    {
        pf_init_pose_cov.m[0][0] = msg.cov[0];
        //pf_init_pose_cov.m[0][1] = msg.cov[3];
        //pf_init_pose_cov.m[0][2] = msg.cov[5];
        //pf_init_pose_cov.m[1][0] = msg.cov[3];
        pf_init_pose_cov.m[1][1] = msg.cov[1];
        //pf_init_pose_cov.m[1][2] = msg.cov[4];
        //pf_init_pose_cov.m[2][0] = msg.cov[5];
        //pf_init_pose_cov.m[2][1] = msg.cov[4];
        pf_init_pose_cov.m[2][2] = msg.cov[2];
    }

    if (NULL != odom_)
    {
        delete odom_;
        odom_ = NULL;
    }
    odom_ = new amcl::AMCLOdom();
    odom_->SetModel(odom_model_type_, 0.5, 0.6, 0.5, 0.4, alpha5_);
    xlog->Info("odom model parameters has changed! \n");     

    if (Uniform_Distribution == msg.pdfType && NULL != map_ )
    {
        if (pf_ != NULL)
        {
            pf_free(pf_);
            pf_ = NULL;
        }
        pf_ = pf_alloc(min_particles_, max_relocalization_particles_,
                       alpha_slow_, alpha_fast_,
                       (pf_init_model_fn_t)Amcl_t::uniformPoseGenerator,
                       (void *)map_);
        pf_set_selective_resampling(pf_, selective_resampling_);
        pf_->pop_err = pf_err_;
        pf_->pop_z = pf_z_;

        //pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);

        //set each particle random pose within free space in current map, if relocalization is enabled
        if(first_map_received_)
            setUniformDistributionPose(max_relocalization_particles_);
        xlog->Info("reset reLocalization particles!\n");
    }
    else if (Gaussion_Distribution == msg.pdfType)
    {
        if (pf_ != NULL)
        {
            pf_free(pf_);
            pf_ = NULL;
        }
        pf_ = pf_alloc(min_particles_, max_particles_,
                       alpha_slow_, alpha_fast_,
                       (pf_init_model_fn_t)Amcl_t::uniformPoseGenerator,
                       (void *)map_);
        pf_set_selective_resampling(pf_, selective_resampling_);
        pf_->pop_err = pf_err_;
        pf_->pop_z = pf_z_;

        delete initial_pose_hyp_;
        initial_pose_hyp_ = new amcl_hyp_t();
        initial_pose_hyp_->pf_pose_mean = pf_init_pose_mean;
        initial_pose_hyp_->pf_pose_cov = pf_init_pose_cov;
        applyInitialPose();
    }    
}

void Amcl_t::setupMsgCb()
{
    laserSub = lcm_m.subscribe(LCM_CHANNEL_Laser, &Amcl_t::laserMsgUpdate, this);
    laserSub->setQueueCapacity(1);

    //districtSub= lcm_m.subscribe("BlockInfoFromSlam", &Amcl_t::districtUpdate, this);
    //districtSub->setQueueCapacity(1);
    
    mapInfoSub = lcm_m.subscribe(LCM_CHANNEL_MapInfo, &Amcl_t::gridMapInfoUpdate, this);
    mapInfoSub->setQueueCapacity(1);
    /*
    odomSub = lcm_m.subscribe(LCM_CHANNEL_Odom, &Amcl_t::odomMsgUpdate, this);
    odomSub->setQueueCapacity(1);
    */

    initPoseCmdSub = lcm_m.subscribe(LCM_CHANNEL_InitAmclPose, &Amcl_t::initPoseCmdUpdate, this);
    initPoseCmdSub->setQueueCapacity(1);

    gridmapSub = lcm_m.subscribe(LCM_CHANNEL_AmclMap, &Amcl_t::gridMapUpdate, this);
    gridmapSub->setQueueCapacity(1);

    rotationPoseSub = lcm_m.subscribe("map_Rotation_Msg", &Amcl_t::rotatePoseCmdUpdate, this);
    rotationPoseSub->setQueueCapacity(1);

    hackCmdSub = lcm_m.subscribe("AmclToSlamCmd", &Amcl_t::exceptionCmdUpdate, this);
    hackCmdSub->setQueueCapacity(1); 

    robotStatusSub = lcm_m.subscribe(LCM_CHANNEL_RobotStatus,&Amcl_t::robotStatusUpdate,this);
    robotStatusSub->setQueueCapacity(1);

    rpcSub = lcm_m.subscribe(LCM_CHANNEL_RpcCmd, &Amcl_t::rpcCmd, this);
    rpcSub->setQueueCapacity(1);

    teleopHackCmd=lcm_m.subscribe(LCM_CHANNEL_HACK, &Amcl_t::teleopCmdUpdate, this);
    teleopHackCmd->setQueueCapacity(1);
}

void Amcl_t::teleopCmdUpdate(const lcm::ReceiveBuffer* rbuf, 
                    const std::string &channel, 
                    const RobotMsg::HackCmd_t *msg)
{
    std::lock_guard<std::mutex> lock(mtx);
    if(!pf_)
    {
        xlog->Info("pf has not been initialized! \n");
    }
    switch (msg->data)
    {
    case 101:
        xlog->Info("pf add direction x:+0.05m \n");
        hackPf(pf_,0.05,0,0);
        break;
    case 102:
        xlog->Info("pf add direction x:-0.05m \n");
        hackPf(pf_,-0.05,0,0);
        break;
    case 103:
        xlog->Info("pf add direction y:+0.05m \n");
        hackPf(pf_,0,0.05,0);
        break;
    case 104:
        xlog->Info("pf add direction y:-0.05m \n");
        hackPf(pf_,0,-0.05,0);
        break;
    case 105:
        xlog->Info("pf add direction theta:+0.05rad \n");
        hackPf(pf_,0,0,0.1);
        break;
    case 106:
        xlog->Info("pf add direction theta:-0.05rad \n");
        hackPf(pf_,0,0,-0.1);
        break;
    case 107:
        alpha1_ += 0.1;
        xlog->Info("alpha1_:+0.1,current:%f,%f,%f,%f \n",alpha1_,alpha2_,alpha3_,alpha4_);
        break;
    case 108:
        if(alpha1_ > 0)
            alpha1_ += -0.1;
        xlog->Info("alpha1_:-0.1,current:%f,%f,%f,%f \n",alpha1_,alpha2_,alpha3_,alpha4_);
        break; 
    case 109:
        alpha2_ += 0.1;
        xlog->Info("alpha2_:+0.1,current:%f,%f,%f,%f \n",alpha1_,alpha2_,alpha3_,alpha4_);
        break; 
    case 110:
        if(alpha2_ > 0)
        alpha2_ += -0.1;
        xlog->Info("alpha2_:-0.1,current:%f,%f,%f,%f \n",alpha1_,alpha2_,alpha3_,alpha4_);
        break; 
    case 111:
        alpha3_ += 0.1;
        xlog->Info("alpha3_:+0.1,current:%f,%f,%f,%f \n",alpha1_,alpha2_,alpha3_,alpha4_);
        break; 
    case 112:
        if(alpha3_ > 0)
        alpha3_ += -0.1;
        xlog->Info("alpha3_:-0.1,current:%f,%f,%f,%f \n",alpha1_,alpha2_,alpha3_,alpha4_);
        break; 
    case 113:
        alpha4_ += 0.1;
        xlog->Info("alpha4_:+0.1,current:%f,%f,%f,%f \n",alpha1_,alpha2_,alpha3_,alpha4_);
        break; 
    case 114:
        if(alpha4_ > 0)
        alpha4_ += -0.1;
        xlog->Info("alpha4_:-0.1,current:%f,%f,%f,%f \n",alpha1_,alpha2_,alpha3_,alpha4_);
        break;
    case 115:
        xlog->Info("useParamCorrect=true; \n");
        useParamCorrect=true;
        break;
    case 116:
        xlog->Info("useParamCorrect=false; \n");
        useParamCorrect=false;
        break;        
    case 117:
        xlog->Info("usePfCorrect=true; \n");
        usePfCorrect=true;
        break;  
    case 118:
        xlog->Info("usePfCorrect=false; \n");
        usePfCorrect=false;
        break;                                                                                                                           
    default:
        break;
    }

    if (NULL != odom_)
    {
        delete odom_;
        odom_ = NULL;
    }
    odom_ = new amcl::AMCLOdom();
    odom_->SetModel(odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_);
    xlog->Info("odom model parameters has changed! \n");        
}

void Amcl_t::hackPf(pf_t* pf,float x,float y,float theta)
{
    pf_sample_set_t *set;
    set = pf->sets + pf->current_set;

    for (int i = 0; i < set->sample_count; i++)
    {
        pf_sample_t* sample = set->samples + i;
        sample->pose.v[0] += x;
        sample->pose.v[1] += y;
        sample->pose.v[2] += theta;
        sample->pose.v[2] = ClipAngle(sample->pose.v[2]);
    }     
}

void Amcl_t::exceptionCmdUpdate(const lcm::ReceiveBuffer* rbuf, 
                    const std::string &channel, 
                    const RobotMsg::HackCmd_t *msg)
{
    if(msg->data==1 || msg->data==0)
    {
        xlog->Info("get hack cmd,relocalization done! \n");
        resetOdom=true;
    }
}

void Amcl_t::robotStatusUpdate(const lcm::ReceiveBuffer* rbuf, 
                                const std::string &channel, 
                                const RobotMsg::RobotStatus_t *msg)
{
    std::lock_guard<std::recursive_mutex> clk(mtxConfig);

    if(msg->rightBumped || msg->leftBumped || msg->bumperProcessing)
    {
        bumpTs = getTimeStap_ms() * 0.001;
    }

    float q0 = msg->q0 / 10000.0;
    float q1 = msg->q1 / 10000.0;
    float q2 = msg->q2 / 10000.0;
    float q3 = msg->q3 / 10000.0;   
    float pitchTemp = asin(-2*q1*q3+2*q0*q2);
    //roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1);

    if(fabs(pitchTemp) > 0.01 && pitchTick == 0)
    {
        return;
    }

    pitchTick++;
    if(pitchTick > 100)
    {
        pitchDiff = fabs(pitchTemp - pitchInitial);
        //xlog->Info("pitch diff:%f \n",pitchDiff);
        if(pitchDiff > 0.05)
        {
            pitchCheckTick++;
            if(pitchCheckTick > 20)
            {
                pitchInvaild=true;
                xlog->Info("set pitch invalid,tick:%d,pitch diff:%f \n",pitchCheckTick,pitchDiff);
                xlog->Info("pitch initial:%f,current:%f \n",pitchInitial,pitchTemp);
            }
        }
        else
        {
            pitchCheckTick = 0;
            pitchInvaild=false;
            //xlog->Info("pitch vaild \n");
        }
    }
    else if(pitchTick < 100)
    {
        pitchSum += pitchTemp;
    }  
    else
    {
        pitchInitial = (pitchSum + pitchTemp) / 100.0;
    }
    //xlog->Info("got robot status! \n");
}

void Amcl_t::rpcCmd(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const UtilsMsg::RpcCmd_t *msg)
{
    int key = msg->key;

    if (key==(int)UtilsMsg::RpcCmdKey_t::EnterSleep)
    {
        xlog->Info("enter sleep \n");
        exit(0);
    }

    if(key == 11)
    {
        xlog->Info("clean start! \n");
        doClean=true;
        doCleanChange=true;
        outfile1<<getTimeStap_ms()/1000.0 <<"    clean start!"<<int(doClean)<<int(doCleanChange)<<std::endl;
        sendEndCmd=false;       
    }
    if(key == 10)
    {
        xlog->Info("clean end! \n");
        doClean=false;
        doCleanChange=false;
        outfile1<<getTimeStap_ms()/1000.0 <<"    clean end!"<<int(doClean)<<int(doCleanChange)<<std::endl;
        sendEndCmd=true;        
    }
}


void Amcl_t::districtUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const NavMsg::PoseArray_t *msg)
{
    xlog->Info("111111111111111111111111\n");
}

pf_vector_t Amcl_t::uniformPoseGenerator(void* arg)
{
  map_t* map = (map_t*)arg;
  //xlog->Info("size_:%d\n", free_space_indices.size());
  //xlog->Info("map.size_x:%d\n", map->size_x);
  //std::cout<<"size:"<<free_space_indices.size()<<std::endl;
  unsigned int rand_index = drand48() * free_space_indices.size();
  std::pair<int,int> free_point = free_space_indices[rand_index];
  pf_vector_t p;
  p.v[0] = MAP_WXGX(map, free_point.first);
  p.v[1] = MAP_WYGY(map, free_point.second);
  p.v[2] = drand48() * 2 * M_PI - M_PI;

  //xlog->Info("_(%.3f, %.3f, %.3f)\n", p.v[0], p.v[1], p.v[2]);
  //std::cout<<" ("<<p.v[0]<<", "<<p.v[1]<<", "<<p.v[2]<<")"<<std::endl;

  return p;
}

void Amcl_t::loadParams()
{
    XCfg *cf = new XCfg(std::string("amcl.cfg"));
    bool enLog = cf->ReadBool("enLog", false);
    int xlogLevel = cf->ReadInt("xlogLevel",4);
    if(!enLog)
        xlog = new XLog(false);
    else 
        xlog = new XLog(true);
    xlog->Initialise("amcl.log");
    xlog->SetThreshold(XLOG_LEVEL(xlogLevel));

    first_map_only_ = cf->ReadBool("first_map_only", false);

    init_pose_[0] = cf->ReadFloat("initial_pose_x", 0.0);
    init_pose_[1] = cf->ReadFloat("initial_pose_y", 0.0);
    init_pose_[2] = cf->ReadFloat("initial_pose_a", 0.0);
    init_cov_[0] = cf->ReadFloat("initial_cov_xx", 0.01);
    init_cov_[1] = cf->ReadFloat("initial_cov_yy", 0.01);
    init_cov_[2] = cf->ReadFloat("initial_cov_aa", 0.01);
    laserMountPose.px = cf->ReadFloat("laserMountPx", 0.15);
    laserMountPose.py = cf->ReadFloat("laserMountPy", 0);
    laserMountPose.pa = cf->ReadFloat("laserMountPa", 0);

    laser_min_range_ = cf->ReadFloat("laser_min_range", -0.15);
    laser_max_range_ = cf->ReadFloat("laser_max_range", 5.0);
    min_particles_ = cf->ReadInt("min_particles", 100);
    max_particles_ = cf->ReadInt("max_particles", 200);
    max_relocalization_particles_ = cf->ReadInt("max_relocalization_particles_", 3000);

    max_beams_ = cf->ReadInt("laser_max_beams", 30);
    xlog->Info("Amcl load cfg, maxParticles=%d minParticles=%d\n", max_particles_, min_particles_);
    pf_err_ = cf->ReadFloat("kld_err", 0.01); 
    pf_z_ = cf->ReadFloat("kld_z", 0.99);

    alpha1_ = cf->ReadFloat("odom_alpha1", 0.2);
    alpha2_ = cf->ReadFloat("odom_alpha2", 0.2);
    alpha3_ = cf->ReadFloat("odom_alpha3", 0.2);
    alpha4_ = cf->ReadFloat("odom_alpha4", 0.2);
    alpha5_ = cf->ReadFloat("odom_alpha5", 0.2);

    do_beamskip_ = cf->ReadBool("do_beamskip", false);
    beam_skip_distance_ = cf->ReadFloat("beam_skip_distance", 0.5);
    beam_skip_threshold_ = cf->ReadFloat("beam_skip_threshold", 0.3);
    beam_skip_error_threshold_ = cf->ReadFloat("beam_skip_error_threshold_", beam_skip_error_threshold_);

    z_hit_ = cf->ReadFloat("laser_z_hit", 0.95);
    z_short_ = cf->ReadFloat("laser_z_short", 0.1);
    z_max_ = cf->ReadFloat("laser_z_max", 0.05);
    z_rand_ = cf->ReadFloat("laser_z_rand", 0.05);
    sigma_hit_ = cf->ReadFloat("laser_sigma_hit", 0.2);
    lambda_short_ = cf->ReadFloat("laser_lambda_short", 0.1);

    laser_likelihood_max_dist_ = cf->ReadFloat("laser_likelihood_max_dist_", 0.2);

    std::string tmp_model_type = cf->ReadString("laser_model_type","likelihood_field");

    if(tmp_model_type == "beam")
        laser_model_type_ = amcl::LASER_MODEL_BEAM;
    else if(tmp_model_type == "likelihood_field")
        laser_model_type_ = amcl::LASER_MODEL_LIKELIHOOD_FIELD;
    else if(tmp_model_type == "likelihood_field_prob"){
        laser_model_type_ = amcl::LASER_MODEL_LIKELIHOOD_FIELD_PROB;
    }
    else
    {
        xlog->Info("Unknown laser model type \"%s\"; defaulting to likelihood_field model",
                tmp_model_type.c_str());
        laser_model_type_ = amcl::LASER_MODEL_LIKELIHOOD_FIELD;
    }

    tmp_model_type = cf->ReadString("odom_model_type","diff");
    if(tmp_model_type == "diff")
        odom_model_type_ = amcl::ODOM_MODEL_DIFF;
    else if(tmp_model_type == "omni")
        odom_model_type_ = amcl::ODOM_MODEL_OMNI;
    else if(tmp_model_type == "diff-corrected")
        odom_model_type_ = amcl::ODOM_MODEL_DIFF_CORRECTED;
    else if(tmp_model_type == "omni-corrected")
        odom_model_type_ = amcl::ODOM_MODEL_OMNI_CORRECTED;
    else
    {
        xlog->Info("Unknown odom model type \"%s\"; defaulting to diff model\n",
                tmp_model_type.c_str());
        odom_model_type_ = amcl::ODOM_MODEL_DIFF;
    }
    xlog->Debug("odom_model_type = %s", tmp_model_type.c_str());
    d_thresh_ = cf->ReadFloat("update_min_d", 0.02);
    a_thresh_ = cf->ReadFloat("update_min_a", M_PI/90);

    resample_interval_ = cf->ReadFloat("resample_interval", 1);
    selective_resampling_ = cf->ReadBool("selective_resampling", false);
    // double tmp_tol;
    // tmp_tol = cf->ReadFloat("transform_tolerance", 0.1);
    alpha_slow_ = cf->ReadFloat("recovery_alpha_slow", 0.001);
    alpha_fast_ = cf->ReadFloat("recovery_alpha_fast", 0.1);
    tf_broadcast_ = cf->ReadBool("tf_broadcast", true);

    // For diagnostics
    std_warn_level_x_ = cf->ReadFloat("std_warn_level_x", 0.2);
    std_warn_level_y_ = cf->ReadFloat("std_warn_level_y", 0.2);
    std_warn_level_yaw_ = cf->ReadFloat("std_warn_level_yaw", 0.1);

    mapFromMapServer = cf->ReadFloat("MapFromMapServer", false);

    laserCorrection = cf->ReadBool("laserCorrection", false);

    useParamCorrect = cf->ReadBool("useParamCorrect", false);
    usePfCorrect = cf->ReadBool("usePfCorrect", false);
}

Amcl_t::Amcl_t(/* args */):
        pf_(NULL),
        resample_count_(0),
        odom_(NULL),
        laser_(NULL),
        initial_pose_hyp_(NULL),
        first_map_received_(false),
        first_reconfigure_call_(true)
{
    map_ = NULL;
    laserSub = NULL;
    odomSub = NULL;
    gridmapSub = NULL;
    initPoseCmdSub = NULL;
    rotationPoseSub = NULL;
    robotStatusSub = NULL;
    rpcSub = NULL;

    laserUpdate = false;
    odomUpdate = false;
    isRunning = false;
    amclPoseUpdate = false;

    initPose.px = 0;
    initPose.py = 0;
    initPose.pa = 0;

    rotationPose.px=0;
    rotationPose.py=0;
    rotationPose.pa=0;

    total_score = 0.0;

    laserCount=1;
    
    lasers_update_.clear();//
    odomFilter.setLen(10);

    pdf_type = Gaussion_Distribution;

    curMapTs = 0;
    slamMapTs = 0;

    loadParams();
    //setupMsgCb();
    //requestMap();//for testing localCostMap
    hitScore = 0.0;
    forwardHitScore = 0.0;
    forwardPfSet=false;
    devGood.clear();
    resetOdom = false;

    pitchTick = 0;
    pitchSum = 0.0;
    pitchInitial = 0.0;
    pitchDiff = 0.0;
    pitchInvaild=false;
    bumpTs = 0.0;

    odomModelHasSet = false;

    doClean=false;
    doCleanChange=false;

    outfile1.open("/tmp/amclTest.log");
    if(!outfile1.is_open())
    {
        xlog->Error("fail to open file! \n");
    }
    outfile1 << std::fixed << std::setprecision(5);  
    outfile1 << "odom ts,x,y,theta,slam map ts,laser ts,rang0 \n";

    sendEndCmd=false;

}

Amcl_t::~Amcl_t()
{
    freeMapDependentMemory();
    if(outfile1.is_open())
    {
        outfile1.close();
    }    
}