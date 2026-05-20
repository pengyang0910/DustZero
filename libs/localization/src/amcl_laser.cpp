/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
///////////////////////////////////////////////////////////////////////////
//
// Desc: AMCL laser routines
// Author: Andrew Howard
// Date: 6 Feb 2003
// CVS: $Id: amcl_laser.cc 7057 2008-10-02 00:44:06Z gbiggs $
//
///////////////////////////////////////////////////////////////////////////

#include <sys/types.h> // required by Darwin
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif

#include "amcl_laser.h"
#include "NavUtils/NavUtils.h"

using namespace amcl;

////////////////////////////////////////////////////////////////////////////////
// Default constructor
AMCLLaser::AMCLLaser(size_t max_beams, map_t* map) : AMCLSensor(), 
						     max_samples(0), max_obs(0), 
						     temp_obs(NULL)
{
  this->time = 0.0;

  this->max_beams = max_beams;
  this->map = map;

  return;
}

AMCLLaser::~AMCLLaser()
{
  if(temp_obs){
	for(int k=0; k < max_samples; k++){
	  delete [] temp_obs[k];
	}
	delete []temp_obs; 
  }
}

void 
AMCLLaser::SetModelBeam(double z_hit,
                        double z_short,
                        double z_max,
                        double z_rand,
                        double sigma_hit,
                        double lambda_short,
                        double chi_outlier)
{
  this->model_type = LASER_MODEL_BEAM;
  this->z_hit = z_hit;
  this->z_short = z_short;
  this->z_max = z_max;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;
  this->lambda_short = lambda_short;
  this->chi_outlier = chi_outlier;
}

void 
AMCLLaser::SetModelLikelihoodField(double z_hit,
                                   double z_rand,
                                   double sigma_hit,
                                   double max_occ_dist)
{
  this->model_type = LASER_MODEL_LIKELIHOOD_FIELD;
  this->z_hit = z_hit;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;

  map_update_cspace(this->map, max_occ_dist);
}

void 
AMCLLaser::SetModelLikelihoodFieldProb(double z_hit,
				       double z_rand,
				       double sigma_hit,
				       double max_occ_dist,
				       bool do_beamskip,
				       double beam_skip_distance,
				       double beam_skip_threshold, 
				       double beam_skip_error_threshold)
{
  this->model_type = LASER_MODEL_LIKELIHOOD_FIELD_PROB;
  this->z_hit = z_hit;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;
  this->do_beamskip = do_beamskip;
  this->beam_skip_distance = beam_skip_distance;
  this->beam_skip_threshold = beam_skip_threshold;
  this->beam_skip_error_threshold = beam_skip_error_threshold;
  map_update_cspace(this->map, max_occ_dist);
}


////////////////////////////////////////////////////////////////////////////////
// Apply the laser sensor model
bool AMCLLaser::UpdateSensor(pf_t *pf, AMCLSensorData *data)
{
  if (this->max_beams < 2)
    return false;

  // Apply the laser sensor model
  if(this->model_type == LASER_MODEL_BEAM)
    pf_update_sensor(pf, (pf_sensor_model_fn_t) BeamModel, data);
  else if(this->model_type == LASER_MODEL_LIKELIHOOD_FIELD)
    pf_update_sensor(pf, (pf_sensor_model_fn_t) LikelihoodFieldModel, data);  
  else if(this->model_type == LASER_MODEL_LIKELIHOOD_FIELD_PROB)
    pf_update_sensor(pf, (pf_sensor_model_fn_t) LikelihoodFieldModelProb, data);  
  else
    pf_update_sensor(pf, (pf_sensor_model_fn_t) BeamModel, data);

  return true;
}


////////////////////////////////////////////////////////////////////////////////
// Determine the probability for the given pose
double AMCLLaser::BeamModel(AMCLLaserData *data, pf_sample_set_t* set)
{
  AMCLLaser *self;
  int i, j, step;
  double z, pz;
  double p;
  double map_range;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;

  self = (AMCLLaser*) data->sensor;

  total_weight = 0.0;

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    p = 1.0;

    step = (data->range_count - 1) / (self->max_beams - 1);
    for (i = 0; i < data->range_count; i += step)
    {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      // Compute the range according to the map
      map_range = map_calc_range(self->map, pose.v[0], pose.v[1],
                                 pose.v[2] + obs_bearing, data->range_max);
      pz = 0.0;

      // Part 1: good, but noisy, hit
      z = obs_range - map_range;
      pz += self->z_hit * exp(-(z * z) / (2 * self->sigma_hit * self->sigma_hit));

      // Part 2: short reading from unexpected obstacle (e.g., a person)
      if(z < 0)
        pz += self->z_short * self->lambda_short * exp(-self->lambda_short*obs_range);

      // Part 3: Failure to detect obstacle, reported as max-range
      if(obs_range == data->range_max)
        pz += self->z_max * 1.0;

      // Part 4: Random measurements
      if(obs_range < data->range_max)
        pz += self->z_rand * 1.0/data->range_max;

      // TODO: outlier rejection for short readings

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz*pz*pz;
    }

    sample->weight *= p;
    total_weight += sample->weight;
  }

  return(total_weight);
}

double AMCLLaser::LikelihoodFieldModel(AMCLLaserData *data, pf_sample_set_t* set)
{
  AMCLLaser *self;
  int i, j, step;
  double z, pz;
  double p;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;
  pf_vector_t hit;

  self = (AMCLLaser*) data->sensor;

  total_weight = 0.0;
  
  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    p = 1.0;

    // Pre-compute a couple of things
    //double z_hit_denom = 2 * self->sigma_hit * self->sigma_hit;
    double z_rand_mult = 1.0/data->range_max;

    step = (data->range_count - 1) / (self->max_beams - 1);

    // Step size must be at least 1
    if(step < 1)
      step = 1;

    for (i = 0; i < data->range_count; i += step)
    {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      // This model ignores max range readings
      if(obs_range >= data->range_max)
        continue;

      // Check for NaN
      if(obs_range != obs_range)
        continue;

      // if(obs_range < 1.5)
      // {
      //   obs_range=0.1*obs_range;
      // }
      // else
      // {
      //   obs_range=0.0268*obs_range+0.1098;
      // }

      pz = 0.0;

      // Compute the endpoint of the beam
      hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
      hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      // Convert to map grid coords.
      int mi, mj;
      mi = MAP_GXWX(self->map, hit.v[0]);
      mj = MAP_GYWY(self->map, hit.v[1]);
      
      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      if(!MAP_VALID(self->map, mi, mj))
        z = self->map->max_occ_dist;
      else
        z = self->map->cells[MAP_INDEX(self->map,mi,mj)].occ_dist;

      double updateSigma=obs_range*0.05/self->sigma_hit;
      if(updateSigma > 1)
      {
        updateSigma=1;
      }
      else if(updateSigma < 0.25)
      {
        updateSigma=0.25;
      }
      if(z > self->sigma_hit*updateSigma)
      {
        z=self->sigma_hit*updateSigma;
      }
      double z_hit_denom = 2 * self->sigma_hit * self->sigma_hit*updateSigma*updateSigma;

      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
      pz += self->z_hit * exp(-(z * z) / z_hit_denom);
      // Part 2: random measurements
      pz += self->z_rand * z_rand_mult;

      // TODO: outlier rejection for short readings

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz*pz*pz;
    }

    sample->weight *= p;
    total_weight += sample->weight;
  }
  return(total_weight);
}

double AMCLLaser::fowardHitScore(AMCLLaserData *data,pf_vector_t amclPoseHit,int startNum,int endNum,float ldist)
{
  AMCLLaser *self;
  self = (AMCLLaser*) data->sensor;
  pf_vector_t pose = pf_vector_coord_add(self->laser_pose, amclPoseHit);
  pose.v[2]=ClipAngle(pose.v[2]);
  int startX=MAP_GXWX(self->map, pose.v[0]);
  int startY=MAP_GYWY(self->map, pose.v[1]);
  // printf("map size,x:%d,y:%d,origin,x:%f,y:%f,amcl odom,x:%d,y:%d \n",
  //        self->map->size_x,self->map->size_y,self->map->origin_x,self->map->origin_y,
  //        startX,startY);
  if(!MAP_VALID(self->map, startX, startY))
  {
    //printf("invalid map point! \n");
    return 0.0;
  }
  double obs_range, obs_bearing;
  pf_vector_t hit;
  int endX,endY;
  double result=0.0;
  int lastEndX=0;
  int lastEndY=0;
  int validlaserPointCount=1200;
  int hitPoint=0;
  int hitThroughPoint=0;
  int hitForwardPoint=0;
  float averagehitDist=0;
  int averageCount=0;
  float absAveragehitDist=0;
  for (int i = startNum; i < endNum; i++)
  {
    obs_range = data->ranges[i][0];
    // if(obs_range < 1.5)
    // {
    //   obs_range=0.1*obs_range;
    // }
    // else
    // {
    //   obs_range=0.0268*obs_range+0.1098;
    // }    
    obs_bearing = data->ranges[i][1];  
    //printf("seq:%d,obs range:%f,obs bearing:%f \n",i,obs_range,obs_bearing);    
    if(obs_range >= data->range_max)
    {
      validlaserPointCount--;
      continue;
    }
    if(obs_range != obs_range)
    {
      validlaserPointCount--;
      continue;
    }
    hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
    hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);
    endX=MAP_GXWX(self->map, hit.v[0]);
    endY=MAP_GYWY(self->map, hit.v[1]);
    //printf("start:x=%d,y=%d,end:x=%d,y=%d \n",startX,startY,endX,endY);
    // if(endX == lastEndX && endY == lastEndY)
    // {
    //   printf("skip point in same cell \n");
    //   continue;
    // }
    // else
    // {
    //   lastEndX=endX;
    //   lastEndY=endY;
    // }
    if(!MAP_VALID(self->map, endX, endY))
    {
      continue;
    }
    //bresenham2D
    int dx=endX-startX;
    int dy=endY-startY;
    unsigned int absX=abs(dx);
    unsigned int absY=abs(dy);
    int offsetX=sign(dx);
    int offsetY=sign(dy)*self->map->size_x;
    unsigned int offset=startX+startY*self->map->size_x;
    int errorSum=0;
    double hitDist=sqrt((startX-endX)*(startX-endX)+(startY-endY)*(startY-endY));
    double dist=0;
    // extend ray line,in X or Y direction
    float rateX=0;
    float rateY=0;
    if(dx > 0)
    {
      rateX=float(self->map->size_x-1-endX)/dx;
    }
    else if(dx == 0)
    {
      rateX=2.0;
    }
    else
    {
      rateX=float(0-endX)/dx;
    }
    if(dy > 0)
    {
      rateY=float(self->map->size_y-1-endY)/dy;
    }
    else if(dy == 0)
    {
      rateY=2.0;
    }
    else
    {
      rateY=float(0-endY)/dy;
    }
    //printf("rateX:%f,rateY:%f \n",rateX,rateY);
    // assert(rateX > 0.00001);
    // assert(rateY > 0.00001);
    float rate=1;
    if(rateX > rateY)
    {
      rate+=rateY;
    }
    else
    {
      rate+=rateX;
    }
    if(rate > 2.0)
    {
      rate=2.0;
    }
    absX=absX*rate;
    absY=absY*rate;    
    //printf("max size:x:%d,y:%d \n",self->map->size_x,self->map->size_y);
    //printf("raytrace rate:%f \nabsX:%d,absY:%d \nextended line x:%d,y:%d \n",rate,absX,absY,startX+absX*sign(dx),startY+absY*sign(dy));

    if(absX >= absY)
    {
      dist=bresenham(absX,absY,offsetX,offsetY,offset,self->map);
    }
    else
    {
      dist=bresenham(absY,absX,offsetY,offsetX,offset,self->map);
    }
    double percent=dist/hitDist*100.0;
    //printf("dist:%f,hit dist:%f,percent:%f \n",dist,hitDist,percent);
    if((percent > 95 && percent < 105) || fabs(dist -hitDist) < 3.0)
    {
      hitPoint++;
    }
    else if(percent >= 0 && percent <= 95)
    {
      hitThroughPoint++;
    }
    else
    {
      hitForwardPoint++;      
    }
    if(hitDist > 6  && hitDist < (ldist/self->map->scale) &&  (percent > 50 && percent < 150))
    {
      averagehitDist+=(dist-hitDist);
      absAveragehitDist+=fabs(dist-hitDist);
      averageCount++;
    }
  }

  if(averageCount != 0)
  {
    averagehitDist=averagehitDist/averageCount;
    absAveragehitDist=absAveragehitDist/averageCount;
  }
  else
  {
    averagehitDist=0;
    absAveragehitDist=0;
  }  
  //printf("average hit dist:%f,absolute average dist:%f \n",averagehitDist,absAveragehitDist);
  // printf("hit point:%d,valid laser point:%d,hit through point:%d,hit forward point:%d \nhit point percent:%f \n",
  //        hitPoint,validlaserPointCount,hitThroughPoint,hitForwardPoint,double(hitPoint)/validlaserPointCount*100.0);
  if(validlaserPointCount < 1)
  {
    return 0.0;
  }
  result=double(hitPoint)/validlaserPointCount*100.0;
  return averagehitDist;
}

double AMCLLaser::laserHitThroughScore(AMCLLaserData *data,pf_vector_t amclPoseHit)
{
  AMCLLaser *self;
  self = (AMCLLaser*) data->sensor;
  //printf("amclpose: x:%f,y:%f,theta:%f \n",amclPoseHit.v[0],amclPoseHit.v[1],amclPoseHit.v[2]);
  pf_vector_t pose = pf_vector_coord_add(self->laser_pose, amclPoseHit);
  //printf("after laser mount x:%f,y:%f,theta:%f \n",pose.v[0],pose.v[1],pose.v[2]);
  pose.v[2]=ClipAngle(pose.v[2]);
  int startX=MAP_GXWX(self->map, pose.v[0]);
  int startY=MAP_GYWY(self->map, pose.v[1]);
  //printf("map size,x:%d,y:%d,origin,x:%f,y:%f,amcl odom,x:%d,y:%d \n",
  //       self->map->size_x,self->map->size_y,self->map->origin_x,self->map->origin_y,
  //       startX,startY);
  if(!MAP_VALID(self->map, startX, startY))
  {
    //printf("invalid map point! \n");
    return 0.0;
  }
  double obs_range, obs_bearing;
  pf_vector_t hit;
  int endX,endY;
  double result=0.0;
  int lastEndX=0;
  int lastEndY=0;
  int validlaserPointCount=1200;
  int hitPoint=0;
  int hitThroughPoint=0;
  int hitForwardPoint=0;
  for (int i = 0; i < data->range_count; i++)
  {
    obs_range = data->ranges[i][0];
    // if(obs_range < 1.5)
    // {
    //   obs_range=0.1*obs_range;
    // }
    // else
    // {
    //   obs_range=0.0268*obs_range+0.1098;
    // }        
    obs_bearing = data->ranges[i][1];
    //printf("seq:%d,obs range:%f,obs bearing:%f \n",i,obs_range,obs_bearing);    
    if(obs_range >= data->range_max)
    {
      validlaserPointCount--;
      continue;
    }
    if(obs_range != obs_range)
    {
      validlaserPointCount--;
      continue;
    }
    hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
    hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);
    endX=MAP_GXWX(self->map, hit.v[0]);
    endY=MAP_GYWY(self->map, hit.v[1]);
    //printf("start:x=%d,y=%d,end:x=%d,y=%d \n",startX,startY,endX,endY);
    // if(endX == lastEndX && endY == lastEndY)
    // {
    //   printf("skip point in same cell \n");
    //   continue;
    // }
    // else
    // {
    //   lastEndX=endX;
    //   lastEndY=endY;
    // }
    if(!MAP_VALID(self->map, endX, endY))
    {
      continue;
    }
    //bresenham2D
    int dx=endX-startX;
    int dy=endY-startY;
    unsigned int absX=abs(dx);
    unsigned int absY=abs(dy);
    int offsetX=sign(dx);
    int offsetY=sign(dy)*self->map->size_x;
    unsigned int offset=startX+startY*self->map->size_x;
    int errorSum=0;
    double hitDist=sqrt((startX-endX)*(startX-endX)+(startY-endY)*(startY-endY));
    double dist=0;
    // extend ray line,in X or Y direction
    float rateX=0;
    float rateY=0;
    if(dx > 0)
    {
      rateX=float(self->map->size_x-1-endX)/dx;
    }
    else if(dx == 0)
    {
      rateX=2.0;
    }
    else
    {
      rateX=float(0-endX)/dx;
    }
    if(dy > 0)
    {
      rateY=float(self->map->size_y-1-endY)/dy;
    }
    else if(dy == 0)
    {
      rateY=2.0;
    }
    else
    {
      rateY=float(0-endY)/dy;
    }
    //printf("rateX:%f,rateY:%f \n",rateX,rateY);
    // assert(rateX > 0.00001);
    // assert(rateY > 0.00001);
    float rate=1;
    if(rateX > rateY)
    {
      rate+=rateY;
    }
    else
    {
      rate+=rateX;
    }
    if(rate > 2.0)
    {
      rate=2.0;
    }
    absX=absX*rate;
    absY=absY*rate;    
    //printf("max size:x:%d,y:%d \n",self->map->size_x,self->map->size_y);
    //printf("raytrace rate:%f \nabsX:%d,absY:%d \nextended line x:%d,y:%d \n",rate,absX,absY,startX+absX*sign(dx),startY+absY*sign(dy));

    if(absX >= absY)
    {
      dist=bresenham(absX,absY,offsetX,offsetY,offset,self->map);
    }
    else
    {
      dist=bresenham(absY,absX,offsetY,offsetX,offset,self->map);
    }
    double percent=dist/hitDist*100.0;
    //printf("dist:%f,hit dist:%f,percent:%f \n",dist,hitDist,percent);
    if((percent > 95 && percent < 105) || fabs(dist -hitDist) < 3.0)
    {
      hitPoint++;
    }
    else if(percent >= 0 && percent <= 95)
    {
      hitThroughPoint++;
    }
    else
    {
      hitForwardPoint++;      
    }
  }
  // printf("hit point:%d,valid laser point:%d,hit through point:%d,hit forward point:%d \nhit point percent:%f \n",
  //        hitPoint,validlaserPointCount,hitThroughPoint,hitForwardPoint,double(hitPoint)/validlaserPointCount*100.0);
  if(validlaserPointCount < 1)
  {
    return 0.0;
  }
  result=double(hitPoint)/validlaserPointCount*100.0;
  return result;
}

double AMCLLaser::bresenham(const unsigned int a,const unsigned int b,const int offsetA,const int offsetB,unsigned int offset,map_t* map)
{
  int error=0;
  double dist=sqrt(a*a+b*b);
  //printf("initial dist:%f \n",dist);
  int x=0;
  int y=0;
  int startX=0;
  int startY=0;
  //check if x,y valid in map
  int lastX=0;
  int lastY=0;
  for(unsigned int i=0;i<=a;i++)
  {
    y=offset/map->size_x;
    x=offset-y*map->size_x;
    // printf("x=%d,y=%d,a=%d,b=%d,error=%d \n",x,y,a,b,error);
    if(i == 0)
    {
      startX=x;
      startY=y;
      lastX=startX;
      lastY=startY;
    }    
    if(abs(x-lastX) > 1 || abs(y-lastY) > 1 || x < 0 || x >= map->size_x || y < 0 || y >= map->size_y)
    {
      //printf("x,y hit out of map bound,stop! stop! stop! stop! stop! stop! \n");
      //exit(0);
    }
    lastX=x;
    lastY=y;
    if(map->cells[offset].occ_state == 1)
    {
      dist=sqrt((x-startX)*(x-startX)+(y-startY)*(y-startY));
      //printf("bresenham hit,x=%d,y=%d \n",x,y);
      // printf("dist:%f \n",dist);
      break;
    }
    offset+=offsetA;
    error+=b;
    if((error-int(a/2)) > 0)
    {
      error-=a;
      offset+=offsetB;
    }   
  }
  return dist;
}

double AMCLLaser::LikelihoodFieldModelProb(AMCLLaserData *data, pf_sample_set_t* set)
{
  AMCLLaser *self;
  int i, j, step;
  double z, pz;
  double log_p;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;
  pf_vector_t hit;

  self = (AMCLLaser*) data->sensor;

  total_weight = 0.0;

  step = ceil((data->range_count) / static_cast<double>(self->max_beams)); 
  
  // Step size must be at least 1
  if(step < 1)
    step = 1;

  // Pre-compute a couple of things
  double z_hit_denom = 2 * self->sigma_hit * self->sigma_hit;
  double z_rand_mult = 1.0/data->range_max;

  double max_dist_prob = exp(-(self->map->max_occ_dist * self->map->max_occ_dist) / z_hit_denom);

  //Beam skipping - ignores beams for which a majoirty of particles do not agree with the map
  //prevents correct particles from getting down weighted because of unexpected obstacles 
  //such as humans 

  bool do_beamskip = self->do_beamskip;
  double beam_skip_distance = self->beam_skip_distance;
  double beam_skip_threshold = self->beam_skip_threshold;
  
  //we only do beam skipping if the filter has converged 
  if(do_beamskip && !set->converged){
    do_beamskip = false;
  }

  //we need a count the no of particles for which the beam agreed with the map 
  int *obs_count = new int[self->max_beams]();

  //we also need a mask of which observations to integrate (to decide which beams to integrate to all particles) 
  bool *obs_mask = new bool[self->max_beams]();
  
  int beam_ind = 0;
  
  //realloc indicates if we need to reallocate the temp data structure needed to do beamskipping 
  bool realloc = false; 

  if(do_beamskip){
    if(self->max_obs < self->max_beams){
      realloc = true;
    }

    if(self->max_samples < set->sample_count){
      realloc = true;
    }

    if(realloc){
      self->reallocTempData(set->sample_count, self->max_beams);     
      fprintf(stderr, "Reallocing temp weights %d - %d\n", self->max_samples, self->max_obs);
    }
  }

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    log_p = 0;
    
    beam_ind = 0;
    
    for (i = 0; i < data->range_count; i += step, beam_ind++)
    {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      // This model ignores max range readings
      if(obs_range >= data->range_max){
        continue;
      }

      // Check for NaN
      if(obs_range != obs_range){
        continue;
      }

      pz = 0.0;

      // Compute the endpoint of the beam
      hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
      hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      // Convert to map grid coords.
      int mi, mj;
      mi = MAP_GXWX(self->map, hit.v[0]);
      mj = MAP_GYWY(self->map, hit.v[1]);
      
      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      
      if(!MAP_VALID(self->map, mi, mj)){
	pz += self->z_hit * max_dist_prob;
      }
      else{
	z = self->map->cells[MAP_INDEX(self->map,mi,mj)].occ_dist;
	if(z < beam_skip_distance){
	  obs_count[beam_ind] += 1;
	}
	pz += self->z_hit * exp(-(z * z) / z_hit_denom);
      }
       
      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
      
      // Part 2: random measurements
      pz += self->z_rand * z_rand_mult;

      assert(pz <= 1.0); 
      assert(pz >= 0.0);

      // TODO: outlier rejection for short readings
            
      if(!do_beamskip){
	log_p += log(pz);
      }
      else{
	self->temp_obs[j][beam_ind] = pz; 
      }
    }
    if(!do_beamskip){
      sample->weight *= exp(log_p);
      total_weight += sample->weight;
    }
  }
  
  if(do_beamskip){
    int skipped_beam_count = 0; 
    for (beam_ind = 0; beam_ind < self->max_beams; beam_ind++){
      if((obs_count[beam_ind] / static_cast<double>(set->sample_count)) > beam_skip_threshold){
	obs_mask[beam_ind] = true;
      }
      else{
	obs_mask[beam_ind] = false;
	skipped_beam_count++; 
      }
    }

    //we check if there is at least a critical number of beams that agreed with the map 
    //otherwise it probably indicates that the filter converged to a wrong solution
    //if that's the case we integrate all the beams and hope the filter might converge to 
    //the right solution
    bool error = false; 

    if(skipped_beam_count >= (beam_ind * self->beam_skip_error_threshold)){
      fprintf(stderr, "Over %f%% of the observations were not in the map - pf may have converged to wrong pose - integrating all observations\n", (100 * self->beam_skip_error_threshold));
      error = true; 
    }

    for (j = 0; j < set->sample_count; j++)
      {
	sample = set->samples + j;
	pose = sample->pose;

	log_p = 0;

	for (beam_ind = 0; beam_ind < self->max_beams; beam_ind++){
	  if(error || obs_mask[beam_ind]){
	    log_p += log(self->temp_obs[j][beam_ind]);
	  }
	}
	
	sample->weight *= exp(log_p);
	
	total_weight += sample->weight;
      }      
  }

  delete [] obs_count; 
  delete [] obs_mask;
  return(total_weight);
}

void AMCLLaser::reallocTempData(int new_max_samples, int new_max_obs){
  if(temp_obs){
    for(int k=0; k < max_samples; k++){
      delete [] temp_obs[k];
    }
    delete []temp_obs; 
  }
  max_obs = new_max_obs; 
  max_samples = fmax(max_samples, new_max_samples); 

  temp_obs = new double*[max_samples]();
  for(int k=0; k < max_samples; k++){
    temp_obs[k] = new double[max_obs]();
  }
}
