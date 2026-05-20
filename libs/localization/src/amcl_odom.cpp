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
// Desc: AMCL odometry routines
// Author: Andrew Howard
// Date: 6 Feb 2003
// CVS: $Id: amcl_odom.cc 7057 2008-10-02 00:44:06Z gbiggs $
//
///////////////////////////////////////////////////////////////////////////

#include <algorithm>

#include <sys/types.h> // required by Darwin
#include <math.h>

#include "amcl_odom.h"

#include "NavUtils/NavUtils.h"

#include <iostream>
#include <fstream>
#include "iomanip"

using namespace amcl;

static double
normalize(double z)
{
  return atan2(sin(z),cos(z));
}
static double
angle_diff(double a, double b)
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

////////////////////////////////////////////////////////////////////////////////
// Default constructor
AMCLOdom::AMCLOdom() : AMCLSensor()
{
  this->time = 0.0;
}

void
AMCLOdom::SetModelDiff(double alpha1, 
                       double alpha2, 
                       double alpha3, 
                       double alpha4)
{
  this->model_type = ODOM_MODEL_DIFF;
  this->alpha1 = alpha1;
  this->alpha2 = alpha2;
  this->alpha3 = alpha3;
  this->alpha4 = alpha4;
}

void
AMCLOdom::SetModelOmni(double alpha1, 
                       double alpha2, 
                       double alpha3, 
                       double alpha4,
                       double alpha5)
{
  this->model_type = ODOM_MODEL_OMNI;
  this->alpha1 = alpha1;
  this->alpha2 = alpha2;
  this->alpha3 = alpha3;
  this->alpha4 = alpha4;
  this->alpha5 = alpha5;
}

void
AMCLOdom::SetModel( odom_model_t type,
                    double alpha1,
                    double alpha2,
                    double alpha3,
                    double alpha4,
                    double alpha5 )
{
  this->model_type = type;
  this->alpha1 = alpha1;
  this->alpha2 = alpha2;
  this->alpha3 = alpha3;
  this->alpha4 = alpha4;
  this->alpha5 = alpha5;
}

////////////////////////////////////////////////////////////////////////////////
// Apply the action model
bool AMCLOdom::UpdateAction(pf_t *pf, AMCLSensorData *data)
{
  AMCLOdomData *ndata;
  ndata = (AMCLOdomData*) data;

  // Compute the new sample poses
  pf_sample_set_t *set;

  set = pf->sets + pf->current_set;
  pf_vector_t old_pose = pf_vector_sub(ndata->pose, ndata->delta);

  //printf("############ updateAction ##############\r\n");
  switch( this->model_type )
  {
  case ODOM_MODEL_OMNI:
  {
    //printf("ODOM_MODEL_OMNI update\r\n");
    double delta_trans, delta_rot, delta_bearing;
    double delta_trans_hat, delta_rot_hat, delta_strafe_hat;

    delta_trans = sqrt(ndata->delta.v[0]*ndata->delta.v[0] +
                       ndata->delta.v[1]*ndata->delta.v[1]);
    delta_rot = ndata->delta.v[2];

    // Precompute a couple of things
    double trans_hat_stddev = (alpha3 * (delta_trans*delta_trans) +
                               alpha1 * (delta_rot*delta_rot));
    double rot_hat_stddev = (alpha4 * (delta_rot*delta_rot) +
                             alpha2 * (delta_trans*delta_trans));
    double strafe_hat_stddev = (alpha1 * (delta_rot*delta_rot) +
                                alpha5 * (delta_trans*delta_trans));

    for (int i = 0; i < set->sample_count; i++)
    {
      pf_sample_t* sample = set->samples + i;

      delta_bearing = angle_diff(atan2(ndata->delta.v[1], ndata->delta.v[0]),
                                 old_pose.v[2]) + sample->pose.v[2];
      double cs_bearing = cos(delta_bearing);
      double sn_bearing = sin(delta_bearing);

      // Sample pose differences
      delta_trans_hat = delta_trans + pf_ran_gaussian(trans_hat_stddev);
      delta_rot_hat = delta_rot + pf_ran_gaussian(rot_hat_stddev);
      delta_strafe_hat = 0 + pf_ran_gaussian(strafe_hat_stddev);
      // Apply sampled update to particle pose
      sample->pose.v[0] += (delta_trans_hat * cs_bearing + 
                            delta_strafe_hat * sn_bearing);
      sample->pose.v[1] += (delta_trans_hat * sn_bearing - 
                            delta_strafe_hat * cs_bearing);
      sample->pose.v[2] += delta_rot_hat ;
    }
  }
  break;
  case ODOM_MODEL_DIFF:
  {
    //printf("ODOM_MODEL_DIFF update here, set->sample_count=%d \r\n", set->sample_count);

    // Implement sample_motion_odometry (Prob Rob p 136)
    double delta_rot1, delta_trans, delta_rot2;
    double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
    double delta_rot1_noise, delta_rot2_noise;

    // Avoid computing a bearing from two poses that are extremely near each
    // other (happens on in-place rotation).
    if(sqrt(ndata->delta.v[1]*ndata->delta.v[1] + 
            ndata->delta.v[0]*ndata->delta.v[0]) < 0.01)
      delta_rot1 = 0.0;
    else
      delta_rot1 = angle_diff(atan2(ndata->delta.v[1], ndata->delta.v[0]),
                              old_pose.v[2]);
    delta_trans = sqrt(ndata->delta.v[0]*ndata->delta.v[0] +
                       ndata->delta.v[1]*ndata->delta.v[1]);
    delta_rot2 = angle_diff(ndata->delta.v[2], delta_rot1);

    ////step 1
    // printf("step 1,origin diff(delta),delta_rot1:%f,delta_trans:%f,delta_rot2:%f \n",delta_rot1,delta_trans,delta_rot2);

    // We want to treat backward and forward motion symmetrically for the
    // noise model to be applied below.  The standard model seems to assume
    // forward motion.
    delta_rot1_noise = std::min(fabs(angle_diff(delta_rot1,0.0)),
                                fabs(angle_diff(delta_rot1,M_PI)));
    delta_rot2_noise = std::min(fabs(angle_diff(delta_rot2,0.0)),
                                fabs(angle_diff(delta_rot2,M_PI)));

    for (int i = 0; i < set->sample_count; i++)
    {
      pf_sample_t* sample = set->samples + i;

      // Sample pose differences
      double ran_gaussian_rot1_noise = pf_ran_gaussian(this->alpha1*delta_rot1_noise*delta_rot1_noise +
                                      this->alpha2*delta_trans*delta_trans);
      double ran_gaussian_trans_noise = pf_ran_gaussian(this->alpha3*delta_trans*delta_trans +
                                      this->alpha4*delta_rot1_noise*delta_rot1_noise +
                                      this->alpha4*delta_rot2_noise*delta_rot2_noise);
      double ran_gaussian_rot2_noise = pf_ran_gaussian(this->alpha1*delta_rot2_noise*delta_rot2_noise +
                                      this->alpha2*delta_trans*delta_trans);

      delta_rot1_hat = angle_diff(delta_rot1,ran_gaussian_rot1_noise);
      delta_trans_hat = delta_trans - ran_gaussian_trans_noise;
      delta_rot2_hat = angle_diff(delta_rot2,ran_gaussian_rot2_noise);

      // delta_rot1_hat = delta_rot1;
      // delta_trans_hat = delta_trans;
      // delta_rot2_hat = delta_rot2;

      // //origin odom delta
      // sample->pose.v[0] += ndata->delta.v[0];
      // sample->pose.v[1] += ndata->delta.v[1];
      // sample->pose.v[2] += ndata->delta.v[2];
      // sample->pose.v[2]=ClipAngle(sample->pose.v[2]);


      //step 3
      // printf("step 3,delta_rot1_hat:%f,delta_trans_hat:%f,delta_rot2_hat:%f \n",
      //         delta_rot1_hat,delta_trans_hat,delta_rot2_hat);

      // Apply sampled update to particle pose
      sample->pose.v[0] += delta_trans_hat * 
              cos(sample->pose.v[2] + delta_rot1_hat);
      sample->pose.v[1] += delta_trans_hat * 
              sin(sample->pose.v[2] + delta_rot1_hat);
      sample->pose.v[2] += delta_rot1_hat + delta_rot2_hat;
      sample->pose.v[2]=ClipAngle(sample->pose.v[2]);

      //step 4
      // printf("update odom,sample pose seq:%d x:%f,y:%f,theta:%f \n ",
      //         i,sample->pose.v[0],sample->pose.v[1],sample->pose.v[2]); 
                
    }   

    // printf("\norigin data x:%f,y:%f,theta:%f \n",ndata->pose.v[0],ndata->pose.v[1],ndata->pose.v[2]);
    // printf("max x:%f,min x:%f,max y:%f,min y:%f,max theta:%f,min theta:%f \n",max_x,min_x,max_y,min_y,max_theta,min_theta);
    // printf("adf x:%f,idf x:%f,adf y:%f,idf y:%f,adf theta:%f,idf theta:%f \n",
    //         max_x-ndata->pose.v[0],min_x-ndata->pose.v[0],max_y-ndata->pose.v[1],min_y-ndata->pose.v[1],max_theta-ndata->pose.v[2],min_theta-ndata->pose.v[2]);
    

  }
  break;
  case ODOM_MODEL_OMNI_CORRECTED:
  {
      //printf("ODOM_MODEL_OMNI_CORRECTED update\r\n");
    double delta_trans, delta_rot, delta_bearing;
    double delta_trans_hat, delta_rot_hat, delta_strafe_hat;

    delta_trans = sqrt(ndata->delta.v[0]*ndata->delta.v[0] +
                       ndata->delta.v[1]*ndata->delta.v[1]);
    delta_rot = ndata->delta.v[2];

    // Precompute a couple of things
    double trans_hat_stddev = sqrt( alpha3 * (delta_trans*delta_trans) +
                                    alpha4 * (delta_rot*delta_rot) );
    double rot_hat_stddev = sqrt( alpha1 * (delta_rot*delta_rot) +
                                  alpha2 * (delta_trans*delta_trans) );
    double strafe_hat_stddev = sqrt( alpha4 * (delta_rot*delta_rot) +
                                     alpha5 * (delta_trans*delta_trans) );

    for (int i = 0; i < set->sample_count; i++)
    {
      pf_sample_t* sample = set->samples + i;

      delta_bearing = angle_diff(atan2(ndata->delta.v[1], ndata->delta.v[0]),
                                 old_pose.v[2]) + sample->pose.v[2];
      double cs_bearing = cos(delta_bearing);
      double sn_bearing = sin(delta_bearing);

      // Sample pose differences
      delta_trans_hat = delta_trans + pf_ran_gaussian(trans_hat_stddev);
      delta_rot_hat = delta_rot + pf_ran_gaussian(rot_hat_stddev);
      delta_strafe_hat = 0 + pf_ran_gaussian(strafe_hat_stddev);
      // Apply sampled update to particle pose
      sample->pose.v[0] += (delta_trans_hat * cs_bearing + 
                            delta_strafe_hat * sn_bearing);
      sample->pose.v[1] += (delta_trans_hat * sn_bearing - 
                            delta_strafe_hat * cs_bearing);
      sample->pose.v[2] += delta_rot_hat ;
    }
  }
  break;
  case ODOM_MODEL_DIFF_CORRECTED:
  {
    //printf("ODOM_MODEL_DIFF_CORRECTED update\r\n");
    // Implement sample_motion_odometry (Prob Rob p 136)
    double delta_rot1, delta_trans, delta_rot2;
    double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
    double delta_rot1_noise, delta_rot2_noise;

    // Avoid computing a bearing from two poses that are extremely near each
    // other (happens on in-place rotation).
    if(sqrt(ndata->delta.v[1]*ndata->delta.v[1] + 
            ndata->delta.v[0]*ndata->delta.v[0]) < 0.01)
      delta_rot1 = 0.0;
    else
      delta_rot1 = angle_diff(atan2(ndata->delta.v[1], ndata->delta.v[0]),
                              old_pose.v[2]);
    delta_trans = sqrt(ndata->delta.v[0]*ndata->delta.v[0] +
                       ndata->delta.v[1]*ndata->delta.v[1]);
    delta_rot2 = angle_diff(ndata->delta.v[2], delta_rot1);

    // We want to treat backward and forward motion symmetrically for the
    // noise model to be applied below.  The standard model seems to assume
    // forward motion.
    delta_rot1_noise = std::min(fabs(angle_diff(delta_rot1,0.0)),
                                fabs(angle_diff(delta_rot1,M_PI)));
    delta_rot2_noise = std::min(fabs(angle_diff(delta_rot2,0.0)),
                                fabs(angle_diff(delta_rot2,M_PI)));

    for (int i = 0; i < set->sample_count; i++)
    {
      pf_sample_t* sample = set->samples + i;

      // Sample pose differences
      delta_rot1_hat = angle_diff(delta_rot1,
                                  pf_ran_gaussian(sqrt(this->alpha1*delta_rot1_noise*delta_rot1_noise +
                                                       this->alpha2*delta_trans*delta_trans)));
      delta_trans_hat = delta_trans - 
              pf_ran_gaussian(sqrt(this->alpha3*delta_trans*delta_trans +
                                   this->alpha4*delta_rot1_noise*delta_rot1_noise +
                                   this->alpha4*delta_rot2_noise*delta_rot2_noise));
      delta_rot2_hat = angle_diff(delta_rot2,
                                  pf_ran_gaussian(sqrt(this->alpha1*delta_rot2_noise*delta_rot2_noise +
                                                       this->alpha2*delta_trans*delta_trans)));

      // Apply sampled update to particle pose
      sample->pose.v[0] += delta_trans_hat * 
              cos(sample->pose.v[2] + delta_rot1_hat);
      sample->pose.v[1] += delta_trans_hat * 
              sin(sample->pose.v[2] + delta_rot1_hat);
      sample->pose.v[2] += delta_rot1_hat + delta_rot2_hat;

    }
  }
  break;
  default:
    //printf("Error: no odom model matched, should be here\r\n");
    break;
  }

  return true;
}
