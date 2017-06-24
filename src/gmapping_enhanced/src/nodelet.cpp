/*
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 * 
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/*
 * Copyright (c) 2017, Beijing SmartConn Tech. Co.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* Author: Brian Gerkey */
/* Modified by: Wei Ren */

#include <memory>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "gmapping_enhanced/slam_gmapping.h"

class SlamGMappingNodelet: public nodelet::Nodelet
{
private:  
    std::unique_ptr<SlamGMapping> m_sgm;

public:
    SlamGMappingNodelet(){}

    ~SlamGMappingNodelet(){}
  
    virtual void onInit()
    {
      NODELET_INFO("Initialising Slam GMapping nodelet.");
      m_sgm.reset(new SlamGMapping(getNodeHandle(), getPrivateNodeHandle()));
      NODELET_INFO("Starting live SLAM.");
      m_sgm->StartLiveSlam();
    }
};

PLUGINLIB_EXPORT_CLASS(SlamGMappingNodelet, nodelet::Nodelet)