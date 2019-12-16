/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DWB_CRITICS_CRITIC_CFG_H_
#define DWB_CRITICS_CRITIC_CFG_H_


#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>


namespace dwb_critics
{

/**
 * @class CriticCfg
 * @brief Template to make critics dynamically reconfigerable
 */

template <typename T>
class CriticCfg
{
public:
  CriticCfg(): initialized_(false){}
  void init(ros::NodeHandle &critic_nh_)
  {
    server_.reset(new dynamic_reconfigure::Server<T>(mutex_, critic_nh_));
    server_->setCallback([this](const T& cfg, uint32_t level) { reconfigure(cfg, level); });
    initialized_ = true;
  }
  T cfg() const
  {
    boost::recursive_mutex::scoped_lock lock(mutex_);
   return cfg_;
  }
  bool isInitialized(){return initialized_;}
private:
  T cfg_;
  bool initialized_;
  std::unique_ptr<dynamic_reconfigure::Server<T>> server_;
  mutable boost::recursive_mutex mutex_;
  void reconfigure(const T& cfg, uint32_t)
  {
    boost::recursive_mutex::scoped_lock lock(mutex_);
    cfg_ = cfg;
  }
};

} /* namespace dwb_critics */
#endif /* DWB_CRITICS_CRITIC_CFG_H_ */
