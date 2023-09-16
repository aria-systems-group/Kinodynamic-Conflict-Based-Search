/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Justin Kottinger */

#include "ompl/base/StateValidityChecker.h"
#include "ompl/base/SpaceInformation.h"

bool ompl::base::StateValidityChecker::isValid(const State *state, const double time)
{
    if (dynObstacles_.empty())
        return isValid(state);
    else
    {
        if (isValid(state))
        {
            int t_key = std::round(time * scalingFactor_);
            auto obsAtTime = dynObstacles_.find(t_key);
            if (obsAtTime != dynObstacles_.end())
            {
                for (auto st = obsAtTime->second.begin(); st != obsAtTime->second.end(); st++)
                {
                    if (!areStatesValid(state, *st))
                        return false;
                }
                return true;
            }
            else
                return true;
        }
        else
            return false;
    }
}

void ompl::base::StateValidityChecker::clearDynamicObstacles()
{
    for (auto t_itr = dynObstacles_.begin(); t_itr != dynObstacles_.end(); t_itr++)
    {
        for (auto st_itr = (t_itr->second).begin(); st_itr != (t_itr->second).end(); st_itr++)
        {
            st_itr->first->freeState(st_itr->second);
        }
    }
    dynObstacles_.clear();
}