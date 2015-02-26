/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Caleb Voss and Wilson Beebe
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

/* Authors: Caleb Voss, Wilson Beebe */

#ifndef V_F_MECHANICAL_WORK_OPTIMIZATION_OBJECTIVE_
#define V_F_MECHANICAL_WORK_OPTIMIZATION_OBJECTIVE_

#include "ompl/base/objectives/MechanicalWorkOptimizationObjective.h"
#include "ompl/geometric/planners/rrt/VFRRT.h"

namespace ompl
{
    namespace base
    {

        /**
         * Optimization objective that computes mechanical work between two states.
         */
        class VFMechanicalWorkOptimizationObjective : public ompl::base::MechanicalWorkOptimizationObjective
        {
    
        public:
    
            /** Constructor. */
            VFMechanicalWorkOptimizationObjective(const ompl::base::SpaceInformationPtr &si, const geometric::VFRRT::VectorField *vf)
                : ompl::base::MechanicalWorkOptimizationObjective(si), vf(*vf), sstate1(si_), sstate2(si_), d(sstate1.reals().size()), qprime(d)
            {
            }
    
            /** Assume we can always do better. */
            bool isSatisfied(ompl::base::Cost c) const
            {
                return false;
            }
    
            /** Compute mechanical work between two states. */
            ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const
            {
                // Per equation 7 in the paper
                Eigen::VectorXd f = vf(s2);
                sstate1 = s1;
                sstate2 = s2;
                for (int i = 0; i < d; i++)
                {
                    qprime[i] = sstate2[i] - sstate1[i];
                }
                // Don't included negative work
                double positiveCostAccrued = std::max((-f).dot(qprime), 0.0);
                return ompl::base::Cost(positiveCostAccrued + pathLengthWeight_*si_->distance(s1,s2));
            }

            bool isSymmetric(void) const
            {
                return false;
            }
    
        private:
    
            /** VectorField associated with the space. */
            const geometric::VFRRT::VectorField &vf;
    
            /** Variables used in computation that we keep around to save on allocations. */
            mutable ompl::base::ScopedState<> sstate1;
            mutable ompl::base::ScopedState<> sstate2;
            const int d;
            mutable Eigen::VectorXd qprime;
    
        };
    }
}

#endif
