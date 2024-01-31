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

#ifndef OMPL_MULTIROBOT_BASE_PLAN_VALIDITY_CHECKER_
#define OMPL_MULTIROBOT_BASE_PLAN_VALIDITY_CHECKER_

#include "ompl/multirobot/control/SpaceInformation.h"
#include "ompl/multirobot/base/ProblemDefinition.h"

namespace ompl
{
    namespace multirobot
    {
        namespace control
        {
            /// @cond IGNORE
            /** \brief Forward declaration of ompl::base::SpaceInformation */
            OMPL_CLASS_FORWARD(SpaceInformation);
            /// @endcond
            
            /// @cond IGNORE
            /** \brief Forward declaration of ompl::base::SpaceInformation */
            OMPL_CLASS_FORWARD(ProblemDefinition);
            /// @endcond

            /// @cond IGNORE
            /** \brief Forward declaration of ompl::multirobot::base::PlanValidityChecker */
            OMPL_CLASS_FORWARD(SystemMerger);
            /// @endcond

            class SystemMerger
            {
            public:
                /** \brief Constructor */
                SystemMerger(const SpaceInformationPtr &si, const base::ProblemDefinitionPtr &pdef) : si_(si), pdef_(pdef)
                {
                }

                /** \brief Given two individuals (index1 and index2), compose them together and return a different ompl::multirobot::SpaceInformation object */
                virtual std::pair<const SpaceInformationPtr, const ompl::multirobot::base::ProblemDefinitionPtr> merge(const int index1, const int index2) const = 0;

            protected:
                /** \brief The instance of space information this state validity checker operates on */
                const SpaceInformationPtr si_;
                const base::ProblemDefinitionPtr pdef_;

            };
        }
    }
}

#endif
