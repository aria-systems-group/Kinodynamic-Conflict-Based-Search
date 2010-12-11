/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Ioan Sucan */

#ifndef OMPL_CONTROL_CONTROL_ALLOCATOR_
#define OMPL_CONTROL_CONTROL_ALLOCATOR_

#include "ompl/control/ControlManifold.h"
#include <boost/noncopyable.hpp>
#include <boost/thread.hpp>
#include <stack>
#include <map>

namespace ompl
{
    namespace control
    {

        /** \brief Definition of an control allocator. This reuses
            memory for controls and frees them only when the instance is
            destroyed. */
        class ControlAllocator : private boost::noncopyable
        {
        public:

            /** \brief Constructor */
            ControlAllocator(const ControlManifoldPtr &manifold) : manifold_(manifold)
            {
            }

            ~ControlAllocator(void)
            {
                clear();
            }

            /** \brief Allocate a control from the specified manifold */
            Control* allocControl(void) const;

            /** \brief Free the memory of the allocated control */
            void freeControl(Control *control) const;

            /** \brief Clear all the allocated memory */
            void clear(void);

            /** \brief Return the number of pre-allocated controls */
            unsigned int size(void) const;

        private:

            typedef std::map< boost::thread::id, std::stack<Control*> > StorageType;

            ControlManifoldPtr   manifold_;
            mutable StorageType  storage_;
            mutable boost::mutex lock_;

        };

    }
}

#endif
