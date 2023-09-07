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

#include "ompl/multirobot/control/planners/pp/PP.h"
#include "ompl/control/planners/rrt/RRT.h"


ompl::multirobot::control::PP::PP(const ompl::multirobot::control::SpaceInformationPtr &si, ompl::base::PlannerPtr solver)
  : ompl::multirobot::base::Planner(si, "PP")
{
    siC_ = si.get();
}

ompl::multirobot::control::PP::~PP()
{
    freeMemory();
}

void ompl::multirobot::control::PP::clear()
{
    base::Planner::clear();
    freeMemory();
}

void ompl::multirobot::control::PP::setup()
{
    base::Planner::setup();
    // setup low-level planners
    if (!siC_->hasPlannerAllocator())
        throw Exception(getName().c_str(), "No PlannerAllocator provided!");

    llSolvers_.resize(siC_->getIndividualCount());
    for (unsigned int r = 0; r < siC_->getIndividualCount(); r++)
    {
        llSolvers_[r] = siC_->allocatePlannerForIndividual(r);
        llSolvers_[r]->setProblemDefinition(pdef_->getIndividual(r));
        // llSolvers_[r]->specs_.approximateSolutions = false; // TO-DO: this will throw an error but it would be nice to set this to false
    }
}

void ompl::multirobot::control::PP::freeMemory()
{
    for (unsigned int r = 0; r < siC_->getIndividualCount(); r++)
    {
        llSolvers_[r]->clear();
        siC_->getIndividual(r)->clearDynamicObstacles();
    }
}

void ompl::multirobot::control::PP::addPathAsDynamicObstacles(const unsigned int individual, const ompl::control::PathControlPtr &path)
{
    path->interpolate();
    auto states = path->getStates();
    auto durs = path->getControlDurations();
    for (auto r = individual + 1; r < si_->getIndividualCount(); r++)
    {
        double time = 0.;
        // add the path as dynamic obstacles
        for (unsigned int step = 0; step < path->getStateCount(); step++)
        {
            auto state =  siC_->getIndividual(individual)->cloneState(states[step]);
            si_->addDynamicObstacleForIndividual(r, individual, state, time);
            time += durs[step];
        }
        // add the robot staying still at goal as dynamic obstacle
        const unsigned int max_steps = 100000;
        for (unsigned int s = 0; s < max_steps; s++)
        {
            auto goal_state =  siC_->getIndividual(individual)->cloneState(states.back());
            time += durs.back();
            si_->addDynamicObstacleForIndividual(r, individual, goal_state, time);
        }
    }
}

ompl::base::PlannerStatus ompl::multirobot::control::PP::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    auto plan(std::make_shared<PlanControl>(si_));
    for (unsigned int r = 0; r < si_->getIndividualCount(); ++r)
    {
        /* plan for individual r while treating individuals 1, ..., r-1 as dynamic obstacles 
            Note: It is theoretically possible to use any planner from ompl::control. We only use RRT for now.
        */
        ompl::base::PlannerStatus solved = llSolvers_[r]->solve(ptc);
        if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION)
        {
            // add the path to the plan
            auto path = std::make_shared<ompl::control::PathControl>(*llSolvers_[r]->getProblemDefinition()->getSolutionPath()->as<ompl::control::PathControl>());
            addPathAsDynamicObstacles(r, path);
            plan->as<PlanControl>()->append(path);
        }
        else
        {
            // failed to find a path for individual r -- return approximate solution
            pdef_->addSolutionPlan(plan, true, si_->getIndividualCount() - r, getName());
            return {true, true};
        }
    }
    // add plan to problem definition
    pdef_->addSolutionPlan(plan, false, false, getName());
    return {true, false};
}

void ompl::multirobot::control::PP::getPlannerData(ompl::base::PlannerData &data) const
{
    std::cout << "getPlannerData() called" << std::endl;
}
