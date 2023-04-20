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

#include "ompl/multirobot/control/planners/kcbs/KCBS.h"
#include "ompl/control/planners/rrt/RRT.h"

ompl::multirobot::control::KCBS::KCBS(const ompl::multirobot::control::SpaceInformationPtr &si): 
    ompl::multirobot::base::Planner(si, "K-CBS"), llSolveTime_(1.), mergeBound_(std::numeric_limits<int>::max()), numNodesExpanded_(0)
{
    siC_ = si.get();

    // Planner::declareParam<double>("range", this, &RRT::setRange, &RRT::getRange, "0.:1.:10000.");
    // Planner::declareParam<double>("goal_bias", this, &RRT::setGoalBias, &RRT::getGoalBias, "0.:.05:1.");
    // Planner::declareParam<bool>("intermediate_states", this, &RRT::setIntermediateStates, &RRT::getIntermediateStates,
    //                             "0,1");
}

ompl::multirobot::control::KCBS::~KCBS()
{
    freeMemory();
}

void ompl::multirobot::control::KCBS::clear()
{
    base::Planner::clear();
    freeMemory();
}

void ompl::multirobot::control::KCBS::freeMemory()
{
    // free memory of every node
    // for (auto n: allNodesSet_)
        // std::cout << "here: " << n->getCost() << std::endl;
}

void ompl::multirobot::control::KCBS::setup()
{
    base::Planner::setup();
    
    // varify that all robots have the same propagation step-size
    double dt = siC_->getIndividual(0)->getPropagationStepSize();
    for (unsigned int r = 1; r < siC_->getIndividualCount(); r++) 
    {
        double dt_other = siC_->getIndividual(r)->getPropagationStepSize();
        if (dt != dt_other) 
        {
            OMPL_WARN("The propagation step size is different between planners. This may cause incorrect solutions.");
            return;
        }
    }

    // setup low-level planners
    for (unsigned int r = 0; r < siC_->getIndividualCount(); r++)
    {
        auto planner = std::make_shared<ompl::control::RRT>(siC_->getIndividual(r));
        planner->setProblemDefinition(pdef_->getIndividual(r));
        // planner->specs_.approximateSolutions = false; // TO-DO: this will throw an error but it would be nice to set this to false
        llSolvers_.push_back(planner);
    }

    // setup conflictCounter_
    for (unsigned int r1 = 0; r1 < siC_->getIndividualCount(); r1++) 
    {
        for (unsigned int r2 = r1 + 1; r2 < siC_->getIndividualCount(); r2++) 
        {
            conflictCounter_.insert({std::make_pair(r1, r2), 0});
        }
    }
}

void ompl::multirobot::control::KCBS::pushNode(const NodePtr &n)
{
    allNodesSet_.insert(n);
    boost::graph_traits<BoostGraph>::vertex_descriptor v = add_vertex(n, tree_);
    treeMap_.insert({n->getName(), v});
    if (n->getParent())
        add_edge(treeMap_[n->getParent()->getName()], treeMap_[n->getName()], tree_);
    pq_.push(n);    
}

ompl::multirobot::control::KCBS::NodePtr ompl::multirobot::control::KCBS::popNode()
{
    numNodesExpanded_ += 1;
    NodePtr n = pq_.top();
    n->setID(numNodesExpanded_);
    pq_.pop();
    return n;
}

std::vector<ompl::multirobot::control::KCBS::Conflict> ompl::multirobot::control::KCBS::findConflicts(const PlanControlPtr &plan) const
{
    // cast to desired type and interpolate the paths
    plan->interpolate();

    // get the maximum number of steps we need to simulate
    unsigned int maxSteps = 0;
    for (unsigned int r = 0; r != siC_->getIndividualCount(); r++)
    {
        if (plan->getPath(r)->getStateCount() > maxSteps)
            maxSteps = plan->getPath(r)->getStateCount();
    }

    // initialize an empty vector of conflicts
    std::vector<Conflict> confs;

    // perform a disjoint check for collision at every step in the plan, assume robots stay in place once they reach goal
    for (unsigned int k = 0; k < maxSteps; k++)
    {
        for (unsigned int r1 = 0; r1 < siC_->getIndividualCount(); r1++)
        {
            for (unsigned int r2 = r1 + 1; r2 < siC_->getIndividualCount(); r2++)
            {
                // get the states for r1 and r2 at step k
                ompl::base::State* state1 = nullptr;
                ompl::base::State* state2 = nullptr;
                if (k < plan->getPath(r1)->getStateCount())
                    state1 = plan->getPath(r1)->getState(k);
                else
                    state1 = plan->getPath(r1)->getStates().back();
                
                if (k < plan->getPath(r2)->getStateCount())
                    state2 = plan->getPath(r2)->getState(k);
                else
                    state2 = plan->getPath(r2)->getStates().back();

                // use state validity checker to perform collision check
                auto otherStatePair = std::make_pair(siC_->getIndividual(r2), state2);
                if (!siC_->getIndividual(r1)->getStateValidityChecker()->areStatesValid(state1, otherStatePair))
                {
                    Conflict c(r1, r2, k, state1, otherStatePair.second);
                    confs.push_back(c);
                    // while in collision, add all of the conflicts to confs and return
                    unsigned int step = k;
                    bool inCollision_ = true;
                    while (inCollision_ && step < maxSteps)
                    {
                        step++;
                        if (k < plan->getPath(r1)->getStateCount())
                            state1 = plan->getPath(r1)->getState(step);
                        else
                            state1 = plan->getPath(r1)->getStates().back();
                
                        if (k < plan->getPath(r2)->getStateCount())
                            otherStatePair.second = plan->getPath(r2)->getState(step);
                        else
                            otherStatePair.second = plan->getPath(r2)->getStates().back();

                        if (siC_->getIndividual(r1)->getStateValidityChecker()->areStatesValid(state1, otherStatePair))
                            inCollision_ = false;
                        else
                        {
                            Conflict cNxt(r1, r2, step, state1, otherStatePair.second);
                            confs.push_back(cNxt);
                        }
                    }
                    return confs;
                }
            }
        }
    }
    return confs;
}

void ompl::multirobot::control::KCBS::updateConflictCounter(const std::vector<Conflict> &conflicsts)
{
    for (auto &c: conflicsts)
    {
        conflictCounter_[std::make_pair(c.robots_[0], c.robots_[1])] += 1;
    }
    for (auto itr = conflictCounter_.begin(); itr != conflictCounter_.end(); itr++)
        std::cout << (itr->first).first << "," << (itr->first).second << ": " << (itr->second) << std::endl;
}

std::pair<int, int> ompl::multirobot::control::KCBS::mergeNeeded()
{
    for (auto itr = conflictCounter_.begin(); itr != conflictCounter_.end(); itr++)
    {
        if (itr->second >= mergeBound_)
            return itr->first;
    }
    return std::make_pair(-1, -1);
}

const ompl::multirobot::control::KCBS::ConstraintPtr ompl::multirobot::control::KCBS::createConstraint(const unsigned int robot, std::vector<Conflict> &confs)
{
    // create new constraint for robot that avoids other_robot
    unsigned int other_robot = (robot == 0) ? 1 : 0;
    const ConstraintPtr constraint = std::make_shared<Constraint>(robot, siC_->getIndividual(confs.front().robots_[other_robot]));
    for (auto &c: confs)
    {
        constraint->timeSteps_.push_back(c.timeStep_);
        constraint->constrainingStates_.push_back(c.states_[other_robot]);
    }
    return constraint;
}

void ompl::multirobot::control::KCBS::attemptReplan(const unsigned int robot, NodePtr &node)
{
    // collect all of the constraints on robot by traversing constraint tree back to root node
    auto nCpy = node;
    std::vector<ConstraintPtr> constraints;
    while (nCpy->getConstraint())
    {
        if (nCpy->getConstraint()->constrainedRobot_ == robot)
            constraints.push_back(nCpy->getConstraint());
        nCpy = nCpy->getParent();
    }

    // clear existing low-level planner data and existing dynamic obstacles
    siC_->getIndividual(robot)->clearDynamicObstacles();

    // add the new dynamic obstacles (the constraints)
    for (ConstraintPtr &c: constraints)
    {
        for (unsigned int k = 0; k < c->timeSteps_.size(); k++)
        {
            const ompl::base::State* state =  c->constrainingSiC_->cloneState(c->constrainingStates_[k]);
            const double time = c->timeSteps_[k] * siC_->getIndividual(robot)->getPropagationStepSize();
            siC_->getIndividual(robot)->addDynamicObstacle(time, c->constrainingSiC_, state);
        }
    }

    llSolvers_[robot]->clear();
    llSolvers_[robot]->getProblemDefinition()->clearSolutionPaths();

    // attempt to find another trajectory
    // if successful, add the new plan to node prior to exit
    llSolvers_[robot]->solve(llSolveTime_);
    if (llSolvers_[robot]->getProblemDefinition()->hasExactSolution())
    {
        PlanControlPtr new_plan = std::make_shared<PlanControl>(si_);
        auto new_path = std::make_shared<ompl::control::PathControl>(*llSolvers_[robot]->getProblemDefinition()->getSolutionPath()->as<ompl::control::PathControl>());
        for (unsigned int r = 0; r < siC_->getIndividualCount(); r++)
        {
            if (r == robot)
                new_plan->append(new_path);
            else
                new_plan->append(node->getParent()->getPlan()->getPath(r));
        }
        node->setPlan(new_plan);
        node->setCost(new_plan->length());
    }
    else
    {
        std::cout << "REPLANNING FAILED! THIS IS A TODO ITEM!" << std::endl;
    }
}

ompl::base::PlannerStatus ompl::multirobot::control::KCBS::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    checkValidity();

    OMPL_INFORM("%s: Starting planning. ", getName().c_str());

    // create root node of constraint tree with an initial solution for every individual
    PlanControlPtr initalPlan = std::make_shared<PlanControl>(si_);
    for (auto itr = llSolvers_.begin(); itr != llSolvers_.end(); itr++) 
    {
        // attempt to find an exact initial path to goal until ptc returns true
        while ( !(*itr)->getProblemDefinition()->hasExactSolution() && !ptc )
        {
            (*itr)->solve(llSolveTime_);
        }
        if (!ptc)
        {
            auto path = std::make_shared<ompl::control::PathControl>(*(*itr)->getProblemDefinition()->getSolutionPath()->as<ompl::control::PathControl>());
            initalPlan->append(path);
        }
        else
        {
            break;
        }
    }

    // create root node
    NodePtr root = std::make_shared<Node>(initalPlan);
    pushNode(root);

    NodePtr solution = nullptr;
    bool solved = false;

    while (!ptc && !pq_.empty())
    {
        // get the best unexplored node in the constraint tree
        const NodePtr currentNode = popNode();

        // TODO: if currentNode has no plan, then try to find one again
        if (currentNode->getCost() < 0)
        {
            std::cout << "ENTERED REPLANNING LOGIC! THIS IS A TODO ITEM!" << std::endl;
        }
        else
        {
            // find conflicts in the current plan
            std::vector<Conflict> confs = findConflicts(currentNode->getPlan());
            for (auto &c: confs)
            {
                std::cout << "Conflict between " << c.robots_[0] << " and " << c.robots_[1] << " at time step " << c.timeStep_ << std::endl;
            }

            // if no conflicts were found, return as solution
            if (confs.empty()) {
                solution = currentNode;
                break;
            }

            // update the conflictCounter_;
            updateConflictCounter(confs);

            // TODO: if a merge is needed, then merge and restart
            if (mergeNeeded() != std::make_pair(-1, -1))
            {
                std::cout << "ENTERED MERGE LOGIC! THIS IS A TODO ITEM!" << std::endl;
                // return mergeAndRestart();
            }

            // create a constraint for every agent in confs
            // for example, if conflicts occur between robots 0 and 2 for the interval dt=[615, 665] then
            // constraint1 is given to robot 0 which forces it to avoid the states of robot 2 for all steps inside dt=[615, 665]
            // constraint2 is given to robot 2 which forces it to avoid the states of robot 0 for all steps inside dt=[615, 665]
            // then, replan for robots 0 and 2 after adding the constraints as dynamic obstacles

            for (unsigned int r = 0; r < 2; r++)
            {
                // create a new constraint
                const ConstraintPtr new_constraint = createConstraint(r, confs);

                // create a new node to house the new constraint, also assign a parent
                NodePtr nxtNode = std::make_shared<Node>();
                nxtNode->setParent(currentNode);
                nxtNode->setConstraint(new_constraint);

                attemptReplan(r, nxtNode);
                std::cout << "here " << nxtNode->getCost() << std::endl;
                pushNode(nxtNode);
            }
        }
    }
    if (solution == nullptr) 
    {
        OMPL_INFORM("%s: No solution found.", getName().c_str());
        return {solved, false};
    }
    else 
    {
        solved = true;
        OMPL_INFORM("%s: Found Solution!", getName().c_str());
        pdef_->addSolutionPlan(solution->getPlan(), false, false, getName());
        OMPL_INFORM("%s: Planning Complete.", getName().c_str());
        return {solved, false};
    }
}

void ompl::multirobot::control::KCBS::getPlannerData(ompl::base::PlannerData &data) const
{
    std::cout << "getPlannerData() called" << std::endl;
}
