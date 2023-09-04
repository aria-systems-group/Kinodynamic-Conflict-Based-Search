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

ompl::multirobot::control::KCBS::KCBS(const ompl::multirobot::control::SpaceInformationPtr &si): 
    ompl::multirobot::base::Planner(si, "K-CBS"), llSolveTime_(1.), mergeBound_(std::numeric_limits<int>::max()), numNodesExpanded_(0), numApproxSolutions_(0), rootSolveTime_(-1)
{
    siC_ = si.get();

    Planner::declareParam<double>("low_level_solve_time", this, &KCBS::setLowLevelSolveTime, &KCBS::getLowLevelSolveTime, "0.:1.:10000000.");
    Planner::declareParam<double>("merge_bound", this, &KCBS::setMergeBound, &KCBS::getMergeBound, "0:1:10000000");
}

ompl::multirobot::control::KCBS::~KCBS()
{
    freeMemory();
}

void ompl::multirobot::control::KCBS::clear()
{
    base::Planner::clear();
    freeMemory();
    numNodesExpanded_ = 0;
    numApproxSolutions_ = 0;
    rootSolveTime_ = -1;
}

void ompl::multirobot::control::KCBS::freeMemory()
{
    // clear the priority queue
    while (!pq_.empty())
    {
        NodePtr n = pq_.top();
        pq_.pop();
        n.reset();
    }
    pq_ = std::priority_queue<NodePtr, std::vector<NodePtr>, NodeCompare>();
    // free memory of every node
    for (auto n: allNodesSet_)
        n.reset();
    allNodesSet_.clear();
    // free memory of every low-level solver
    for (auto &p: llSolvers_)
        p.reset();
    // free memory of all the dynamic obstaces
    for (unsigned int r = 0; r < siC_->getIndividualCount(); r++)
        siC_->getIndividual(r)->clearDynamicObstacles();
    // free memory of the merged planner (if it exists)
    if (mergerPlanner_)
        mergerPlanner_.reset();
    // reset conflict counter
    conflictCounter_.clear();
    // clear the boost graph
    tree_.clear();
    treeMap_.clear();
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

    if (!siC_->hasPlannerAllocator())
        throw Exception(getName().c_str(), "No PlannerAllocator provided!");

    // setup low-level planners
    llSolvers_.resize(siC_->getIndividualCount());
    for (unsigned int r = 0; r < siC_->getIndividualCount(); r++)
    {
        llSolvers_[r] = siC_->allocatePlannerForIndividual(r);
        llSolvers_[r]->setProblemDefinition(pdef_->getIndividual(r));
        // llSolvers_[r]->specs_.approximateSolutions = false; // TO-DO: this will throw an error but it would be nice to set this to false
    }

    // setup conflictCounter_
    for (unsigned int r1 = 0; r1 < siC_->getIndividualCount(); r1++) 
    {
        for (unsigned int r2 = r1 + 1; r2 < siC_->getIndividualCount(); r2++) 
        {
            conflictCounter_.insert({std::make_pair(r1, r2), 0});
        }
    }

    // check if merger is set
    if (!siC_->getSystemMerger())
        OMPL_WARN("%s: SystemMerger not set! Planner will fail if mergeBound_ is triggered.", getName().c_str());
}

void ompl::multirobot::control::KCBS::pushNode(const NodePtr &n)
{
    // add a node to the tree_ and pq_
    allNodesSet_.insert(n);
    if (n->getID() == -1)
    {
        boost::graph_traits<BoostGraph>::vertex_descriptor v = add_vertex(n, tree_);
        treeMap_.insert({n->getName(), v});
        if (n->getParent())
            add_edge(treeMap_[n->getParent()->getName()], treeMap_[n->getName()], tree_);
    }
    pq_.push(n);    
}

ompl::multirobot::control::KCBS::NodePtr ompl::multirobot::control::KCBS::popNode()
{
    // pop a node and assign it an ID
    NodePtr n = pq_.top();
    if (n->getID() == -1)
    {
        numNodesExpanded_ += 1;
        n->setID(numNodesExpanded_);
    }
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
                        if (step < plan->getPath(r1)->getStateCount())
                            state1 = plan->getPath(r1)->getState(step);
                        else
                            state1 = plan->getPath(r1)->getStates().back();
                
                        if (step < plan->getPath(r2)->getStateCount())
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
    // update the conflictCounter map with the newly found conflicts.
    for (auto &c: conflicsts)
    {
        conflictCounter_[std::make_pair(c.robots_[0], c.robots_[1])] += 1;
    }
}

std::pair<int, int> ompl::multirobot::control::KCBS::mergeNeeded()
{
    // iterate through all of the possible merge pairs and check if any pairs have too many conflicts. If so, return the pair. Otherwise, return (-1, -1)
    for (auto itr = conflictCounter_.begin(); itr != conflictCounter_.end(); itr++)
    {
        if (itr->second > mergeBound_)
            return itr->first;
    }
    return std::make_pair(-1, -1);
}

const ompl::multirobot::control::KCBS::ConstraintPtr ompl::multirobot::control::KCBS::createConstraint(const unsigned int index, std::vector<Conflict> &confs)
{
    // create new constraint for robot that avoids other_robot
    unsigned int other_index = (index == 0) ? 1 : 0;
    const ConstraintPtr constraint = std::make_shared<Constraint>(confs.front().robots_[index], siC_->getIndividual(confs.front().robots_[other_index]));
    for (auto &c: confs)
    {
        constraint->timeSteps_.push_back(c.timeStep_);
        constraint->constrainingStates_.push_back(c.states_[other_index]);
    }
    return constraint;
}

void ompl::multirobot::control::KCBS::attemptReplan(const unsigned int robot, NodePtr node, const bool retry)
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
            ompl::base::State* state =  c->constrainingSiC_->cloneState(c->constrainingStates_[k]);
            const double time = c->timeSteps_[k] * siC_->getIndividual(robot)->getPropagationStepSize();
            siC_->getIndividual(robot)->addDynamicObstacle(time, c->constrainingSiC_, state);
        }
    }

    llSolvers_[robot]->clear();
    llSolvers_[robot]->getProblemDefinition()->clearSolutionPaths();

    // attempt to find another trajectory
    // if successful, add the new plan to node prior to exit
    ompl::base::PlannerStatus solved;
    if (retry)
        solved = node->getLowLevelSolver()->solve(llSolveTime_);
    else
        solved = llSolvers_[robot]->solve(llSolveTime_);
    
    if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION)
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
        numApproxSolutions_ += 1;
        // save the planner prior to exit only if planner not already saved
        if (!retry)
        {
            // need to save the existing low-level solver to the node and create a new one for the rest of the system
            node->setLowLevelSolver(llSolvers_[robot]);
            llSolvers_[robot] = siC_->allocatePlannerForIndividual(robot);
            llSolvers_[robot]->setProblemDefinition(pdef_->getIndividual(robot));
        }
    }
    pushNode(node);
}

void ompl::multirobot::control::KCBS::parallelRootSolutionHelper(PlanControlPtr plan, unsigned int startIdx, unsigned int endIdx)
{
    for (unsigned int i = startIdx; i < endIdx; i++)
    {
        while (!llSolvers_[i]->getProblemDefinition()->hasExactSolution())
            llSolvers_[i]->solve(llSolveTime_);
        auto path = std::make_shared<ompl::control::PathControl>(*llSolvers_[i]->getProblemDefinition()->getSolutionPath()->as<ompl::control::PathControl>());
        plan->replace(i, path);
    }
}

std::vector<unsigned int> ompl::multirobot::control::KCBS::split(const unsigned int jobs, const unsigned int workers)
{
    std::vector<unsigned int> num_jobs_per_workers;
    // If x % n == 0 then the minimum
    // difference is 0 and all
    // numbers are x / n
    if (jobs % workers == 0)
    {
        for(unsigned int i=0; i < workers; i++)
            num_jobs_per_workers.push_back(jobs / workers);
    }
    else
    {
 
        // up to n-(x % n) the values will be x / n
        // after that, the values will be x / n + 1
        unsigned int zp = workers - (jobs % workers);
        unsigned int pp = jobs / workers;
        for(unsigned int i=0; i < workers; i++)
        {
            if (i >= zp)
                num_jobs_per_workers.push_back(pp + 1);
            else
                num_jobs_per_workers.push_back(pp);
        }
    }
    return num_jobs_per_workers;
}

void ompl::multirobot::control::KCBS::parallelRootSolution(PlanControlPtr plan)
{
    // Create an array of threads.
    std::vector<std::thread> threads;

    // Divide the loop into equal segments.
    const unsigned int num_workers = std::min(siC_->getIndividualCount(), numThreads_); // check that there is a way for the system to tell us how many threads there are

    auto jobs_for_worker = split(siC_->getIndividualCount(), num_workers);

    OMPL_INFORM("%s: Assigned %d workers to plan paths for %d robots.", getName().c_str(), num_workers, siC_->getIndividualCount());

    // Create a thread for each segment.
    unsigned int start = 0;
    unsigned int end = jobs_for_worker[0];
    for (unsigned int i = 0; i < num_workers; i++)
    {
        threads.push_back(std::thread(&ompl::multirobot::control::KCBS::parallelRootSolutionHelper, 
            this, plan, start, end));

        // update start and end 
        start = end;
        end += jobs_for_worker[i + 1];
    }

    // Join all of the threads.
    for (auto& thread : threads) {
        thread.join();
    }
}

void ompl::multirobot::control::KCBS::parallelNodeExpansion(NodePtr& solution, std::vector<unsigned int>& resevered)
{
    if (solution) // another thread beat this one to a solution
        return;
    
    // get the best unexplored node in the constraint tree
    NodePtr currentNode = popNode();

    // if current node has not plan, then attempt to find one again
    if (currentNode->getCost() == std::numeric_limits<double>::max())
    {
        // check it constrained robot is already reserved by someone else
        auto reserved_itr = std::find(resevered.begin(), resevered.end(), currentNode->getConstraint()->constrainedRobot_);
        if (reserved_itr == resevered.end()) // not reserved, proceed to plan for it
        {
            // use existing tree to attempt a replan
            // reserve robot
            resevered.push_back(currentNode->getConstraint()->constrainedRobot_);
            // attempt replan    
            attemptReplan(currentNode->getConstraint()->constrainedRobot_, currentNode, true);
        }
        else // already reserved, just push to queue for later use
            pushNode(currentNode);
    }
    else if (currentNode->getCost() == 0) // node was reserved last time we tried to find a plan
    {
        // check it constrained robot is already reserved by someone else
        auto reserved_itr = std::find(resevered.begin(), resevered.end(), currentNode->getConstraint()->constrainedRobot_);
        if (reserved_itr == resevered.end()) // not reserved, proceed to plan for it
        {
            // reserve robot
            resevered.push_back(currentNode->getConstraint()->constrainedRobot_);
            // attempt replan
            attemptReplan(currentNode->getConstraint()->constrainedRobot_, currentNode, false);
        }
        else
            pushNode(currentNode);
    }
    else
    {
        // find conflicts in the current plan
        std::vector<Conflict> confs = findConflicts(currentNode->getPlan());

        // if no conflicts were found, return as solution
        if (confs.empty()) {
            solution = currentNode;
            return;
        }

        // for debugging
        // for (auto &c: confs)
        // {
        //     std::cout << "conflict between " << c.robots_[0] << " and " << c.robots_[1] << " at time " << c.timeStep_ << " with states " << c.states_[0] << " and " << c.states_[1]  << std::endl;
        // }

        // update the conflictCounter_;
        updateConflictCounter(confs);

        // FIXME: need to keep the merge logic somehow
        // if merge is needed, then merge and restart
        // std::pair<int, int> merge_indices = mergeNeeded();
        // if (merge_indices != std::make_pair(-1, -1))
        // {
        //     if (!siC_->getSystemMerger())
        //     {
        //         OMPL_INFORM("%s: Merge was triggered but no SystemMerger was provided. UNable to expand node.", getName().c_str());
        //         return; // FIXME: need to notify that expansion failed
        //     }
        //     else
        //     {
        //         OMPL_INFORM("%s: Merge was triggered. Composing individuals %d and %d.", getName().c_str(), merge_indices.first, merge_indices.second);
        //         std::pair<const SpaceInformationPtr, const ompl::multirobot::base::ProblemDefinitionPtr> new_defs = siC_->merge(merge_indices.first, merge_indices.second);
        //         if (new_defs.first && new_defs.second)
        //         {
        //             mergerPlanner_ = std::make_shared<KCBS>(new_defs.first);
        //             mergerPlanner_->setProblemDefinition(new_defs.second);
        //             return mergerPlanner_->solve(ptc);
        //         }
        //         else
        //         {
        //             OMPL_INFORM("%s: SystemMerge was triggered but failed. Unable to expand node.", getName().c_str());
        //             return; // FIXME: need to notify that expansion failed
        //         }
                
        //     }
        // }

        // create a constraint for every agent in confs
        // for example, if conflicts occur between robots 0 and 2 for the interval dt=[615, 665] then
        // constraint1 is given to robot 0 which forces it to avoid the states of robot 2 for all steps inside dt=[615, 665]
        // constraint2 is given to robot 2 which forces it to avoid the states of robot 0 for all steps inside dt=[615, 665]
        // then, replan for robots 0 and 2 after adding the constraints as dynamic obstacles

        std::vector<std::thread> threads;

        for (unsigned int r = 0; r < 2; r++)
        {
            // create a new constraint
            const ConstraintPtr new_constraint = createConstraint(r, confs);

            // create a new node to house the new constraint, also assign a parent
            NodePtr nxtNode = std::make_shared<Node>();
            nxtNode->setParent(currentNode);
            nxtNode->setConstraint(new_constraint);

            // check it constrained robot is already reserved by someone else
            auto reserved_itr = std::find(resevered.begin(), resevered.end(), new_constraint->constrainedRobot_);
            if (reserved_itr == resevered.end()) // not reserved, proceed to plan for it
            {
                // reserve robot
                resevered.push_back(new_constraint->constrainedRobot_);
                // attempt to replan and push node to priority queue
                threads.push_back(std::thread(&ompl::multirobot::control::KCBS::attemptReplan, 
                    this, new_constraint->constrainedRobot_, nxtNode, false));
            }
            else // already reserved, just push to queue for later use
            {
                // make cost 0 s.t. we can expand from it next time (hopefully)
                nxtNode->setCost(0);
                pushNode(nxtNode);
            }   
        }
        // Join all of the threads.
        for (auto& thread : threads) {
            thread.join();
        }
    }
}

ompl::base::PlannerStatus ompl::multirobot::control::KCBS::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    checkValidity();

    OMPL_INFORM("%s: Starting planning. ", getName().c_str());

    // start the timer for root solution
    auto start = std::chrono::high_resolution_clock::now();
    // initialize the initial plan and fill it with the required size
    PlanControlPtr initalPlan = std::make_shared<PlanControl>(si_);
    for (unsigned int i = 0; i < siC_->getIndividualCount(); i++)
    {
        auto dummy_path = std::make_shared<ompl::control::PathControl>(siC_->getIndividual(i));
        initalPlan->append(dummy_path);
    }

    // get the initial solution
    parallelRootSolution(initalPlan);
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    double duration_s = (duration_ms.count() * 0.001);
    rootSolveTime_ = duration_s;

    // create root node
    NodePtr root = std::make_shared<Node>(initalPlan);
    pushNode(root);

    NodePtr solution = nullptr;
    bool solved = false;

    std::vector<unsigned int> resevered;

    while (!ptc && !pq_.empty())
    {
        // use multiple threads to expand multiple nodes at once
        const unsigned int numNodesInQueue = pq_.size();
        const unsigned int test = std::floor(numThreads_ / 2);
        const unsigned int numNodesSelect = std::min(numNodesInQueue, test);
        std::vector<std::thread> threads;
        for (unsigned int i = 0; i < numNodesSelect; i++)
        {
            threads.push_back(std::thread(&ompl::multirobot::control::KCBS::parallelNodeExpansion, this, std::ref(solution), std::ref(resevered)));
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // wait 0.1 seconds s.t. reserved is updated properly
        }
        // Join all of the threads.
        for (auto& thread : threads) {
            thread.join();
        }
        resevered.clear();
        if (solution)
            break;
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
