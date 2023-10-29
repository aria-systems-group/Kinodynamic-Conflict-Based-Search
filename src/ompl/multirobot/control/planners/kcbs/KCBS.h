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

#ifndef OMPL_MULTIROBOT_CONTROL_PLANNERS_KCBS_
#define OMPL_MULTIROBOT_CONTROL_PLANNERS_KCBS_

#include "ompl/multirobot/control/planners/PlannerIncludes.h"
#include "ompl/control/PlannerData.h"
#include "ompl/util/Exception.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/property_map/transform_value_property_map.hpp>
#include <random>
#include <utility>
#include <queue>
#include <map>
#include <unordered_set>
#include <chrono>
#include <thread>


namespace ompl
{
    namespace multirobot
    {
        namespace control
        {
            /// @cond IGNORE
            /** \brief Forward declaration of ompl::base::SpaceInformation */
            OMPL_CLASS_FORWARD(KCBS);
            /// @endcond

            /**
            @anchor cKCBS
            @par Short description
            Kinodynamic Conflict-Based Search is a two level search consisting of
            a high-level constraint tree search and a low level motion planner search. 
            The algorithm works by planning for all individuals seperately and resolving 
            system collisions via constraints on the low-level solver. Please see the paper 
            for details.

            J. Kottinger, S. Almagor and M. Lahijanian, "Conflict-Based Search 
            for Multi-Robot Motion Planning with Kinodynamic Constraints," 2022 
            IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 
            Kyoto, Japan, 2022, pp. 13494-13499, doi: 10.1109/IROS47612.2022.9982018.

            */

            /** \brief Kinodynamic Conflict-Based Search Algorithm */
            class KCBS : public multirobot::base::Planner
            {
            public:
                /** \brief Constructor */
                KCBS(const ompl::multirobot::control::SpaceInformationPtr &si);

                /** \brief Destructor */
                ~KCBS() override;

                void getPlannerData(ompl::base::PlannerData &data) const override;

                ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override;

                void clear() override;

                void setup() override;

                /** Set the merge bound. */
                void setMergeBound(unsigned int b) {mergeBound_ = b;};

                unsigned int getMergeBound() const {return mergeBound_;};

                unsigned int getNumberOfNodesExpanded() const {return numNodesExpanded_;};

                unsigned int getNumberOfApproximateSolutions() const {return numApproxSolutions_;};

                double getRootSolveTime() const {return rootSolveTime_;};

                /** Set the low-level solve time. */
                void setLowLevelSolveTime(const double t) {llSolveTime_ = t;};

                /** Get the low-level solve time. */
                double getLowLevelSolveTime() const {return llSolveTime_;};

                /** Setter function for the number of workers (threads) */
                void setNumThreads(const unsigned int value) {numThreads_ = value;};

                /** \brief Output the constraint tree in graphViz format. */
                void printConstraintTree(std::ostream &out)
                {
                    boost::dynamic_properties dp;
                    auto bundle = get(boost::vertex_bundle, tree_);
                    // dp.property("node_id", boost::make_transform_value_property_map(std::mem_fn(&Node::getName), bundle));
                    dp.property("node_id", boost::make_transform_value_property_map(std::mem_fn(&Node::getLabel), bundle));
                    write_graphviz_dp(out, tree_, dp);
                }

            protected:

                /** \brief A conflict occurs when robot1_ located at state1_ and robot2_ located at state2_ collide at timeStep_ */
                struct Conflict
                {
                    Conflict(unsigned int r1, unsigned int r2, unsigned int step, ompl::base::State* st1, ompl::base::State* st2):
                        robots_{r1, r2}, states_{st1, st2}, timeStep_(step) {}
                    // copy constructor
                    Conflict(const Conflict &other)
                    {
                        this->robots_[0] = other.robots_[0];
                        this->robots_[1] = other.robots_[1];
                        this->states_[0] = other.states_[0];
                        this->states_[1] = other.states_[1];
                        this->timeStep_ = other.timeStep_;
                    }
                    unsigned int robots_[2];
                    ompl::base::State* states_[2];
                    unsigned int timeStep_;
                };

                /// @cond IGNORE
                /** \brief Forward declaration of ompl::base::Planner */
                OMPL_CLASS_FORWARD(Constraint);
                /// @endcond

                // A collection of dynamic obstacles for a robot
                struct Constraint
                {
                    Constraint(int r): constrainedRobot_(r), constrainingSiC_(nullptr), timeSteps_(), constrainingStates_() {}
                    Constraint(int r, ompl::control::SpaceInformationPtr otherSiC): 
                        constrainedRobot_(r), constrainingSiC_(otherSiC), timeSteps_(), constrainingStates_() {}
                    ~Constraint()
                    {
                        constrainingSiC_.reset();
                        // for (auto &st: constrainingStates_)
                        //     constrainingSiC_->freeState(st);
                    }
                    unsigned int constrainedRobot_;
                    ompl::control::SpaceInformationPtr constrainingSiC_;
                    std::vector<int> timeSteps_;
                    std::vector<ompl::base::State*> constrainingStates_;
                };

                /// @cond IGNORE
                /** \brief Forward declaration of ompl::base::Planner */
                OMPL_CLASS_FORWARD(Node);
                /// @endcond

                /** \brief The node of the high-level constraint tree. */
                class Node
                {
                public:
                    Node(): plan_(nullptr), parent_(nullptr), constraint_(nullptr), 
                        cost_(std::numeric_limits<int>::max()), name_(generateRandomName()), id_(-1), llSolver_(nullptr) {};

                    Node(const PlanControlPtr plan): plan_(plan), parent_(nullptr), constraint_(nullptr), 
                        cost_(std::numeric_limits<int>::max()), name_(generateRandomName()), id_(-1), llSolver_(nullptr) {};

                    ~Node()
                    {
                        plan_.reset();
                        parent_.reset();
                        constraint_.reset();
                        if (llSolver_)
                            llSolver_.reset();

                    }

                    void setPlan(const PlanControlPtr &plan) {plan_ = plan;};

                    void setParent(const NodePtr &p) {parent_ = p;};

                    void setConstraint(const ConstraintPtr &c) {constraint_ = c;};

                    void setCost(const int c) 
                    {
                        cost_ = c;
                    };

                    void setID(const int id) {id_ = id;};

                    void setLowLevelSolver(ompl::base::PlannerPtr &planner) {llSolver_ = planner;};

                    void setConflicts(std::vector<Conflict> c) {conflicts_ = c;};

                    const PlanControlPtr &getPlan() const {return plan_;};

                    NodePtr &getParent() {return parent_;};

                    const ConstraintPtr &getConstraint() const {return constraint_;};

                    int getCost() const {return cost_;};

                    std::string getName() const { return name_; };

                    int getID() const {return id_;};

                    const std::vector<Conflict> getConflicts() const {return conflicts_;};

                    std::string getLabel()
                    {
                        std::string costString;
                        if (cost_ < std::numeric_limits<int>::max())
                            costString = std::to_string(cost_);
                        else
                            costString = "-1";

                        std::string contstraintString;
                        if (constraint_)
                        {
                            contstraintString = "{" + std::to_string(constraint_->constrainedRobot_) + 
                                                ", [" + std::to_string(constraint_->timeSteps_.front()) + "," +
                                                std::to_string(constraint_->timeSteps_.back()) + "]}";
                        }
                        else
                            contstraintString = "None";
                        
                        std::string label = getName() + "\n" +
                                            std::to_string(getID()) + "\n" +
                                            costString + "\n" +
                                            contstraintString;
                        return label;
                    }

                    ompl::base::PlannerPtr getLowLevelSolver() const {return llSolver_;};

                    // ompl::control::PlannerData* getPlannerData() const {return data_;};
                
                private:
                    /** \brief Generates a random alpha-numeric name for a node */
                    std::string generateRandomName()
                    {
                        std::string name;
                        std::string chars(
                            "abcdefghijklmnopqrstuvwxyz"
                            "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                            "1234567890");
                        std::random_device rng;
                        std::uniform_int_distribution<> index_dist(0, chars.size() - 1);
                        for(int i = 0; i < 8; ++i) {
                            name += chars[index_dist(rng)];
                        }
                        return name;
                    }
                    
                    /** \brief The representative plan of the node. */
                    PlanControlPtr plan_;
                    
                    /** \brief The parent node in the constraint tree. */
                    NodePtr parent_;

                    // /** \brief The constraint of the node. */
                    ConstraintPtr constraint_;
                    
                    /** \brief The cost of the node. */
                    int cost_;

                    /** \brief The name of the node wrt to the constraint tree -- used by BoostGraph */
                    std::string name_;

                    /** \brief The ID of the node is equal to the order that this* was popped from the priority queue -- used by BoostGraph */
                    int id_;

                    /** \brief The PlannerPtr responsible for filling this node. Only use during retry */
                    ompl::base::PlannerPtr llSolver_;

                    /** \brief The conflicts within this node */
                    std::vector<Conflict> conflicts_;
                };

                /** \Brief The comparator function that the priority queue uses to sort the nodes. */
                struct NodeCompare
                {
                    bool operator()(const NodePtr &n1, const NodePtr &n2) const
                    {
                        // defines a min-heap on a nodes cost
                        return n1->getCost() > n2->getCost();
                    }
                };

                /** \brief The hash function for the nodes, used to maintain uniqueness inside allNodesSet_ */
                struct NodeHasher
                {
                    std::size_t operator()(const NodePtr &node) const
                    {
                        return std::hash<std::string>()(node->getName());
                    }
                };

                /** \brief A cost of a node is equivalent to the number of unique conflict pairs */
                int evaluateCost(const std::vector<Conflict> confs);

                /** \brief Compute the initial solution using multiple threads */
                void parallelRootSolution(PlanControlPtr plan);

                /** \brief generates trajectories for robots in range [startIdx, endIdx) and saves them into plan */
                void parallelRootSolutionHelper(PlanControlPtr plan, unsigned int startIdx, unsigned int endIdx);

                /** \breif expand a single node from the queue, check it for conflicts, and expand it */
                void parallelNodeExpansion(NodePtr& solution, std::vector<unsigned int>& resevered, std::pair<int, int>& merge_indices);

                /** \brief The main replanning function for the high-level constraint tree. Updates data of node if replan was successful */
                void attemptReplan(const unsigned int robot, NodePtr node, const bool retry = false);

                /** \brief Create a constraint from the conflicts */
                const ConstraintPtr createConstraint(const unsigned int robot, std::vector<Conflict> &confs);

                /** Function to check if a merge is needed. */
                std::pair<int, int> mergeNeeded();

                /** \brief Updates the conflictCounter_ given the new set of conflicts. */
                void updateConflictCounter(const std::vector<Conflict> &conflicsts);

                /** \brief Function that simulates a plan to determine if it is valid. */
                std::vector<Conflict> findConflicts(const PlanControlPtr &plan) const;

                /** \brief Add a node to the priority queue and the allNodes_ list */
                void pushNode(const NodePtr &n);

                /** \brief Get the top element and then pop it out of the queue */
                NodePtr popNode();

                /** \brief Helper function for splitting a number of jobs evenly amongst a number of workers*/
                std::vector<unsigned int> split(const unsigned int jobs, const unsigned int workers);

                /** \brief Free the memory allocated by this planner */
                void freeMemory();

                /** \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience */
                const SpaceInformation *siC_;

                /** \brief An ordered container containing a solver for every individual */
                std::vector<ompl::base::PlannerPtr> llSolvers_;

                /** \brief The computation time for the low-level solver. */
                double llSolveTime_;

                /** \brief The bound for merging two individuals into one */
                unsigned int mergeBound_;

                /** \brief A hash table to track the conflict pairs -- used in conjuntion with mergeBound_ for triggering a merge. */
                std::map<std::pair<int, int>, unsigned int> conflictCounter_;

                /** \brief The priority queue of the constraint tree */
                std::priority_queue<NodePtr, std::vector<NodePtr>, NodeCompare> pq_;

                /** \brief A list of all nodes, used by freeMemory */
                std::unordered_set<NodePtr, NodeHasher> allNodesSet_;

                /** \brief the boost::adjacency_list object used for examining the high-level constraint tree's behavior. */
                using BoostGraph = boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, NodePtr>;
                BoostGraph tree_;

                /** \brief the map for making edges in tree_. */
                std::map<std::string, boost::graph_traits<BoostGraph>::vertex_descriptor> treeMap_;

                /** \brief The number of nodes expanded during the search. */
                unsigned int numNodesExpanded_;

                unsigned int numApproxSolutions_;

                double rootSolveTime_;

                /** \brief The number of workers (threads) used to generate the root node */
                unsigned int numThreads_{4};

                /** \brief Another instance of K-CBS for solving the merged problem -- not always used but saved for memory purposes. */
                KCBSPtr mergedPlanner_{nullptr};
                
            };
        }
    }
}

#endif
