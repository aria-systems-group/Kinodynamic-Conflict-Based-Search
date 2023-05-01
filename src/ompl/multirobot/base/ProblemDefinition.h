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

/* Author: Justin Kottinger */


#ifndef OMPL_MULTIROBOT_PROBLEM_DEFINITION_
#define OMPL_MULTIROBOT_PROBLEM_DEFINITION_

#include <ompl/base/ProblemDefinition.h>
#include "ompl/multirobot/base/Plan.h"
#include "ompl/multirobot/base/SpaceInformation.h"
#include <mutex>

namespace ompl
{
    namespace multirobot
    {
        namespace base
        {
            /// @cond IGNORE
            /** \brief Forward declaration of ompl::base::ProblemDefinition */
            OMPL_CLASS_FORWARD(ProblemDefinition);
            OMPL_CLASS_FORWARD(OptimizationObjective);
            OMPL_CLASS_FORWARD(Plan);
            /// @endcond

            /** \brief Representation of a solution to the multi-agent planning problem */
            struct PlannerSolution
            {
            
                /** \brief Construct a solution that consists of a \e plan and its attributes (whether it is \e approximate
                 * and the \e difference to the desired goal) */
                PlannerSolution(const PlanPtr &plan): 
                    plan_(plan), length_(plan ? plan->length() : std::numeric_limits<double>::infinity())
                {
                }

                /** \brief Return true if two solutions are the same */
                bool operator==(const PlannerSolution &p) const
                {
                    return plan_ == p.plan_;
                }

                /** \brief Define a ranking for solutions */
                bool operator<(const PlannerSolution &b) const;

                /** \brief Specify that the solution is approximate and set the difference to the goal. */
                void setApproximate(double difference)
                {
                    approximate_ = true;
                    difference_ = difference;
                }

                // /** \brief Set the optimization objective used to optimize this solution, the cost of the solution and
                //  * whether it was optimized or not. */
                // void setOptimized(const OptimizationObjectivePtr &opt, Cost cost, bool meetsObjective)
                // {
                //     opt_ = opt;
                //     cost_ = cost;
                //     optimized_ = meetsObjective;
                // }

                /** \brief Set the name of the planner used to compute this solution */
                void setPlannerName(const std::string &name)
                {
                    plannerName_ = name;
                }

                /** \brief When multiple solutions are found, each is given a number starting at 0, so that the order in
                 * which the solutions was found can be retrieved. */
                int index_{-1};

                /** \brief Solution plan */
                PlanPtr plan_;

                /** \brief For efficiency reasons, keep the length of the plan as well */
                double length_;

                /** \brief True if goal was not achieved, but an approximate solution was found */
                bool approximate_{false};

                /** \brief The achieved difference between the found solution and the desired goal */
                double difference_{0.};

                // /** \brief True if the solution was optimized to meet the specified optimization criterion */
                // bool optimized_{false};

                // /** \brief Optimization objective that was used to optimize this solution */
                // OptimizationObjectivePtr opt_;

                // /** \brief The cost of this solution path, with respect to the optimization objective */
                // Cost cost_;

                /** \brief Name of planner type that generated this solution, as received from Planner::getName() */
                std::string plannerName_;
            };

            /** \brief Definition of a problem to be solved. This includes
            the start state(s) for the system and a goal specification.
            Will contain solutions, if found.  */
            class ProblemDefinition
            {
            public:
                // non-copyable
                ProblemDefinition(const ProblemDefinition &) = delete;
                ProblemDefinition &operator=(const ProblemDefinition &) = delete;

                /** \brief Create a problem definition given the SpaceInformation it is part of */
                ProblemDefinition(SpaceInformationPtr si);

                // /** \brief Return a copy of the problem definition
                //  *
                //  * A deep copy is made of the start and goal states. A shallow copy is made
                //  * of shared ptrs. The set of solutions paths and the intermediate solution
                //  * callback function are not copied.
                //  */
                // ProblemDefinitionPtr clone() const;

                // virtual ~ProblemDefinition()
                // {
                //     clearStartStates();
                // }

                /** \brief Get the space information this problem definition is for */
                const SpaceInformationPtr &getSpaceInformation() const
                {
                    return si_;
                }

                void clearSolutionPaths()
                {
                    for (auto &pdef: individuals_)
                        pdef->clearSolutionPaths();
                    solutions_->clear();
                }

                // /** \brief Add a start state. The state is copied. */
                // void addStartStateAtIndex(const unsigned int index, const ompl::base::State *state)
                // {
                //     if (index + 1 >= si_->getIndividualCount())
                //         startStates_.resize(index + 1);
                //     startStates_[index].push_back(si_->getIndividual(index)->cloneState(state));
                // }

                // /** \copydoc addStartStateAtIndex() */
                // void addStartStateAtIndex(const unsigned int index, const ompl::base::ScopedState<> &state)
                // {
                //     if (index + 1 >= si_->getIndividualCount())
                //         startStates_.resize(index + 1);
                //     startStates_[index].push_back(si_->getIndividual(index)->cloneState(state.get()));

                // }

                // /** \brief Check whether a specified starting state is
                //     already included in the problem definition and
                //     optionally return the index of that starting state */
                // bool hasStartState(const State *state, unsigned int *startIndex = nullptr) const;

                // /** \brief Clear all start states (memory is freed) */
                // void clearStartStatesAtIndex(unsigned int index)
                // {
                //     if (index + 1 >= si_->getIndividualCount())
                //         startStates_.resize(index + 1);
                //     for (auto &startState : startStates_[index])
                //         si_->getIndividual(index)->freeState(startState);
                //     startStates_[index].clear();
                // }

                // /** \brief Returns the number of start states */
                // unsigned int getStartStateCount() const
                // {
                //     return startStates_.size();
                // }

                // /** \brief Returns a specific start state */
                // const ompl::base::State *getIndividualStartState(unsigned int individual, unsigned int index) const
                // {
                //     return startStates_[individual][index];
                // }

                // /** \copydoc getIndividualStartState() */
                // ompl::base::State *getIndividualStartState(unsigned int individual, unsigned int index)
                // {
                //     return startStates_[individual][index];
                // }

                // /** \brief Set the goal. */
                // void setGoalAtIndex(unsigned int index, const ompl::base::GoalPtr &goal)
                // {
                //     if (index + 1 >= si_->getIndividualCount())
                //         goals_.resize(index + 1);
                //     goals_[index] = goal;
                // }

                // /** \brief Clear the goal. Memory is freed. */
                // void clearGoalAtIndex(unsigned int index)
                // {
                //     if (index + 1 >= si_->getIndividualCount())
                //         goals_.resize(index + 1);
                //     goals_[index].reset();
                // }

                // /** \brief Return the current goal */
                // const ompl::base::GoalPtr &getGoalAtIndex(unsigned int index) const
                // {
                //     return goals_[index];
                // }

                // /** \brief Get all the input states. This includes start
                //     states and states that are part of goal regions that
                //     can be casted as ompl::base::GoalState or
                //     ompl::base::GoalStates. */
                // void getInputStates(std::vector<const State *> &states) const;

                /** \brief Adds an individual as part of the multi-agent state space. */
                void addIndividual(const ompl::base::ProblemDefinitionPtr &individual);

                /** \brief Get a specific subspace from the compound state space */
                const ompl::base::ProblemDefinitionPtr &getIndividual(unsigned int index) const;

                /** \brief Get the number of individuals that make up the multi-agent state space */
                unsigned int getIndividualCount() const;

                void lock()
                {
                    locked_ = true;
                }

                // /** \brief In the simplest case possible, we have a single
                //     starting state and a single goal state.

                //     This function simply configures the problem definition
                //     using these states (performs the needed calls to
                //     addStartState(), creates an instance of
                //     ompl::base::GoalState and calls setGoal() on it. */
                // void setStartAndGoalStatesAtIndex(unsigned int index, const ompl::base::State *start, const ompl::base::State *goal,
                //                        double threshold = std::numeric_limits<double>::epsilon());

                // /** \copydoc setStartAndGoalStatesAtIndex() */
                // void setStartAndGoalStatesAtIndex(unsigned int index, const ompl::base::ScopedState<> &start, const ompl::base::ScopedState<> &goal,
                //                        const double threshold = std::numeric_limits<double>::epsilon())
                // {
                //     setStartAndGoalStatesAtIndex(index, start.get(), goal.get(), threshold);
                // }


                // /** \brief A simple form of setting the goal. This is called by setStartAndGoalStates(). A more general form
                //  * is setGoalAtIndex() */
                // void setGoalStateAtIndex(unsigned int index, const ompl::base::State *goal, double threshold = std::numeric_limits<double>::epsilon());

                // /** \copydoc setStartAndGoalStates() */
                // void setStartAndGoalStates(const ScopedState<> &start, const ScopedState<> &goal,
                //                            const double threshold = std::numeric_limits<double>::epsilon())
                // {
                //     setStartAndGoalStates(start.get(), goal.get(), threshold);
                // }

                // /** \copydoc setGoalState() */
                // void setGoalState(const ScopedState<> &goal,
                //                   const double threshold = std::numeric_limits<double>::epsilon())
                // {
                //     setGoalState(goal.get(), threshold);
                // }

                // /** \brief Check if an optimization objective was defined for planning  */
                // bool hasOptimizationObjective() const
                // {
                //     return optimizationObjective_ != nullptr;
                // }

                // /** \brief Get the optimization objective to be considered during planning */
                // const OptimizationObjectivePtr &getOptimizationObjective() const
                // {
                //     return optimizationObjective_;
                // }

                // /** \brief Set the optimization objective to be considered during planning */
                // void setOptimizationObjective(const OptimizationObjectivePtr &optimizationObjective)
                // {
                //     optimizationObjective_ = optimizationObjective;
                // }

                // /** \brief When this function returns a valid function pointer, that function should be called
                //     by planners that compute intermediate solutions every time a better solution is found */
                // const ReportIntermediateSolutionFn &getIntermediateSolutionCallback() const
                // {
                //     return intermediateSolutionCallback_;
                // }

                // /** \brief Set the callback to be called by planners that can compute intermediate solutions */
                // void setIntermediateSolutionCallback(const ReportIntermediateSolutionFn &callback)
                // {
                //     intermediateSolutionCallback_ = callback;
                // }

                // /** \brief A problem is trivial if a given starting state already
                //     in the goal region, so we need no motion planning. startID
                //     will be set to the index of the starting state that
                //     satisfies the goal. The distance to the goal can
                //     optionally be returned as well. */
                // bool isTrivial(unsigned int *startIndex = nullptr, double *distance = nullptr) const;

                // /** \brief Check if a straight line path is valid. If it
                //     is, return an instance of a path that represents the
                //     straight line.

                //     \note When planning under geometric constraints, this
                //     works only if the goal region can be sampled. If the
                //     goal region cannot be sampled, this call is equivalent
                //     to calling isTrivial()

                //     \note When planning under differential constraints,
                //     the system is propagated forward in time using the
                //     null control. */
                // PathPtr isStraightLinePathValid() const;

                // /** \brief Many times the start or goal state will barely touch an obstacle. In this case, we may want to
                //  * automatically
                //   * find a nearby state that is valid so motion planning can be performed. This function enables this
                //  * behaviour.
                //   * The allowed distance for both start and goal states is specified. The number of attempts
                //   * is also specified. Returns true if all states are valid after completion. */
                // bool fixInvalidInputStates(double distStart, double distGoal, unsigned int attempts);

                // /** \brief Returns true if a solution path has been found (could be approximate) */
                // bool hasSolution() const;

                // /** \brief Returns true if an exact solution path has been found. Specifically returns hasSolution &&
                //  * !hasApproximateSolution() */
                // bool hasExactSolution() const
                // {
                //     return this->hasSolution() && !this->hasApproximateSolution();
                // }

                // /** \brief Return true if the top found solution is
                //     approximate (does not actually reach the desired goal,
                //     but hopefully is closer to it) */
                // bool hasApproximateSolution() const;

                // /** \brief Get the distance to the desired goal for the top solution. Return -1.0 if there are no solutions
                //  * available. */
                // double getSolutionDifference() const;

                // /** \brief Return true if the top found solution is optimized (satisfies the specified optimization
                //  * objective) */
                // bool hasOptimizedSolution() const;

                /** \brief Return the top solution path, if one is found. The top path is a shortest
                    path that was found, preference being given to solutions that are not approximate.

                    This will need to be casted into the specialization computed by the planner */
                PlanPtr getSolutionPlan() const;

                // /** \brief Return true if a top solution is found, with the top solution passed by reference in the function
                //    header
                //      The top path is a shortest path that was found, preference being given to solutions that are not
                //    approximate.
                //     This will need to be casted into the specialization computed by the planner */
                // bool getSolution(PlannerSolution &solution) const;

                // /** \brief Add a solution path in a thread-safe manner. Multiple solutions can be set for a goal.
                //     If a solution does not reach the desired goal it is considered approximate.
                //     Optionally, the distance between the desired goal and the one actually achieved is set by \e difference.
                //     Optionally, the name of the planner that generated the solution
                // */
                void addSolutionPlan(const PlanPtr &path, bool approximate = false, double difference = -1.0,
                                     const std::string &plannerName = "Unknown") const;

                /** \brief Add a solution path in a thread-safe manner. Multiple solutions can be set for a goal. */
                void addSolutionPlan(const PlannerSolution &sol) const;

                // /** \brief Get the number of solutions already found */
                // std::size_t getSolutionCount() const;

                // /** \brief Get all the solution paths available for this goal */
                // std::vector<PlannerSolution> getSolutions() const;

                // /** \brief Forget the solution paths (thread safe). Memory is freed. */
                // void clearSolutionPaths() const;

                // /** \brief Returns true if the problem definition has a proof of non existence for a solution */
                // bool hasSolutionNonExistenceProof() const;

                // /** \brief Removes any existing instance of SolutionNonExistenceProof */
                // void clearSolutionNonExistenceProof();

                // /** \brief Retrieve a pointer to the SolutionNonExistenceProof instance for this problem definition */
                // const SolutionNonExistenceProofPtr &getSolutionNonExistenceProof() const;

                // /** \brief Set the instance of SolutionNonExistenceProof for this problem definition */
                // void setSolutionNonExistenceProof(const SolutionNonExistenceProofPtr &nonExistenceProof);

                // /** \brief Print information about the start and goal states and the optimization objective */
                // void print(std::ostream &out = std::cout) const;
                
                /// @cond IGNORE
                OMPL_CLASS_FORWARD(PlannerSolutionSet);
                /// @endcond

                class PlannerSolutionSet
                {
                public:
                    PlannerSolutionSet() = default;

                    void add(const PlannerSolution &s)
                    {
                        std::lock_guard<std::mutex> slock(lock_);
                        int index = solutions_.size();
                        solutions_.push_back(s);
                        solutions_.back().index_ = index;
                        std::sort(solutions_.begin(), solutions_.end());
                    }

                    void clear()
                    {
                        std::lock_guard<std::mutex> slock(lock_);
                        solutions_.clear();
                    }

                    std::vector<PlannerSolution> getSolutions()
                    {
                        std::lock_guard<std::mutex> slock(lock_);
                        std::vector<PlannerSolution> copy = solutions_;
                        return copy;
                    }

                    // bool isApproximate()
                    // {
                    //     std::lock_guard<std::mutex> slock(lock_);
                    //     bool result = false;
                    //     if (!solutions_.empty())
                    //         result = solutions_[0].approximate_;
                    //     return result;
                    // }

                    // bool isOptimized()
                    // {
                    //     std::lock_guard<std::mutex> slock(lock_);
                    //     bool result = false;
                    //     if (!solutions_.empty())
                    //         result = solutions_[0].optimized_;
                    //     return result;
                    // }

                    // double getDifference()
                    // {
                    //     std::lock_guard<std::mutex> slock(lock_);
                    //     double diff = -1.0;
                    //     if (!solutions_.empty())
                    //         diff = solutions_[0].difference_;
                    //     return diff;
                    // }

                    PlanPtr getTopSolution()
                    {
                        std::lock_guard<std::mutex> slock(lock_);
                        PlanPtr copy;
                        if (!solutions_.empty())
                            copy = solutions_[0].plan_;
                        return copy;
                    }

                    bool getTopSolution(PlannerSolution &solution)
                    {
                        std::lock_guard<std::mutex> slock(lock_);

                        if (!solutions_.empty())
                        {
                            solution = solutions_[0];
                            return true;
                        }
                        else
                        {
                            return false;
                        }
                    }

                    std::size_t getSolutionCount()
                    {
                        std::lock_guard<std::mutex> slock(lock_);
                        std::size_t result = solutions_.size();
                        return result;
                    }

                private:
                    std::vector<PlannerSolution> solutions_;
                    std::mutex lock_;
                };

            protected:
                /** \brief The multi-agent space information this problem definition is for */
                SpaceInformationPtr si_;

                /** \brief The individual space informations that make up the multi-agent state space */
                std::vector<ompl::base::ProblemDefinitionPtr> individuals_;

                /** \brief The number of indivudals in the multi-agent state space */
                unsigned int individualCount_{0u};

                /** \brief Boolean that indicates that there are no additional individuals to add */
                bool locked_;

                // /** \brief A Representation of a proof of non-existence of a solution for this problem definition */
                // SolutionNonExistenceProofPtr nonExistenceProof_;

                // /** \brief The objective to be optimized while solving the planning problem */
                // OptimizationObjectivePtr optimizationObjective_;

                // /** \brief Callback function which is called when a new intermediate solution has been found.*/
                // ReportIntermediateSolutionFn intermediateSolutionCallback_;

            private:

                /** \brief The set of solutions computed for this problem definition (maintains an array of PlannerSolution's) */
                PlannerSolutionSetPtr solutions_;
            };
        }
    }
}

#endif
