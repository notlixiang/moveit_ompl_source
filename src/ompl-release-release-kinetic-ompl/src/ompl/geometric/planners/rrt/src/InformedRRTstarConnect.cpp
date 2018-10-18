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

/* Author: Ioan Sucan */

#include "ompl/geometric/planners/rrt/InformedRRTstarConnect.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

//构造函数，需要加一些变量
ompl::geometric::InformedRRTstarConnect::InformedRRTstarConnect(const base::SpaceInformationPtr &si) : base::Planner(si,
                                                                                                                     "InformedRRTstarConnect") {
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.directed = true;

    maxDistance_ = 0.0;

    Planner::declareParam<double>("range", this, &InformedRRTstarConnect::setRange, &InformedRRTstarConnect::getRange,
                                  "0.:1.:10000.");
    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
}

//析构函数，这里不用改
ompl::geometric::InformedRRTstarConnect::~InformedRRTstarConnect() {
    freeMemory();
}

//初始化，不用改
void ompl::geometric::InformedRRTstarConnect::setup() {
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!tStart_)
        tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    if (!tGoal_)
        tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    tStart_->setDistanceFunction(
            std::bind(&InformedRRTstarConnect::distanceFunction, this, std::placeholders::_1, std::placeholders::_2));
    tGoal_->setDistanceFunction(
            std::bind(&InformedRRTstarConnect::distanceFunction, this, std::placeholders::_1, std::placeholders::_2));
}

//清空内存，若重用树则不用改
void ompl::geometric::InformedRRTstarConnect::freeMemory() {
    std::vector<Motion *> motions;

    if (tStart_) {
        tStart_->list(motions);
        for (unsigned int i = 0; i < motions.size(); ++i) {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }

    if (tGoal_) {
        tGoal_->list(motions);
        for (unsigned int i = 0; i < motions.size(); ++i) {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
}

//重置数据
void ompl::geometric::InformedRRTstarConnect::clear() {
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (tStart_)
        tStart_->clear();
    if (tGoal_)
        tGoal_->clear();
    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
}

//扩展树，不用改
ompl::geometric::InformedRRTstarConnect::GrowState
ompl::geometric::InformedRRTstarConnect::growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion) {

    /* find closest state in the tree */
    Motion *nmotion = tree->nearest(rmotion);

    /* assume we can reach the state we go towards */
    bool reach = true;

    /* find state to add */
    base::State *dstate = rmotion->state;
    double d = si_->distance(nmotion->state, rmotion->state);
    if (d > maxDistance_) {
        si_->getStateSpace()->interpolate(nmotion->state, rmotion->state, maxDistance_ / d, tgi.xstate);
        dstate = tgi.xstate;
        reach = false;
    }
    // if we are in the start tree, we just check the motion like we normally do;
    // if we are in the goal tree, we need to check the motion in reverse, but checkMotion() assumes the first state it receives as argument is valid,
    // so we check that one first
    bool validMotion = tgi.start ? si_->checkMotion(nmotion->state, dstate) :
                       si_->getStateValidityChecker()->isValid(dstate) && si_->checkMotion(dstate, nmotion->state);
//checkMotion有优化，只检测待到达的姿态是否合法
    if (validMotion) {
        /* create a motion */
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, dstate);
        motion->parent = nmotion;
        motion->root = nmotion->root;
        tgi.xmotion = motion;

        tree->add(motion);
        if (reach)
            return REACHED;//可直接到达这个点
        else
            return ADVANCED;//以步长生长了这个树
    } else
        return TRAPPED;//可扩展的最大距离小于步长，生长失败
}

ompl::base::PlannerStatus ompl::geometric::InformedRRTstarConnect::solve(const base::PlannerTerminationCondition &ptc) {

    checkValidity();//规划器自检
    base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (!goal) {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    while (const base::State *st = pis_.nextStart()) {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        tStart_->add(motion);
    }

//检查初始姿态是否合法
    if (tStart_->size() == 0) {
        OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

//检查目标姿态是否合法
    if (!goal->couldSample()) {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(),
                (int) (tStart_->size() + tGoal_->size()));

    OMPL_INFORM("%s: Starting planning with %d tStart_ %d tGoal_", getName().c_str(),
                (int) (tStart_->size()), (int) (tGoal_->size()));


    //初始化tgi
    TreeGrowingInfo tgi;
    tgi.xstate = si_->allocState();//初始姿态

    Motion *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    bool startTree = true;
    bool solved = false;

//
//    if (tGoal_->size() == 0 || pis_.getSampledGoalsCount() < tGoal_->size() / 2) {
//        const base::State *st = tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
//        if (st) {
//            Motion *motion = new Motion(si_);
//            si_->copyState(motion->state, st);
//            motion->root = motion->state;
//            tGoal_->add(motion);
//        }
//
//        if (tGoal_->size() == 0) {
//            OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
//            //break;
//        }
//    }

    //ptc 是否终止
    bool connect_reached=false;

    //while (ptc == false) {
        while (connect_reached == false) {
        //来回切换出发树和终点树
        TreeData &tree = startTree ? tStart_ : tGoal_;
        tgi.start = startTree;
        startTree = !startTree;
        TreeData &otherTree = startTree ? tStart_ : tGoal_;

        //生成合法的目标姿态，单目标时可放在循环外，可能只在多目标时有用
        if (tGoal_->size() == 0 || pis_.getSampledGoalsCount() < tGoal_->size() / 2) {
            const base::State *st = tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st) {
                Motion *motion = new Motion(si_);
                si_->copyState(motion->state, st);
                motion->root = motion->state;
                tGoal_->add(motion);
            }

            if (tGoal_->size() == 0) {
                OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
                break;
            }
        }

        /* sample random state */

//采样并生长树
        sampler_->sampleUniform(rstate);

        GrowState gs = growTree(tree, tgi, rmotion);

        if (gs != TRAPPED) {//可能直达，可能扩展
            /* remember which motion was just added */
            Motion *addedMotion = tgi.xmotion;

            /* attempt to connect trees */

            /* if reached, it means we used rstate directly, no need top copy again */
            if (gs != REACHED)//扩展的情况,将采样点更新为扩展到的点，保证合法性
                si_->copyState(rstate, tgi.xstate);

            GrowState gsc = ADVANCED;
            tgi.start = startTree;
            while (gsc == ADVANCED)//切换为另一棵树，反向生长到最远
                gsc = growTree(otherTree, tgi, rmotion);

            Motion *startMotion = startTree ? tgi.xmotion : addedMotion;
            Motion *goalMotion = startTree ? addedMotion : tgi.xmotion;

            /* if we connected the trees in a valid way (start and goal pair is valid)*/
            if (gsc == REACHED && goal->isStartGoalPairValid(startMotion->root, goalMotion->root)) {
                // it must be the case that either the start tree or the goal tree has made some progress
                // so one of the parents is not nullptr. We go one step 'back' to avoid having a duplicate state
                // on the solution path
                //退后一步避免重复点
                if (startMotion->parent)
                    startMotion = startMotion->parent;
                else
                    goalMotion = goalMotion->parent;

                connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);

                /* construct the solution path */
                Motion *solution = startMotion;
                std::vector<Motion *> mpath1;
                while (solution != nullptr) {
                    mpath1.push_back(solution);
                    solution = solution->parent;
                }

                solution = goalMotion;
                std::vector<Motion *> mpath2;
                while (solution != nullptr) {
                    mpath2.push_back(solution);
                    solution = solution->parent;
                }

                PathGeometric *path = new PathGeometric(si_);
                path->getStates().reserve(mpath1.size() + mpath2.size());
                for (int i = mpath1.size() - 1; i >= 0; --i)
                    path->append(mpath1[i]->state);
                for (unsigned int i = 0; i < mpath2.size(); ++i)
                    path->append(mpath2[i]->state);

                //pdef_->addSolutionPath(base::PathPtr(path), false, 0.0, getName());
                //solved = true;
                connect_reached=true;
                break;
            }
        }
    }

    si_->freeState(tgi.xstate);
    si_->freeState(rstate);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(),
                tStart_->size(), tGoal_->size());

    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::InformedRRTstarConnect::getPlannerData(base::PlannerData &data) const {
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (tStart_)
        tStart_->list(motions);

    for (unsigned int i = 0; i < motions.size(); ++i) {
        if (motions[i]->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motions[i]->state, 1));
        else {
            data.addEdge(base::PlannerDataVertex(motions[i]->parent->state, 1),
                         base::PlannerDataVertex(motions[i]->state, 1));
        }
    }

    motions.clear();
    if (tGoal_)
        tGoal_->list(motions);

    for (unsigned int i = 0; i < motions.size(); ++i) {
        if (motions[i]->parent == nullptr)
            data.addGoalVertex(base::PlannerDataVertex(motions[i]->state, 2));
        else {
            // The edges in the goal tree are reversed to be consistent with start tree
            data.addEdge(base::PlannerDataVertex(motions[i]->state, 2),
                         base::PlannerDataVertex(motions[i]->parent->state, 2));
        }
    }

    // Add the edge connecting the two trees
    data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));
}

void ompl::geometric::InformedRRTstarConnect::doRRTStar(PathGeometric *path) {
    // RRT* Algorithm
    Motion *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    sampler_->sampleUniform(rstate);

}