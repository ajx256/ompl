/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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

/* Authors: Ioan Sucan, Luis G. Torres */

#include "ompl/base/objectives/PathLengthOptimizationObjective.h"

ompl::base::PathLengthOptimizationObjective::PathLengthOptimizationObjective(const SpaceInformationPtr &si, double maximumPathLength) : 
    AccumulativeOptimizationObjective(si),
    maxPathLength_(maximumPathLength)
{
    description_ = "Path Length";
}

bool ompl::base::PathLengthOptimizationObjective::isSatisfied(const Cost* cost) const
{
    return (cost->as<CostType>()->getValue() <= maxPathLength_);
}

bool ompl::base::PathLengthOptimizationObjective::isCostLessThan(const Cost* c1, const Cost* c2) const
{
    return (c1->as<CostType>()->getValue() < c2->as<CostType>()->getValue());
}

void ompl::base::PathLengthOptimizationObjective::getIncrementalCost(const State *s1, const State *s2, Cost* cost) const
{
    cost->as<CostType>()->setValue(si_->distance(s1,s2));
}

void ompl::base::PathLengthOptimizationObjective::combineObjectiveCosts(const Cost* c1, const Cost* c2, Cost* cost) const
{
    cost->as<CostType>()->setValue(c1->as<CostType>()->getValue() + 
				   c2->as<CostType>()->getValue());
}
void ompl::base::PathLengthOptimizationObjective::getInitialCost(const State* s, Cost* cost) const
{
    cost->as<CostType>()->setValue(0.0);
}

ompl::base::Cost* ompl::base::PathLengthOptimizationObjective::allocCost(void) const
{
    return new CostType;
}

void ompl::base::PathLengthOptimizationObjective::copyCost(Cost* dest, const Cost* src) const
{
    dest->as<CostType>()->setValue(src->as<CostType>()->getValue());
}

void ompl::base::PathLengthOptimizationObjective::freeCost(Cost* cost) const
{
    delete cost->as<CostType>();
}
