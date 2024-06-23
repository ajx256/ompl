/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2024, Andrew Mitchell
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
*   * Neither the name of Andrew Mitchell nor the names of any
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

/* Author: Andrew Mitchell */

#include "ompl/base/spaces/Dubins3DStateSpace.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Exception.h"
#include <queue>
#include <memory>
#include <boost/math/constants/constants.hpp>

using namespace ompl::base;

namespace
{
    const double twopi = 2. * boost::math::constants::pi<double>();
    const double DUBINS_EPS = 1e-6;
    const double DUBINS_ZERO = -1e-7;

    inline double mod2pi(double x)
    {
        if (x < 0 && x > DUBINS_ZERO)
            return 0;
        double xm = x - twopi * floor(x / twopi);
        if (twopi - xm < .5 * DUBINS_EPS)
            xm = 0.;
        return xm;
    }

    inline double computeRhoV(const double rhoHMin, const double rhoH)
    {
        // This is the full rho_v = (rho_min^(-2) - rho_h^(-2))^(-1/2) value from the literature...
        return 1.0 / std::sqrt(1.0 / rhoHMin / rhoHMin - 1.0 / rhoH / rhoH);
    }
} // namespace

// The nomenclature in the paper refers to this Dubins maneuver as a unique {RIGHT, RIGHT, RIGHT} maneuver.
// However, I believe this is not actually correct, and that their added 7th Dubins maneuver type is not necessary.
// Given how the RRR maneuver is implemented, it seems more like the original {LEFT, RIGHT, LEFT} maneuver,
// but with 0 lengths for the two LEFT turn segments. So I think it should be implemented as such.
DubinsStateSpace::DubinsPath Dubins3DStateSpace::dubinsRRR(
    const State *state1, const State *state2, const double rhoH) const
{
    // Get the initial and final Dubins states.
    const auto *s1 = state1->as<DubinsStateSpace::StateType>();
    const auto *s2 = state2->as<DubinsStateSpace::StateType>();

    // Compute all the values we need to compute the 2D Dubins Path.
    double dx = s2->getX() - s1->getX();
    double dy = s2->getY() - s1->getY();
    double D = std::sqrt(dx * dx + dy * dy);

    // Normalize distance
    double d = D / rhoH;

    // Normalize the problem using rotation
    double th = mod2pi(std::atan2(dy, dx));
    double alpha = mod2pi(s1->getYaw() - th);
    double beta = mod2pi(s2->getYaw() - th);

    if ((std::fabs(d) < (rhoMin_ * 0.00001)) && 
        (std::fabs(alpha) < (rhoMin_ * 0.00001)) &&
        (std::fabs(beta) < (rhoMin_ * 0.00001)) )
    {
        double dist2D = std::fmax(std::fabs(s1->getX() - s2->getX()), 
            std::fabs(s1->getY() - s2->getY()));

        if (dist2D < (rhoMin_ * 0.00001))
        {
            return DubinsStateSpace::DubinsPath(
                DubinsStateSpace::dubinsPathType[5], 0.0, twopi, 0.0);
        }
    }

    return {};
}

bool Dubins3DStateSpace::isDubinsRRR(const DubinsStateSpace::DubinsPath &dubins2DPath) const
{
    if (dubins2DPath.type_ == DubinsStateSpace::dubinsPathType[5] &&
        !(dubins2DPath.length_[0] > 0.0) && dubins2DPath.length_[1] > 0.0 && !(dubins2DPath.length_[2] > 0.0))
    {
        return true;   
    }

    return false;
}

void Dubins3DStateSpace::stateAtSegmentOffset(double offset, const DubinsStateSpace::StateType* stateInit, DubinsStateSpace::StateType* stateOffset, 
                                              const DubinsStateSpace::DubinsPathSegmentType segmentType) const
{
    double sinYaw = std::sin(stateInit->getYaw());
    double cosYaw = std::cos(stateInit->getYaw());

    switch (segmentType)
    {
    case DubinsStateSpace::DUBINS_LEFT:
        stateOffset->setXY(stateInit->getX() + std::sin(stateInit->getYaw() + offset) - sinYaw, 
            stateInit->getY() - std::cos(stateInit->getYaw() + offset) + std::cos(stateInit->getYaw()));
        stateOffset->setYaw(stateInit->getYaw() + offset);
        break;

    case DubinsStateSpace::DUBINS_RIGHT:
        stateOffset->setXY(stateInit->getX() - std::sin(stateInit->getYaw() - offset) + sinYaw, 
            stateInit->getY() + std::cos(stateInit->getYaw() - offset) - std::cos(stateInit->getYaw()));
        stateOffset->setYaw(stateInit->getYaw() - offset);
        break;
    
    case DubinsStateSpace::DUBINS_STRAIGHT:
        stateOffset->setXY(stateInit->getX() + cosYaw * offset, stateInit->getY() + sinYaw * offset);
        stateOffset->setYaw(stateInit->getYaw());
        break;
    }    
}

double Dubins3DStateSpace::distance(const State *state1, const State *state2) const
{
    Dubins3DStateSpace::Dubins3DPath s1s2Path = dubins(state1, state2);

    return s1s2Path.length();
}

std::string Dubins3DStateSpace::dubinsPathTypeToString(const DubinsStateSpace::DubinsPath& path) const
{
    if (isDubinsRRR(path))
    {
        return "RRR";
    }
    else if (path.type_ == DubinsStateSpace::dubinsPathType[0])
    {
        return "LSL";
    }
    else if (path.type_ == DubinsStateSpace::dubinsPathType[1])
    {
        return "RSR";
    }
    else if (path.type_ == DubinsStateSpace::dubinsPathType[2])
    {
        return "RSL";
    }
    else if (path.type_ == DubinsStateSpace::dubinsPathType[3])
    {
        return "LSR";
    }
    else if (path.type_ == DubinsStateSpace::dubinsPathType[4])
    {
        return "RLR";
    }
    else if (path.type_ == DubinsStateSpace::dubinsPathType[5])
    {
        return "LRL";
    }
    else
    {
        return "XXX";
    }
}

std::string Dubins3DStateSpace::dubinsPathTypeSegmentToString(const DubinsStateSpace::DubinsPathSegmentType& segmentType) const
{
    if (segmentType == DubinsStateSpace::DUBINS_LEFT)
    {
        return "L";
    }
    else if (segmentType == DubinsStateSpace::DUBINS_RIGHT)
    {
        return "R";
    }
    else if (segmentType == DubinsStateSpace::DUBINS_STRAIGHT)
    {
        return "S";
    }
    else
    {
        return "X";
    }
}

std::vector<State *> Dubins3DStateSpace::sampleStates(int numSamples, const Dubins3DPath &path)
{
    // Sample n equi-distant states along the path
    std::vector<State *> states;
    states.reserve(numSamples);

    for (int sample = 0; sample < numSamples; sample++)
    {
        // Get an equi-distant offset along the path
        double offset = (path.verticalLength() * sample) / (numSamples - 1);
        // Now convert that offset along the path to a 'time' within [0,1]
        double t = offset / path.verticalLength();
        // And use that as the point to interpolate
        auto* state = allocState();
        interpolate(path, t, state);
        states.push_back(state);
    }

    return states;
}

std::vector<State *> Dubins3DStateSpace::sampleStatesResolution(double resolution, const Dubins3DStateSpace::Dubins3DPath &path)
{
    std::vector<State *> states;
    states.reserve((path.verticalLength() * resolution) + 1);

    // Get some offset along the vertical path
    double offset = 0.0;
    for (; offset <= path.verticalLength(); offset += resolution)
    {
        // Then convert that offset to some time within [0,1]
        double t = offset / path.verticalLength();
        auto * state = allocState();
        interpolate(path, t, state);
        states.push_back(state);
    }

    // If the resolution was too large, we may have missed the last state. But regardless of the resolution,
    // we want the last state included.
    if ((offset - resolution) < path.verticalLength())
    {
        auto * state = allocState();
        interpolate(path, 1.0, state);
        states.push_back(state);
    }

    return states;
}

void Dubins3DStateSpace::interpolate(const State *from, const State *to, const double t, State *state) const
{
    bool firstTime = true;
    Dubins3DStateSpace::Dubins3DPath path;
    interpolate(from, to, t, firstTime, path, state);
}

void Dubins3DStateSpace::interpolate(const State *from, const State *to, const double t, bool &firstTime,
    Dubins3DStateSpace::Dubins3DPath &path, State *state) const
{
    if (t >= 1.)
    {
        if (to != state)
            copyState(state, to);
        return;
    }
    if (t <= 0.)
    {
        if (from != state)
            copyState(state, from);
        return;
    }
    
    if (firstTime)
    {
        path = dubins(from, to);
        firstTime = false;
    }
    interpolate(path, t, state);
}

void Dubins3DStateSpace::interpolate(const Dubins3DStateSpace::Dubins3DPath &path, double t, State *state) const
{
    // We make t be within [0,1] to fit OMPL's existing API, but convert it back to an offset along the path now.
    t *= path.verticalLength();
    // First we need to get the state at t along the vertical path.
    DubinsStateSpace verticalStateSpace(path.rhoV_);
    auto *verticalState = verticalStateSpace.allocState()->as<DubinsStateSpace::StateType>();
    interpolate(path.verticalPath_, path.rhoV_, t, verticalState);
    // The x component of the vertical path is the distance along the horizontal path that the vertical component 
    // corresponds to, so use that as the "time" or "offset" we use for interpolating along the horzontal path.
    DubinsStateSpace horizontalStateSpace(path.rhoH_);
    auto *horizontalState = horizontalStateSpace.allocState()->as<DubinsStateSpace::StateType>();
    interpolate(path.horizontalPath_, path.rhoH_, verticalState->getX(), horizontalState);
    // Then merge the components of the horizontal and vertical states to get the full 3D state.
    // They look like this, so we want x, y, and heading from horizontal and z and pitch from vertical.
    //   Horizontal 2D Dubins curve: {x, y, heading}
    //   Vertical 2D Dubins curve: {length along horizontal curve, z, pitch}
    state->as<StateType>()->setRPY(0.0, verticalState->getYaw(), horizontalState->getYaw());
    state->as<StateType>()->setXYZ(horizontalState->getX(), horizontalState->getY(), verticalState->getY());
    verticalStateSpace.freeState(verticalState);
    horizontalStateSpace.freeState(horizontalState);
}

void Dubins3DStateSpace::interpolate(const DubinsStateSpace::DubinsPath &path, const double rho, double t, State *state) const
{
    DubinsStateSpace stateSpace(rho);
    // Start by getting three configurations
    //     1) The initial configuration, translated to the origin
    auto *s1 = stateSpace.allocState()->as<DubinsStateSpace::StateType>();
    s1->setXY(0.0, 0.0);
    s1->setYaw(path.qi_.yaw);
    //     2) The end of the first segment
    auto *s2 = stateSpace.allocState()->as<DubinsStateSpace::StateType>();
    stateAtSegmentOffset(path.length_[0], s1, s2, path.type_[0]);
    //     3) The end of the second segment
    auto *s3 = stateSpace.allocState()->as<DubinsStateSpace::StateType>();
    stateAtSegmentOffset(path.length_[1], s2, s3, path.type_[1]);
    // Use those along with t to get our interpolated state
    // Normalize the offset
    t /= rho;

    if (t < path.length_[0])
    {
        // Interpolated state in the first segment
        stateAtSegmentOffset(t, s1, state->as<DubinsStateSpace::StateType>(), path.type_[0]);
    }
    else if (t < (path.length_[0] + path.length_[1]))
    {
        // Interpolated state in the second segment
        stateAtSegmentOffset(t - path.length_[0], s2, state->as<DubinsStateSpace::StateType>(), path.type_[1]);
    }
    else
    {
        // Interpolated state in the third segment
        stateAtSegmentOffset(t - path.length_[0] - path.length_[1], s3, state->as<DubinsStateSpace::StateType>(), path.type_[2]);
    }

    // Then translate the interpolated back from the origin to the original point
    state->as<DubinsStateSpace::StateType>()->setXY(state->as<DubinsStateSpace::StateType>()->getX() * rho + path.qi_.x, 
                                                    state->as<DubinsStateSpace::StateType>()->getY() * rho + path.qi_.y);
    state->as<DubinsStateSpace::StateType>()->setYaw(mod2pi(state->as<DubinsStateSpace::StateType>()->getYaw()));

    stateSpace.freeState(s1);
    stateSpace.freeState(s2);
    stateSpace.freeState(s3);
}

unsigned int Dubins3DStateSpace::validSegmentCount(const State *state1, const State *state2) const
{
    return StateSpace::validSegmentCount(state1, state2);
}

std::vector<DubinsStateSpace::DubinsPath> Dubins3DStateSpace::constructDecoupledHorizontalVerticalPaths(
    DubinsStateSpace& horizontalStateSpace, DubinsStateSpace::StateType* startStateH, DubinsStateSpace::StateType* endStateH,
    DubinsStateSpace& verticalStateSpace, DubinsStateSpace::StateType* startStateV, DubinsStateSpace::StateType* endStateV) const
{
    std::vector<DubinsStateSpace::DubinsPath> decoupledCurves;
    decoupledCurves.reserve(2);

    double rhoH = horizontalStateSpace.getTurningRadius();
    double rhoV = verticalStateSpace.getTurningRadius();

    // Now solve for the horizontal Dubins path.
    // Check if we want {RIGHT, RIGHT, RIGHT} first, as if we do, we will use that over 
    // any other feasible Dubins path.
    DubinsStateSpace::DubinsPath dubinsH = dubinsRRR(startStateH, endStateH, rhoH);
    if ((dubinsH.length() * rhoH) < twopi || std::isinf(dubinsH.length() * rhoH))
    {
        dubinsH = horizontalStateSpace.dubins(startStateH, endStateH);
    }

    // Get the vertical curvature based on the horizontal radius.
    // This is the (rho_min^(-2) - rho_h^(-2)) value from the literature...
    double curvatureV = std::sqrt(1.0 / rhoMin_ / rhoMin_ - 1.0 / rhoH / rhoH);
    // If the vertical curvature is too small, we can't feasibly find a solution, return early.
    if (curvatureV < 0.00001)
    {
        return decoupledCurves;
    }

    // Update the vertical end state's X position based on the length of the horizontal 2D Dubins
    // path that we just found.
    endStateV->setX(dubinsH.length() * rhoH);

    // Now solve for the vertical Dubins path.
    // Check if we want {RIGHT, RIGHT, RIGHT} first, as if we do, we will use that over 
    // any other feasible Dubins path.
    DubinsStateSpace::DubinsPath dubinsV = dubinsRRR(startStateV, endStateV, rhoV);
    if ((dubinsV.length() * rhoV) < twopi || std::isinf(dubinsV.length() * rhoV))
    {
        dubinsV = verticalStateSpace.dubins(startStateV, endStateV);
    }

    // TODO: Why don't we like RLR? Just suboptimal?
    if (dubinsV.type_ == DubinsStateSpace::dubinsPathType[4])
    {
        return decoupledCurves;
    }

    if ((dubinsV.type_ == DubinsStateSpace::dubinsPathType[1]) ||
        (dubinsV.type_ == DubinsStateSpace::dubinsPathType[2]) ||
        isDubinsRRR(dubinsV))
    {
        if ((startStateV->getYaw() - dubinsV.length_[0]) < minPitch_)
        {
            return decoupledCurves;
        }
    }
    else
    {
        if ((startStateV->getYaw() + dubinsV.length_[0]) > maxPitch_)
        {
            return decoupledCurves;
        }
    }

    decoupledCurves.push_back(dubinsH);
    decoupledCurves.push_back(dubinsV);

    return decoupledCurves;
}

Dubins3DStateSpace::Dubins3DPath Dubins3DStateSpace::dubins(
    const State *state1, const State *state2) const
{
    // First start by checking if the start and end states are the same
    const auto* startState = state1->as<StateType>();
    const auto* endState = state2->as<StateType>();

    if (equalStates(startState, endState))
    {
        return Dubins3DPath(startState->asDubinsConfiguration(), endState->asDubinsConfiguration(), 
            {DubinsStateSpace::dubinsPathType[0], 0, 0, 0}, {DubinsStateSpace::dubinsPathType[0], 0, 0, 0}, 
            0, 0);
    }

    // Start the horizontal turning radius as being equal to the minimal turning radius.
    double rhoH = rhoMin_;
    double rhoV = computeRhoV(rhoMin_, rhoH);

    // We're going to need a DubinsStateSpace to get the horizontal path, for our current value
    // of rhoH.
    DubinsStateSpace horizontalStateSpace(rhoH);
    // Then construct the start and end states of the horizontal path.
    auto *startStateH = horizontalStateSpace.allocState()->as<DubinsStateSpace::StateType>();
    startStateH->setXY(startState->getX(), startState->getY());
    startStateH->setYaw(startState->getRPY().gamma());
    auto *endStateH = horizontalStateSpace.allocState()->as<DubinsStateSpace::StateType>();
    endStateH->setXY(endState->getX(), endState->getY());
    endStateH->setYaw(endState->getRPY().gamma());

    // We're going to need a DubinsStateSpace to get the vertical path, for our current value
    // of rhoV.
    DubinsStateSpace verticalStateSpace(rhoV);
    // Then construct the start and end states of the vertical path.
    auto *startStateV = verticalStateSpace.allocState()->as<DubinsStateSpace::StateType>();
    startStateV->setXY(0.0, startState->getZ());
    startStateV->setYaw(startState->getRPY().beta());
    auto *endStateV = verticalStateSpace.allocState()->as<DubinsStateSpace::StateType>();
    endStateV->setXY(0.0, endState->getZ());
    endStateV->setYaw(endState->getRPY().beta());

    // Try solving for the decoupled horizontal and vertical curves using rhoH and [minPitch, maxPitch].
    // It is possible that a vertical curve that satisfies [minPitch, maxPitch] is not feasible for this value
    // of rhoH. In that case, we will iteratively increase rhoH until it is feasible.
    std::vector<DubinsStateSpace::DubinsPath> decoupledCurves = constructDecoupledHorizontalVerticalPaths(horizontalStateSpace, startStateH, endStateH,
                                                                                                          verticalStateSpace, startStateV, endStateV);

    // As long as the vertical curve is not feasible for the current value of rhoH, double rhoH and try again...
    // (Please interpret !decoupledCurves.empty() as IsFeasible(curveVertical, pitchLimits) from the literature)
    while (decoupledCurves.empty() && !std::isinf(rhoH) && !std::isnan(rhoH))
    {
        // Increase the horizontal turning radius
        rhoH *= 2;
        // Compute the resulting vertical curve
        rhoV = computeRhoV(rhoMin_, rhoH);
        // Update the turning radius for the 2D state spaces
        horizontalStateSpace.setTurningRadius(rhoH);
        verticalStateSpace.setTurningRadius(rhoV);
        // Try to construct the decoupled paths again
        decoupledCurves = constructDecoupledHorizontalVerticalPaths(horizontalStateSpace, startStateH, endStateH,
                                                                    verticalStateSpace, startStateV, endStateV);
    }

    // Once we have found a feasible solution, perform local optimization of the horizontal radius to decrease it while still
    // maintaining feasibility of the vertical curve.
    double delta = 0.1 * rhoMin_;
    while (!decoupledCurves.empty() && std::abs(delta) > 0.0000000001)
    {
        // Get candidate values for rhoH and rhoV
        double rhoHPrime = std::max(rhoMin_, rhoH + delta);
        double rhoVPrime = computeRhoV(rhoMin_, rhoHPrime);
        // Update the decoupled 2D state spaces
        horizontalStateSpace.setTurningRadius(rhoHPrime);
        verticalStateSpace.setTurningRadius(rhoVPrime);
        // Solve for the decoupled paths
        std::vector<DubinsStateSpace::DubinsPath> decoupledCurvesPrime = constructDecoupledHorizontalVerticalPaths(
            horizontalStateSpace, startStateH, endStateH,
            verticalStateSpace, startStateV, endStateV);
        // (Please interpret !decoupledCurves.empty() as IsFeasible(curveVertical, pitchLimits) from the literature)
        if (!decoupledCurvesPrime.empty() && (decoupledCurvesPrime[1].length() * rhoVPrime) < (decoupledCurves[1].length() * rhoV))
        {
            // A new best solution has been found, so update.
            rhoH = rhoHPrime;
            rhoV = rhoVPrime;
            decoupledCurves = decoupledCurvesPrime;
            delta *= 2;
        }
        else
        {
            delta *= -0.1;
        }

        // Update the decoupled 2D state spaces
        horizontalStateSpace.setTurningRadius(rhoH);
        verticalStateSpace.setTurningRadius(rhoV);
    }

    horizontalStateSpace.freeState(startStateH);
    horizontalStateSpace.freeState(endStateH);
    verticalStateSpace.freeState(startStateV);
    verticalStateSpace.freeState(endStateV);

    if (decoupledCurves.empty())
    {
        // No path exists
        return Dubins3DPath(startState->asDubinsConfiguration(), endState->asDubinsConfiguration());
    }

    // Construct the final 3D Dubins Path by combining the decoupled 2D horizontal and vertical Dubins Paths.
    Dubins3DStateSpace::Dubins3DPath path(startState->asDubinsConfiguration(), endState->asDubinsConfiguration(), 
        decoupledCurves[0], decoupledCurves[1], rhoH, rhoV);

    return path;
}

void Dubins3DMotionValidator::defaultSettings()
{
    stateSpace_ = dynamic_cast<Dubins3DStateSpace *>(si_->getStateSpace().get());
    if (stateSpace_ == nullptr)
        throw Exception("No state space for motion validator");
}

bool Dubins3DMotionValidator::checkMotion(const State *s1, const State *s2,
                                                    std::pair<State *, double> &lastValid) const
{
    /* assume motion starts in a valid configuration so s1 is valid */

    bool result = true, firstTime = true;
    Dubins3DStateSpace::Dubins3DPath path;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    if (nd > 1)
    {
        /* temporary storage for the checked state */
        State *test = si_->allocState();

        for (int j = 1; j < nd; ++j)
        {
            stateSpace_->interpolate(s1, s2, (double)j / (double)nd, firstTime, path, test);
            if (!si_->isValid(test))
            {
                lastValid.second = (double)(j - 1) / (double)nd;
                if (lastValid.first != nullptr)
                    stateSpace_->interpolate(s1, s2, lastValid.second, firstTime, path, lastValid.first);
                result = false;
                break;
            }
        }
        si_->freeState(test);
    }

    if (result)
        if (!si_->isValid(s2))
        {
            lastValid.second = (double)(nd - 1) / (double)nd;
            if (lastValid.first != nullptr)
                stateSpace_->interpolate(s1, s2, lastValid.second, firstTime, path, lastValid.first);
            result = false;
        }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}

bool Dubins3DMotionValidator::checkMotion(const State *s1, const State *s2) const
{
    /* assume motion starts in a valid configuration so s1 is valid */
    if (!si_->isValid(s2))
        return false;

    bool result = true, firstTime = true;
    Dubins3DStateSpace::Dubins3DPath path;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    /* initialize the queue of test positions */
    std::queue<std::pair<int, int>> pos;
    if (nd >= 2)
    {
        pos.emplace(1, nd - 1);

        /* temporary storage for the checked state */
        State *test = si_->allocState();

        /* repeatedly subdivide the path segment in the middle (and check the middle) */
        while (!pos.empty())
        {
            std::pair<int, int> x = pos.front();

            int mid = (x.first + x.second) / 2;
            stateSpace_->interpolate(s1, s2, (double)mid / (double)nd, firstTime, path, test);

            if (!si_->isValid(test))
            {
                result = false;
                break;
            }

            pos.pop();

            if (x.first < mid)
                pos.emplace(x.first, mid - 1);
            if (x.second > mid)
                pos.emplace(mid + 1, x.second);
        }

        si_->freeState(test);
    }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}
