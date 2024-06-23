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

#ifndef OMPL_BASE_SPACES_DUBINS_3D_STATE_SPACE_
#define OMPL_BASE_SPACES_DUBINS_3D_STATE_SPACE_

#include "ompl/base/spaces/SE3StateSpace.h"
#include "ompl/base/spaces/DubinsStateSpace.h"
#include "ompl/base/MotionValidator.h"
#include <boost/math/constants/constants.hpp>
#include <eigen3/unsupported/Eigen/EulerAngles>
#include <limits>

namespace ompl
{
    namespace base
    {
        /** \brief An SE(3) state space where distance is measured by the
            length of 3D Dubins curves.

            Note that this Dubins distance is \b not a proper distance metric,
            so nearest neighbor methods that rely on distance() being a metric
            (such as ompl::NearestNeighborsGNAT) will not always return the
            true nearest neighbors or get stuck in an infinite loop.

            The notation and solutions in the code are taken from:<br>
            Petr Vana, Armando Alves Neto, Jan Faigl, and Douglas G. Macharet, 
            “Minimal 3D Dubins Path with Bounded Curvature and Pitch Angle,”
            ICRA, 2020.

            This code is based on the Julia code from the paper: https://github.com/comrob/Dubins3D.jl

            3D Dubins curves, as described in the paper referenced above,
            are computed by solving for the 2D Dubins curves in the horizontal
            and vertical planes, and iteratively increasing the minimum turning
            radius in the horizontal plane so long as the min and max pitch bounds
            are violated in the vertical plane. The 3D curve can then be constructed
            by sampling points from both the horizontal and vertical curves, yielding
            the full SE(3) state.

            2D curves of the form:
                Horizontal 2D Dubins curve: {x, y, heading}
                Vertical 2D Dubins curve: {length along horizontal curve, z, pitch}
        */
        class Dubins3DStateSpace : public SE3StateSpace
        {
        public:
            typedef Eigen::EulerSystemXYZ TaitBryanSystem;
            typedef Eigen::EulerAngles<double, TaitBryanSystem> TaitBryanAngles;

            /** \brief Provided for convenience for easy conversions */
            static double degToRad(double val)
            {
                return ((M_PI * val) / 180.0);
            }

            /** \brief Provided for convenience for easy conversions */
            static double radToDeg(double val)
            {
                return (val * (180.0 / M_PI));
            }

            /** \brief A struct for holding 3D Dubins configurations. For specifying initial and final configurations along the path. */
            struct Dubins3DConfiguration
            {
                double x{0.0};
                double y{0.0};
                double z{0.0};
                double roll{0.0};
                double pitch{0.0};
                double yaw{0.0};
            };

            /** \brief Dubins3D StateType is an SE3 StateType but with convenience functions for setting and 
             *         getting Tait Bryan Euler Angles. */
            class StateType : public SE3StateSpace::StateType
            {
            public:
                StateType() = default;

                /** \brief Set the roll, pitch, and yaw of the state */
                void setRPY(double roll, double pitch, double yaw)
                {
                    TaitBryanAngles rpy(roll, pitch, yaw);
                    Eigen::Quaterniond quat(rpy.toRotationMatrix());
                    quat.normalize();

                    rotation().w = quat.coeffs().w();
                    rotation().x = quat.coeffs().x();
                    rotation().y = quat.coeffs().y();
                    rotation().z = quat.coeffs().z();
                }

                /** \brief Get the roll, pitch, and yaw of the state */
                TaitBryanAngles getRPY() const
                {
                    Eigen::Quaterniond quat(rotation().w, rotation().x, rotation().y, rotation().z);
                    TaitBryanAngles rpy(quat.toRotationMatrix());

                    return rpy;
                }

                /** \brief Prints the state in the form [x, y, z, roll, pitch, yaw] */
                void printState(std::ostream &out) const
                {
                    out << "[" << getX() << ", " << getY() << ", " << getZ() << ", " 
                        << radToDeg(getRPY().alpha()) << ", " << radToDeg(getRPY().beta()) << ", " 
                        << radToDeg(getRPY().gamma()) << "]" << std::endl;
                }

                /** \brief Returns a string representation of the state in the form [x, y, z, roll, pitch, yaw] */
                std::string toString() const
                {
                    std::string str = "[" + std::to_string(getX()) + ", " + std::to_string(getY()) + ", " 
                        + std::to_string(getZ()) + ", " + std::to_string(radToDeg(getRPY().alpha())) + ", " 
                        + std::to_string(radToDeg(getRPY().beta())) + ", " + std::to_string(radToDeg(getRPY().gamma())) 
                        + "]";

                    return str;
                }

                /** \brief Returns this state as a Dubins3DConfiguration, in the form [x, y, z, roll, pitch, yaw] */
                Dubins3DConfiguration asDubinsConfiguration() const
                {
                    return {
                        getX(),
                        getY(),
                        getZ(),
                        getRPY().alpha(),
                        getRPY().beta(),
                        getRPY().gamma()
                    };
                }

                /** \brief Overloaded operator for writing string representation of the state to an ostream */
                friend std::ostream& operator<<(std::ostream& stream, const StateType& state)
                {
                    stream << state.toString();
                    return stream;
                }
            };

            /** \brief Complete description of a 3D Dubins path */
            class Dubins3DPath
            {
            public:
                Dubins3DPath(Dubins3DConfiguration qi = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 
                             Dubins3DConfiguration qf = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                             DubinsStateSpace::DubinsPath horizontalPath = {DubinsStateSpace::dubinsPathType[0], 0, 0, 0}, 
                             DubinsStateSpace::DubinsPath verticalPath = {DubinsStateSpace::dubinsPathType[0], 0, 0, 0}, 
                             double rhoH = std::numeric_limits<double>::infinity(), double rhoV = std::numeric_limits<double>::infinity())
                : qi_(qi),
                  qf_(qf),
                  horizontalPath_(horizontalPath), 
                  verticalPath_(verticalPath),
                  rhoH_(rhoH), 
                  rhoV_(rhoV)
                {
                    // Length of the 3D path is equivalent to the length
                    // of the vertical 2D path.
                    length_[0] = verticalPath.length_[0];
                    length_[1] = verticalPath.length_[1];
                    length_[2] = verticalPath.length_[2];
                    assert(length_[0] >= 0.);
                    assert(length_[1] >= 0.);
                    assert(length_[2] >= 0.);
                    
                    if (rhoH_ > 0)
                    {
                        curvatureH_ = 1.0 / rhoH_;
                    }

                    if (rhoV_ > 0)
                    {
                        curvatureV_ = 1.0 / rhoV_;
                    }
                }

                double length() const
                {
                    return verticalLength();
                }

                double horizontalLength() const
                {
                    return horizontalPath_.length() * rhoH_;
                }

                double verticalLength() const
                {
                    return verticalPath_.length() * rhoV_;
                }

                /** The initial configuration along the path */
                Dubins3DConfiguration qi_;
                /** The final configuration along the path */
                Dubins3DConfiguration qf_;
                /** Horizontal 2D Dubins path */
                DubinsStateSpace::DubinsPath horizontalPath_;
                /** Vertical 2D Dubins path */
                DubinsStateSpace::DubinsPath verticalPath_;
                /** Path segment lengths */
                double length_[3];
                /** Whether the path should be followed "in reverse" */
                bool reverse_{false};
                /** Horizontal turning radius for this 3D Dubins Path */
                double rhoH_{0.0};
                /** Vertical radius for this 3D Dubins Path */
                double rhoV_{0.0};
                /** Horizontal curvature for this 3D Dubins Path */
                double curvatureH_{0.0};
                /** Vertical curvature for this 3D Dubins Path */
                double curvatureV_{0.0};
            };

            Dubins3DStateSpace(double turningRadius = 1.0, double minPitch = -1.0, double maxPitch = 1.0)
            : rhoMin_(turningRadius), minPitch_(minPitch), maxPitch_(maxPitch)
            {
                type_ = StateSpaceType::STATE_SPACE_3D_DUBINS;
            }

            bool isMetricSpace() const override
            {
                return false;
            }

            double distance(const State *state1, const State *state2) const override;

            /** \brief Return the 2D Dubins path type as a string. */
            std::string dubinsPathTypeToString(const base::DubinsStateSpace::DubinsPath& path) const;
            /** \brief Return the 2D Dubins path segment type as a string. */
            std::string dubinsPathTypeSegmentToString(const DubinsStateSpace::DubinsPathSegmentType& segmentType) const;

            /** Sample the 3D Dubins path to get n equi-distant states along the curve. */
            std::vector<State*> sampleStates(int numSamples, const Dubins3DPath &path);

            /** Sample the 3D Dubins path at the chosen resolution to get states along the curve. */
            std::vector<State*> sampleStatesResolution(double resolution, const Dubins3DPath &path);

            void interpolate(const State *from, const State *to, double t, State *state) const override;
            virtual void interpolate(const State *from, const State *to, double t, bool &firstTime,
                                    Dubins3DPath &path, State *state) const;

            /** \brief Always false for 3D Dubins. */
            bool hasSymmetricDistance() const override
            {
                return isSymmetric_;
            }

            /** \brief Always false for 3D Dubins. */
            bool hasSymmetricInterpolate() const override
            {
                return isSymmetric_;
            }

            unsigned int validSegmentCount(const State *state1, const State *state2) const override;

            void sanityChecks() const override
            {
                double zero = std::numeric_limits<double>::epsilon();
                double eps = std::numeric_limits<float>::epsilon();
                int flags = ~(STATESPACE_INTERPOLATION | STATESPACE_TRIANGLE_INEQUALITY | STATESPACE_DISTANCE_BOUND);
                if (!isSymmetric_)
                    flags &= ~STATESPACE_DISTANCE_SYMMETRIC;
                StateSpace::sanityChecks(zero, eps, flags);
            }

            /** \brief Return a shortest 3D Dubins path from SE(3) state state1 to SE(3) state state2 */
            Dubins3DPath dubins(const State *state1, const State *state2) const;

        protected:
            /** \brief Tries to construct a RRR 2D Dubins path type. This is a seventh type included in the paper,
             *         but is implemented here as a special case of the LRL type where the two L segments have 0 length.
             */
            DubinsStateSpace::DubinsPath dubinsRRR(const State *state1, const State *state2, const double rhoH) const;

            /** \brief Checks if the 2D Dubins path is of the RRR type, which is an LRL type with lengths of zero 
             *         for the two L segments and a non-zero length for the R segment.
             */
            bool isDubinsRRR(const DubinsStateSpace::DubinsPath &dubins2DPath) const;

            /** \brief Gets the state at the given offset along the Dubins segment. */
            void stateAtSegmentOffset(double offset, const DubinsStateSpace::StateType* stateInit, 
                                      DubinsStateSpace::StateType* stateOffset, const DubinsStateSpace::DubinsPathSegmentType segmentType) const;

            virtual void interpolate(const Dubins3DPath &path, double t, State *state) const;

            void interpolate(const DubinsStateSpace::DubinsPath &path, const double rho, double t, State *state) const;

            /** \brief Attempts to construct a 3D Dubins path by deconstructing it into finding the horizontal and vertical 2D Dubins
             *         paths in the XY and SZ planes that satisfy the 3D path's minimum turning radius, minimum pitch, and 
             *         maximum pitch constraints.
             */
            std::vector<DubinsStateSpace::DubinsPath> constructDecoupledHorizontalVerticalPaths(
                DubinsStateSpace& horizontalStateSpace, DubinsStateSpace::StateType* startStateH, DubinsStateSpace::StateType* endStateH,
                DubinsStateSpace& verticalStateSpace, DubinsStateSpace::StateType* startStateV, DubinsStateSpace::StateType* endStateV) const;

            /** \brief Turning radius */
            double rhoMin_;

            /** \brief Minimum pitch angle */
            double minPitch_;

            /** \brief Maximum pitch angle */
            double maxPitch_;

            /** \brief Whether the distance is "symmetrized"

                If true the distance from state s1 to state s2 is the same as the
                distance from s2 to s1. This is done by taking the \b minimum
                length of the 3D Dubins curves that connect s1 to s2 and s2 to s1.
                Dubins paths are not symmetric, so this is not allowed to be set for
                3D Dubins paths. 
            */
            bool isSymmetric_{false};
        };

        /** \brief A 3D Dubins motion validator that only uses the state validity checker.
            Motions are checked for validity at a specified resolution.

            This motion validator is almost identical to the DiscreteMotionValidator
            except that it remembers the optimal DubinsPath between different calls to
            interpolate. */
        class Dubins3DMotionValidator : public MotionValidator
        {
        public:
            Dubins3DMotionValidator(SpaceInformation *si) : MotionValidator(si)
            {
                defaultSettings();
            }
            Dubins3DMotionValidator(const SpaceInformationPtr &si) : MotionValidator(si)
            {
                defaultSettings();
            }
            ~Dubins3DMotionValidator() override = default;
            bool checkMotion(const State *s1, const State *s2) const override;
            bool checkMotion(const State *s1, const State *s2, std::pair<State *, double> &lastValid) const override;

        private:
            Dubins3DStateSpace *stateSpace_;
            void defaultSettings();
        };
    }
}
#endif
