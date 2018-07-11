/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Locus Robotics
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
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#include <fuse_variables/velocity_linear_3d_stamped.h>
#include <ros/time.h>

#include <ceres/autodiff_cost_function.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <gtest/gtest.h>

#include <vector>

using fuse_variables::VelocityLinear3DStamped;


TEST(VelocityLinear3DStamped, Type)
{
  VelocityLinear3DStamped variable(ros::Time(12345678, 910111213));
  EXPECT_EQ("fuse_variables::VelocityLinear3DStamped", variable.type());
}

TEST(VelocityLinear3DStamped, UUID)
{
  // Verify two velocities at the same timestamp produce the same UUID
  {
    VelocityLinear3DStamped variable1(ros::Time(12345678, 910111213));
    VelocityLinear3DStamped variable2(ros::Time(12345678, 910111213));
    EXPECT_EQ(variable1.uuid(), variable2.uuid());

    VelocityLinear3DStamped variable3(ros::Time(12345678, 910111213), fuse_core::uuid::generate("c3po"));
    VelocityLinear3DStamped variable4(ros::Time(12345678, 910111213), fuse_core::uuid::generate("c3po"));
    EXPECT_EQ(variable3.uuid(), variable4.uuid());
  }

  // Verify two velocities at different timestamps produce different UUIDs
  {
    VelocityLinear3DStamped variable1(ros::Time(12345678, 910111213));
    VelocityLinear3DStamped variable2(ros::Time(12345678, 910111214));
    VelocityLinear3DStamped variable3(ros::Time(12345679, 910111213));
    EXPECT_NE(variable1.uuid(), variable2.uuid());
    EXPECT_NE(variable1.uuid(), variable3.uuid());
    EXPECT_NE(variable2.uuid(), variable3.uuid());
  }

  // Verify two velocities with different hardware IDs produce different UUIDs
  {
    VelocityLinear3DStamped variable1(ros::Time(12345678, 910111213), fuse_core::uuid::generate("wall-e"));
    VelocityLinear3DStamped variable2(ros::Time(12345678, 910111213), fuse_core::uuid::generate("mo"));
    EXPECT_NE(variable1.uuid(), variable2.uuid());
  }
}

struct CostFunctor
{
  CostFunctor() {}

  template <typename T> bool operator()(const T* const x, T* residual) const
  {
    residual[0] = x[0] - T(3.0);
    residual[1] = x[1] + T(8.0);
    residual[2] = x[2] - T(17.0);
    return true;
  }
};

TEST(VelocityLinear3DStamped, Optimization)
{
  // Create a VelocityLinear3DStamped
  VelocityLinear3DStamped velocity(ros::Time(12345678, 910111213), fuse_core::uuid::generate("hal9000"));
  velocity.x() = 1.5;
  velocity.y() = -3.0;
  velocity.y() = 14.0;

  // Create a simple a constraint
  ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunctor, 3, 3>(new CostFunctor());

  // Build the problem.
  ceres::Problem problem;
  problem.AddParameterBlock(
    velocity.data(),
    velocity.size(),
    velocity.localParameterization());
  std::vector<double*> parameter_blocks;
  parameter_blocks.push_back(velocity.data());
  problem.AddResidualBlock(
    cost_function,
    nullptr,
    parameter_blocks);

  // Run the solver
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Check
  EXPECT_NEAR(3.0, velocity.x(), 1.0e-5);
  EXPECT_NEAR(-8.0, velocity.y(), 1.0e-5);
  EXPECT_NEAR(17.0, velocity.z(), 1.0e-5);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
