/*! \file
 *
 * \author Emmanuel Dean-Leon
 * \author Florian Bergner
 * \author J. Rogelio Guadarrama-Olvera
 * \author Simon Armleder
 * \author Gordon Cheng
 *
 * \version 0.1
 * \date 14.02.2020
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 * #### Acknowledgment
 *  This project has received funding from the European Union‘s Horizon 2020
 *  research and innovation programme under grant agreement No 732287.
 */

#ifndef OPEN_WALKER_CORE_STATE_TRAJECTORIES_H
#define OPEN_WALKER_CORE_STATE_TRAJECTORIES_H

#include <ow_core/types.h>
#include <geometry_msgs/PoseStamped.h>

#include <ow_core/trajectory/state_trajectory.h>

namespace ow
{

/**
 * @brief Specialization of StateTrajectory for Cartesian space.
 *  
 */
class CartesianStateTrajectory : 
  public ow_core::StateTrajectory<ow::CartesianState>
{

public:
  typedef ow_core::StateTrajectory<ow::CartesianState> Base;
  typedef ow_core::PolynomialTrajectory<ow::CartesianPosition, ow::Scalar> Trajectory;

public:
  /*!
   * \brief Constructor.
   *
   * Takes a constructed trajectory object as input.
   */
  CartesianStateTrajectory(const Trajectory& trajectory) :
    Base(trajectory)
  {
  }

  /*!
   * \brief Deconstructor.
   */
  ~CartesianStateTrajectory()
  {
  }

  /**
   * @brief Convert to nav_msgs::Path with n_steps evenly spaced in time.
   * 
   * @param n_steps 
   * @return nav_msgs::Path 
   */
  std::vector<geometry_msgs::PoseStamped> toNavPath(size_t n_steps)
  {
    std::vector<geometry_msgs::PoseStamped> path;
    geometry_msgs::PoseStamped pose;

    double dt_eval = (Base::endTime() - Base::startTime())/ow::Scalar(n_steps);

    for(size_t i = 0; i < n_steps; ++i)
    {
      ow::Scalar time = Base::startTime() + i*dt_eval;
      pose.pose = trajectory_.evaluate(time);
      path.push_back(pose);
    }
    return path;
  }

};

}

#endif