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


#ifndef OPEN_WALKER_CORE_I_TRAJECTORY_H
#define OPEN_WALKER_CORE_I_TRAJECTORY_H

#include <vector>
#include <ow_core/types.h>

namespace ow_core{

/*!
 * \brief The ITrajectory class.
 *
 * The interface class for trajectories.
 * 
 * Trajectories need to provide these functions.
 */
template<typename _Value, typename _Scalar>
class ITrajectory
{
public:
  typedef _Scalar Scalar;
  typedef _Value Value;

  typedef std::vector<Scalar> ScalarVec;                // [time][Scalar]
  typedef std::vector<Value> ValueVec;                  // [time][value] or [derivative][value]
  typedef std::vector<ValueVec> ValueMatrix;            // [time][derivative][value]

public:
  /*!
   * \brief Virtual destructor.
   */
  virtual ~ITrajectory()
  {
  }

  /*!
   * \brief get end time of the trajectory
   */
  virtual Scalar startTime() const = 0;

  /*!
   * \brief get end time of the trajectory
   */
  virtual Scalar endTime() const = 0;

  /*!
   * \brief evaluate trajectory at a single time t
   */
  virtual Value evaluate(const Scalar& t) const = 0;

  /*!
   * \brief evaluate trajectory at at all times t_vec
   * 
   * \return A vector of values with the solution at all t in t_vec.
   */
  virtual ValueVec evaluate(const ScalarVec& t_vec) const = 0;

  /*!
   * \brief evaluate the trajectory derivatives at a single time t.
   * 
   * \return The value of the d order derivative at time t
   */
  virtual Value evaluateDiff(const Scalar& t, int d = 1) const = 0;

  /*!
   * \brief evaluate the trajectory derivatives at all t in t_vec.
   * 
   * \return A vector of values with the d order derivative at time t
   */
  virtual ValueVec evaluateDiff(const ScalarVec& t_vec, int d = 1) const = 0;

  /*!
   * \brief evaluate trajectory at a single time t and all of its 
   * derivatives up to d
   * 
   * \return Vector with [derivative][value]
   */
  virtual ValueVec evaluateAll(const Scalar& t, int d = 1) const = 0;

  /*!
   * \brief evaluate trajectory at all times in t_vec and all 
   * of its derivatives up to d.
   * 
   * \return 2d vector with [derivative][time][value]
   */
  virtual ValueMatrix evaluateAll(const ScalarVec& t_vec, int d = 1) const = 0;
};

} // namespace ow_core

#endif // OPEN_WALKER_CORE_I_SCALAR_FILTER_H
