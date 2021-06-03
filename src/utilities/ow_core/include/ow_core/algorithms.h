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


#ifndef OPEN_WALKER_CORE_ALGORITHMS_H
#define OPEN_WALKER_CORE_ALGORITHMS_H

/*! \file types.h
 *  \brief Contains all the algorithms.
 */

// get the configurations to correctly define all algorithms
#include <ow_core/configuration.h>

#include <ow_core/algorithms/scalar_finite_difference.h>
#include <ow_core/algorithms/scalar_butterworth_filter.h>
#include <ow_core/algorithms/scalar_exponential_decay_filter.h>

#include <ow_core/algorithms/state_differentiator.h>
#include <ow_core/algorithms/state_integrator.h>

// the namespace for the configured algorithms
namespace ow
{
  typedef ow_core::ScalarFiniteDifference<ow::Scalar> 
    ScalarFiniteDifference;

  typedef ow_core::ScalarButterWorthFilter<ow::Scalar> 
    ScalarButterWorthFilter;

  typedef ow_core::ScalarExponentialDecayFilter<ow::Scalar> 
    ScalarExponentialDecayFilter;

  template<typename _T>
  using StateIntegrator = ow_core::StateIntegrator<_T>;
  template<typename _T>
  using StateDifferentiator = ow_core::StateDifferentiator<_T>;
}

#endif  // OPEN_WALKER_CORE_ALGORITHMS_H
