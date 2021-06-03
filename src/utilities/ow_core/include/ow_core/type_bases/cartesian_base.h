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


#ifndef OPEN_WALKER_CORE_CARTESIAN_BASE_H
#define OPEN_WALKER_CORE_CARTESIAN_BASE_H

#include <Eigen/Dense>
#include <ow_core/utilities/forward_declarations.h>
#include <ow_core/type_bases/vector_base.h>

namespace ow_core
{
/*!
 * \brief The CartesianBase class.
 */
template <typename _Derived>
class CartesianBase : 
  public VectorBase<_Derived>
{
public:
  typedef _Derived Derived;
  typedef VectorBase<_Derived> Base;

  typedef typename ow::traits<Derived>::Base::Scalar Scalar;
  typedef typename ow::traits<Derived>::LinearRef LinearRef;
  typedef typename ow::traits<Derived>::AngularRef AngularRef;
  typedef typename ow::traits<Derived>::CLinearRef CLinearRef;
  typedef typename ow::traits<Derived>::CAngularRef CAngularRef;
  enum {
    LinearPartAtCompileTime = ow::traits<Derived>::LinearPartAtCompileTime,
    AngularPartCompileTime = ow::traits<Derived>::AngularPartCompileTime,
  };

public:
  /*!
   * \brief access to linear part
   */
  LinearRef linear()
  {
    return LinearRef(Base::derived(), LinearPartAtCompileTime);
  }

  /*!
   * \brief const access to linear part
   */
  CLinearRef linear() const
  {
    return CLinearRef(Base::derived(), LinearPartAtCompileTime);
  }

  /*!
   * \brief access to angular part
   */
  AngularRef angular()
  {
    return AngularRef(Base::derived(), AngularPartCompileTime);
  }

  /*!
   * \brief const access to angular part
   */
  CAngularRef angular() const
  {
    return CAngularRef(Base::derived(), AngularPartCompileTime);
  }

  /*!
   * \brief Assignment operator
   */
  template<typename OtherDerived>
  CartesianBase& operator=(const CartesianBase<OtherDerived>& other)
  {
    linear() = static_cast<const ow_core::Vector3<Scalar>& >(other.linear());
    angular() = static_cast<const ow_core::Vector3<Scalar>& >(other.angular());
    return *this;
  }
};

}  // namespace ow_core

#endif  // OPEN_WALKER_CORE_CARTESIAN_BASE_H
