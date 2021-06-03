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


#ifndef OPEN_WALKER_CORE_CARTESIAN_ACCELERATION_REF_H
#define OPEN_WALKER_CORE_CARTESIAN_ACCELERATION_REF_H

#include <ow_core/utilities/type_guard.h>
#include <ow_core/type_references/linear_acceleration_ref.h>
#include <ow_core/type_references/angular_acceleration_ref.h>
#include <geometry_msgs/Accel.h>

#include <ow_core/type_bases/cartesian_base.h>

namespace ow{

/*!
 * \brief The traits class for the CartesianAcceleration class.
 *
 * This class contains the typedefs and enums for
 * the CartesianAcceleration class.
 *
 * These typedefs and enums are used to define and describe
 * the properties (traits) of this class.
 */
template<typename _Scalar>
struct traits<ow_core::CartesianAcceleration<_Scalar> >
{
  typedef Eigen::Matrix<_Scalar, 6, 1> Base;
  typedef ow_core::LinearAccelerationRef<Base> LinearRef;
  typedef ow_core::AngularAccelerationRef<Base> AngularRef;
  typedef ow_core::LinearAccelerationRef<const Base> CLinearRef;
  typedef ow_core::AngularAccelerationRef<const Base> CAngularRef;
  enum {
    LinearPartAtCompileTime = 0,
    AngularPartCompileTime = 3,
  };
};

} // namespace ow

namespace ow_core
{  
/*!
 * \brief The CartesianAcceleration class.
 *
 * The CartesianAcceleration is of type Eigen::Vector6 and is
 * represented by the math symbol \f$\ddot{\mathbf{X}}\f$.
 *
 * Stores the linear and angular acceleration
 * in a 6 dimensional vector. The linear acceleration part is represented
 * by the first three elements. The angular acceleration by the last three elements.
 *
 */
template <typename _Scalar>
class CartesianAcceleration : 
  public Eigen::Matrix<_Scalar, 6, 1>,
  public CartesianBase<CartesianAcceleration<_Scalar> >,
  TypeGuard
{
  OW_TYPE_GUARD(CartesianAcceleration)

public:
  typedef _Scalar Scalar;
  typedef Eigen::Matrix<Scalar, 6, 1> Base;
  typedef CartesianBase<CartesianAcceleration<_Scalar> > CBase;

  /*!
   * \brief Construct as Default
   *
   * Default is Identity
   */
  static const CartesianAcceleration<Scalar>& Default()
  {
    static const CartesianAcceleration v = Base::Zero();
    return v;
  }

public:
  /*!
   * \brief Default Constructor
   */
  CartesianAcceleration() 
  {
  }

  /*!
   * \brief Copy constructor.
   */
  CartesianAcceleration(const CartesianAcceleration& other) :
    Base(other)
  {
  }

  /*!
   * \brief Copy Constructor
   */
  CartesianAcceleration(const CBase& other) :
    CBase(other)
  {
  }

  /*!
   * \brief Copy constructor.
   *
   * This copy constructor not only works with Eigen matrices
   * but also with their expressions.
   */
  template <typename OtherDerived>
  CartesianAcceleration(const Eigen::EigenBase<OtherDerived>& other) : 
    Base(other.derived())
  {
  }

  /*!
   * \brief Copy constructor.
   * 
   */
  CartesianAcceleration(const SpatialVector<Scalar>& other)
  {
    CBase::operator=(other);
  }

  /*!
   * \brief Copy constructor form geometry_msgs::Accel.
   */
  explicit CartesianAcceleration(const geometry_msgs::Accel& other)
  {
    operator=(other);
  }

  /*!
   * \brief Use assignment operators of base class.
   */
  using Base::operator=;
  using CBase::operator=;

  /*!
   * \brief Assignment operator.
   */
  CartesianAcceleration& operator=(const CartesianAcceleration& other)
  {
    Base::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment from SpatialVector.
   *
   */
  CartesianAcceleration& operator=(const SpatialVector<Scalar>& other)
  {
    CBase::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment from geometry_msgs::Accel.
   */
  void operator=(const geometry_msgs::Accel& Xpp)
  {
    CBase::linear() = Xpp.linear;
    CBase::angular() = Xpp.angular;
  }

  /*!
   * \brief Conversion to geometry_msgs::Accel.
   */
  operator geometry_msgs::Accel() const
  {
    geometry_msgs::Accel Xpp;
    Xpp.linear = CBase::linear();
    Xpp.angular = CBase::angular();
    return Xpp;
  }

  /*!
   * \brief Conversion to geometry_msgs::Twist.
   */
  geometry_msgs::Accel toAccelMsg() const
  {
    return static_cast<geometry_msgs::Accel>(*this);
  }
};

}  // namespace ow_core

#endif  // OPEN_WALKER_CORE_CARTESIAN_ACCELERATION_REF_H
