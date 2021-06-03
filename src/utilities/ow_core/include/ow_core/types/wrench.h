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


#ifndef OPEN_WALKER_CORE_WRENCH_H
#define OPEN_WALKER_CORE_WRENCH_H

#include <geometry_msgs/Wrench.h>

#include <ow_core/type_references/force_ref.h>
#include <ow_core/type_references/moment_ref.h>

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
struct traits<ow_core::Wrench<_Scalar> >
{
  typedef Eigen::Matrix<_Scalar, 6, 1> Base;
  typedef ow_core::ForceRef<Base> LinearRef;
  typedef ow_core::MomentRef<Base> AngularRef;
  typedef ow_core::ForceRef<const Base> CLinearRef;
  typedef ow_core::MomentRef<const Base> CAngularRef;
  enum {
    LinearPartAtCompileTime = 0,
    AngularPartCompileTime = 3,
  };
};

} // namespace ow

namespace ow_core
{
/*!
 * \brief The Wrench class.
 *
 * The Wrench is of type Eigen::Vector6 and is
 * represented by the math symbol \f$\mathbf{W}\f$.
 *
 * Stores the force and moment in a 6 dimensional vector.
 * The force is represented by the first three elements.
 * The moment velocity by the last three elements.
 *
 */
template <typename _Scalar>
class Wrench :
    public Eigen::Matrix<_Scalar, 6, 1>,
    public CartesianBase<Wrench<_Scalar> >,
    TypeGuard
{
  OW_TYPE_GUARD(Wrench)

public:
  typedef _Scalar Scalar;
  typedef Eigen::Matrix<Scalar, 6, 1> Base;
  typedef CartesianBase<Wrench<_Scalar> > CBase;

  /*!
   * \brief Construct as Default
   *
   * Default is Zero.
   */
  static const Wrench& Default()
  {
    static const Wrench v = Base::Zero();
    return v;
  }

public:
  /*!
   * \brief Default Constructor
   */
  Wrench()
  {
  }

  /*!
   * \brief Copy Constructor
   */
  Wrench(const Wrench& other) : 
    Base(other)
  {
  }

  /*!
   * \brief Copy Constructor
   */
  Wrench(const CBase& other) :
    CBase(other)
  {
  }

  /*!
   * \brief Constructor from Scalars
   */
  Wrench(const Scalar& fx, const Scalar& fy, const Scalar& fz, 
         const Scalar& mux, const Scalar& muy, const Scalar& muz)
  {
    force().x() = fx;
    force().y() = fy;
    force().z() = fz;
    moment().x() = mux;
    moment().y() = muy;
    moment().z() = muz;
  }

  /*!
   * \brief Copy constructor.
   *
   * This copy constructor not only works with Eigen matrices
   * but also with their expressions.
   */
  template <typename OtherDerived>
  Wrench(const Eigen::EigenBase<OtherDerived>& other) :
    Base(other)
  {
  }

  /*!
   * \brief Copy constructor.
   * 
   */
  Wrench(const SpatialVector<Scalar>& other)
  {
    CBase::operator=(other);
  }

  /*!
   * \brief Copy constructor form geometry_msgs::Wrench.
   */
  explicit Wrench(const geometry_msgs::Wrench& other)
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
  const Wrench& operator=(const Wrench& other)
  {
    Base::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment operator.
   */
  Wrench& operator=(const SpatialVector<Scalar>& other)
  {
    CBase::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment from geometry_msgs::Wrench.
   *
   */
  void operator=(const geometry_msgs::Wrench& W)
  {
    force() = W.force;
    moment() = W.torque;
  }

  /*!
   * \brief Conversion to geometry_msgs::Twist.
   */
  operator geometry_msgs::Wrench() const
  {
    geometry_msgs::Wrench W;
    W.force = force();
    W.torque = moment();
    return W;
  }

  /*!
   * \brief Conversion to geometry_msgs::Twist.
   */
  geometry_msgs::Wrench toWrenchMsg() const
  {
    return static_cast<geometry_msgs::Wrench>(*this);
  }

  /*!
   * \brief access to linear part
   */
  ForceRef<Base> force()
  {
    return CBase::linear();
  }

  /*!
   * \brief const access to linear part
   */
  ForceRef<const Base> force() const
  {
    return CBase::linear();
  }

  /*!
   * \brief access to angular part
   */
  MomentRef<Base> moment()
  {
    return CBase::angular();
  }

  /*!
   * \brief const access to angular part
   */
  MomentRef<const Base> moment() const
  {
    return CBase::angular();
  }
};

}  // namespace ow_core

#endif  // OPEN_WALKER_CORE_WRENCH_H
