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


#ifndef OPEN_WALKER_CORE_ANGULAR_POSITION_H
#define OPEN_WALKER_CORE_ANGULAR_POSITION_H

#include <ow_core/utilities/type_guard.h>
#include <ow_core/type_references/angular_position_ref.h>
#include <ow_core/type_bases/angular_position_base.h>

#include <ow_core/math/random_orientation.h>
#include <ow_core/math/maps.h>

namespace ow{

/*!
 * \brief The traits class for the AngularPosition class.
 *
 * This class contains the typedefs and enums for
 * the AngularPosition class.
 *
 * These typedefs and enums are used to define and describe
 * the properties (traits) of this class.
 */
template<typename _Scalar>
struct traits<ow_core::AngularPosition<_Scalar> >
{
  typedef _Scalar Scalar;
  typedef int Index;

  enum {
    RowsAtCompileTime = 4,
    ColsAtCompileTime = 1,
    SizeAtCompileTime = 4,
  };
};

} // namespace ow

namespace ow_core
{

/*!
 * \brief The AngularPosition class.
 *
 * The AngularPosition is of type Eigen::Quaternion and is
 * represented by the math symbol \f$\mathbf{Q}\f$.
 */
template <typename _Scalar>
class AngularPosition : 
  public Eigen::Quaternion<_Scalar>,
  public AngularPositionBase<AngularPosition<_Scalar> >,
  TypeGuard
{
  OW_TYPE_GUARD(AngularPosition)

public:
  typedef _Scalar Scalar;
  typedef Eigen::Quaternion<_Scalar> Base;
  typedef AngularPositionBase<AngularPosition<_Scalar> > ABase;

  typedef typename Base::AngleAxisType AngleAxisType;

protected:

public:
  /*!
  * \brief Get Zero AngularPosition.
  * \note Zero AngularPosition is Identity Quaternion
  */
  static const AngularPosition& Zero()
  {
    static AngularPosition v = Base::Identity();
    return v;
  }

  /*!
  * \brief Construct a Angular Postion from an element in its tangential space.
  */
  static AngularPosition ExpMap(const ow_core::Vector3<Scalar>& omega)
  {
    return ow::expMapS3(omega);
  }

  /*!
  * \brief Generate a random uniformly sampled AngularPosition in S(3).
  * 
  * Usage: 
  *   std::random_device rd;
  *   ow::AngularPosition Q = ow::AngularPosition::Random(rd)
  */
  template<typename Generator>
  static AngularPosition Random(Generator& generator)
  {
    return ow::uniformRandomQuaternion<Scalar>(generator);
  }

public:
  /*!
   * \brief Default Constructor.
   */
  AngularPosition()
  {
  }

  /*!
   * \brief Copy constructor.
   */
  AngularPosition(const AngularPosition& other) :
    Base(other)
  {
  }

  /*!
   * \brief Assignment from Scalar values
   *
   * Internally the coefficients are stored in the following order:
   * [x, y, z, w]
   */
  AngularPosition(const Scalar& w, const Scalar& x, const Scalar& y, const Scalar& z) :
    Base(w,x,y,z)
  {
  }

  /*!
   * \brief Copy constructor.
   *
   * This copy constructor not only works with Eigen matrices
   * but also with their expressions.
   */
  template <typename OtherDerived>
  AngularPosition(const Eigen::MatrixBase<OtherDerived>& other) :
    Base(other)
  {
  }

  /*!
   * \brief Copy constructor from QuaternionBase.
   *
   */
  template <typename OtherDerived>
  AngularPosition(const Eigen::QuaternionBase<OtherDerived>& other) :
    Base(other)
  {
  }

  /*!
   * \brief Copy constructor.
   * 
   * Allows construction from AngleAxis.
   */
  explicit AngularPosition(const AngleAxisType& other) :
    Base(other)
  {
  }

  /*!
   * \brief Copy constructor.
   *
   * This copy constructor allows construction from
   * the reference class.
   */
  template <typename OtherDerived>
  AngularPosition(const AngularPositionRef<OtherDerived>& other) :
    Base(other)
  {
  }

  template <typename OtherDerived>
  explicit AngularPosition(const ow_core::Rotation3Base<OtherDerived>& other) :
    Base(other.derived())
  {
  }

  /*!
   * \brief Copy constructor from ow_core::Rotation3
   */
  explicit AngularPosition(const Rotation3<Scalar>& other) :
    Base(other)
  {
  }

  /*!
   * \brief Copy constructor from ow_core::Rotation3Ref
   */
  template <typename OtherDerived>
  explicit AngularPosition(const Rotation3Ref<OtherDerived>& other) :
    Base(other)
  {
  }

  /*!
   * \brief Use assignment operators of base class.
   */
  using Base::operator=;
  using ABase::operator=;

  /*!
   * \brief Assignment operator.
   */
  AngularPosition& operator=(const AngularPosition& other)
  {
    Base::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment operator.
   */
  template<typename OtherDerived>
  AngularPosition& operator=(const AngularPositionRef<OtherDerived>& other)
  {
    Base::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment operator.
   */
  AngularPosition& operator=(const Rotation3<Scalar>& other)
  {
    Base::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment operator.
   */
  template <typename OtherDerived>
  AngularPosition& operator=(const Rotation3Ref<OtherDerived>& other)
  {
    Base::operator=(other);
    return *this;
  }

};

}  // namespace ow_core

#endif  // OPEN_WALKER_CORE_ANGULAR_POSITION_H
