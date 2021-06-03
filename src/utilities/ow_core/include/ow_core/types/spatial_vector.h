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


#ifndef OPEN_WALKER_CORE_SPATIAL_VECTOR_H
#define OPEN_WALKER_CORE_SPATIAL_VECTOR_H

#include <ow_core/utilities/type_guard.h>
#include <ow_core/type_bases/cartesian_base.h>

namespace ow{

/*!
 * \brief The traits class for the SpatialVector class.
 *
 * This class contains the typedefs and enums for
 * the SpatialVector class.
 *
 * These typedefs and enums are used to define and describe
 * the properties (traits) of this class.
 */
template<typename _Scalar>
struct traits<ow_core::SpatialVector<_Scalar> >
{
  typedef Eigen::Matrix<_Scalar, 6, 1> Base;
  typedef ow_core::Vector3Ref<Base> LinearRef;
  typedef ow_core::Vector3Ref<Base> AngularRef;
  typedef ow_core::Vector3Ref<const Base> CLinearRef;
  typedef ow_core::Vector3Ref<const Base> CAngularRef;
  enum {
    LinearPartAtCompileTime = 3,
    AngularPartCompileTime = 0,
  };
};

} // namespace ow

namespace ow_core
{
/*!
 * \brief The SpatialVector class.
 *
 * The SpatialVector is of type CartesianBase.
 * 
 * Can be used to assign Spatial Types our ow types.
 */
template <typename _Scalar>
class SpatialVector :
  public Eigen::Matrix<_Scalar, 6, 1>,
  public CartesianBase<SpatialVector<_Scalar> >,
  TypeGuard
{
  OW_TYPE_GUARD(SpatialVector)

public:
  typedef _Scalar Scalar;
  typedef Eigen::Matrix<_Scalar,6,1> Base;
  typedef CartesianBase<SpatialVector<Scalar> > CBase;

public:
  /*!
   * \brief Default Constructor.
   */
  SpatialVector() :
    Base()
  {
  }

  /*!
   * \brief Copy constructor.
   */
  SpatialVector(const SpatialVector& other) :
    Base(other)
  {
  }

  /*!
   * \brief Copy constructor.
   * 
   */
  SpatialVector(const CartesianVelocity<Scalar>& other)
  {
    CBase::operator=(other);
  }

  /*!
   * \brief Copy constructor.
   * 
   */
  SpatialVector(const CartesianAcceleration<Scalar>& other)
  {
    CBase::operator=(other);
  }

  /*!
   * \brief Copy constructor.
   * 
   */
  SpatialVector(const Wrench<Scalar>& other)
  {
    CBase::operator=(other);
  }

  /*!
   * \brief Copy constructor.
   * 
   */
  SpatialVector(const CartesianVector<Scalar>& other)
  {
    CBase::operator=(other);
  }

  /*!
   *  \brief Copy constructor.
   *
   * This copy constructor not only works with Eigen matrices
   * but also with their expressions.
   */
  template<typename OtherDerived>
  SpatialVector(const Eigen::EigenBase<OtherDerived>& other) :
    Base(other)
  {
  }

  /*!
   * \brief Use assignment operators of VBase class.
   */
  using Base::operator=;
  using CBase::operator=;

  /*!
   * \brief Assignment from SpatialVector.
   */
  SpatialVector& operator=(const SpatialVector& other)
  {
    Base::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment from SpatialVector.
   *
   */
  SpatialVector& operator=(const CartesianVelocity<Scalar>& other)
  {
    CBase::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment from SpatialVector.
   *
   */
  SpatialVector& operator=(const CartesianAcceleration<Scalar>& other)
  {
    CBase::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment from SpatialVector.
   *
   */
  SpatialVector& operator=(const Wrench<Scalar>& other)
  {
    CBase::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment from CartesianVector.
   *
   */
  SpatialVector& operator=(const CartesianVector<Scalar>& other)
  {
    CBase::operator=(other);
    return *this;
  }
};

}  // namespace ow_core


#endif  // OPEN_WALKER_CORE_VECTOR_DOF_H
