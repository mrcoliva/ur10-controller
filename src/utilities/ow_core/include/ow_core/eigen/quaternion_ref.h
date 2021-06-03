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


#ifndef OPEN_WALKER_CORE_EIGEN_QUATERNION_REF_H
#define OPEN_WALKER_CORE_EIGEN_QUATERNION_REF_H

#include <Eigen/Geometry>

// has to be in eigen namespace for QuaternionBase<...> to work with custom type QuaternionRef
namespace Eigen
{
// forward declaration
template <typename _Derived>
class QuaternionRef;

namespace internal
{
/*!
 * \brief The traits class for QuaternionRef.
 *
 * This class contains the typedefs and enums for
 * the Eigen::QuaternionRef class.
 *
 * These typedefs and enums are used to define and describe
 * the properties (traits) of the new class.
 * The templates of the Eigen library heavily depend on
 * the traits to properly handle the different type classes.
 *
 * \note Since the Eigen library uses the "Curiously Recurring Template Pattern"
 * (read https://eigen.tuxfamily.org/dox/TopicInsideEigenExample.html)
 * the new Eigen type class has to be defined in the Eigen namespace.
 */
template <typename _Derived>
struct traits<QuaternionRef<_Derived> >
{
  typedef QuaternionRef<_Derived> PlainObject;
  typedef typename _Derived::Scalar Scalar;
  typedef Eigen::Block<_Derived, 4, 1> Coefficients;

  enum
  {
    Alignment = internal::traits<Coefficients>::Alignment,
    Flags = LvalueBit
  };
};

}  // namespace internal

/*!
 * \brief The QuaternionRef class.
 *
 * This class provides access to the quaternion elements within
 * another Eigen type class. This Eigen type class has to provide/support
 * the Eigen::Block access.
 *
 * We need this special type to get the behavior of Eigen::Quaternion
 * in references where the source of the Quaternion elements is
 * not a Quaternion itsself.
 */
template <typename _Derived>
class QuaternionRef : public Eigen::QuaternionBase<QuaternionRef<_Derived> >
{
public:
  typedef _Derived Derived;
  typedef Eigen::QuaternionBase<QuaternionRef<_Derived> > Base;
  typedef typename Eigen::internal::traits<QuaternionRef>::Coefficients Coefficients;

protected:
  Coefficients coeffs_;  //!< The quaternion coefficients in a Eigen::Block.

public:
  /*!
   * \brief Default Constructor.
   *
   * \param ref
   *      the reference to storage Eigen object to access the
   *      elements of the quaternion via Eigen::Block.
   *
   * \param start_row
   *      the start index of the row for Eigen::Block.
   *
   * \param start_col
   *      the start index of the column for Eigen::Block.
   */
  QuaternionRef(Derived& ref, int start_row = 0, int start_col = 0) :
    coeffs_(ref, start_row, start_col)
  {
  }

  //    /*!
  //     *  \brief Copy constructor.
  //     */
  //    QuaternionRef(const QuaternionRef& other) :
  //        m_coeffs(other.coeffs())
  //    {}

  /*!
   * \brief Use assignment operators of base class.
   */
  using Base::operator=;

  /*!
   * \brief Get quaternion coefficients.
   *
   * This function is used internally by Eigen to
   * get the coefficients for the Quaternion
   * functionalities provided by Eigen::QuaternionBase.
   * 
   * \note coeffs are stored in the order [x, y, z, w]
   */
  Coefficients& coeffs()
  {
    return coeffs_;
  }

  /*!
   * \brief Get quaternion coefficients.
   *
   * This function is used internally by Eigen to
   * get the coefficients for the Quaternion
   * functionalities provided by Eigen::QuaternionBase.
   */
  const Coefficients& coeffs() const
  {
    return coeffs_;
  }

private:
  /*!
   * \brief No null reference.
   */
  QuaternionRef();
};

}  // namespace Eigen

#endif  // OPEN_WALKER_CORE_EIGEN_QUATERNION_REF_H
