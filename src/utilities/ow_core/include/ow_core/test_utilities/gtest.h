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


#ifndef OPEN_WALKER_CORE_GTEST_UTILITIES_H
#define OPEN_WALKER_CORE_GTEST_UTILITIES_H

#include <gtest/gtest.h>
#include <type_traits>

#include <Eigen/Dense>

namespace ow_test
{

// testing precision
const double PRECISION = 1e-7;

// test Eigen::Matrix m1 and m2 for acceptable error bounds
template <typename _DerivedA,typename _DerivedB>
::testing::AssertionResult eigenMatrixNear(
  const Eigen::MatrixBase<_DerivedA>& m1, const std::string& m1_name, 
  const Eigen::MatrixBase<_DerivedB>& m2, const std::string& m2_name,
  double prec = PRECISION)
{
  if(m1.cols() != m2.cols()) 
  {
    return ::testing::AssertionFailure() 
      << "\n  Differnt number of cols for " << m1_name 
      << " and " << m2_name << ":\n  "
      << m1_name << ".cols()=" << m1.cols() << "\n  "
      << m2_name << ".cols()=" << m2.cols();
  }
  if(m2.rows()!= m2.rows()) 
  {
    return ::testing::AssertionFailure() 
      << "\n  Differnt number of rows for " << m1_name 
      << " and " << m2_name << ":\n"
      << m1_name << ".rows()=" << m1.rows() << "\n  "
      << m2_name << ".rows()=" << m2.rows();
  }
  if(((m1 - m2).array().abs() < prec).all())
  {
    return ::testing::AssertionSuccess();
  }
  else
  {
    ::testing::AssertionResult res(false); 
    res << "\n  Detla between Eigen::Matrix '" << m1_name 
        << "' and Eigen::Matrix '" << m2_name << "' detected:\n  ";

    for (int j = 0; j < m1.cols(); ++j) 
    {
      for (int i = 0; i < m1.rows(); ++i) 
      {
        double delta = std::abs(m1(i,j) - m2(i,j));
        if(delta > prec) {
          res << "At entry (" << i << "," << j << "):" << "\n    ";
          res << m1_name << "(" << i << "," << j << ")=" << m1(i,j) << "\n    ";
          res << m2_name << "(" << i << "," << j << ")=" << m2(i,j) << "\n    ";
          res << "Delta=" << delta << "\n  ";
        }
      }
    }
    res << "Precision set to prec=" << prec; 
    return res; 
  }
  return ::testing::AssertionFailure();
}

template <typename _DerivedA, typename _DerivedB>
::testing::AssertionResult eigenRotationNear(
  const Eigen::RotationBase<_DerivedA,3>& Q_a, const std::string& q_a_name,
  const Eigen::RotationBase<_DerivedB,3>& Q_b, const std::string& q_b_name,
  double prec = PRECISION)
{
  typedef typename Eigen::RotationBase<_DerivedA,3>::RotationMatrixType Matrix3A;
  typedef typename Eigen::RotationBase<_DerivedB,3>::RotationMatrixType Matrix3B;

  Matrix3A R_a = Q_a.toRotationMatrix();
  Matrix3B R_b = Q_b.toRotationMatrix();

  ::testing::AssertionResult res 
    = eigenMatrixNear(R_a, q_a_name, R_b, q_b_name, prec);
  if(!res)
  {
    ::testing::AssertionResult res_wrapped(false);
    res_wrapped << "\n  Detla between Eigen::RotationBase " 
                << q_a_name << " and Eigen::RotationBase " 
                << q_b_name << " detected:\n  "; 
    res_wrapped << res.message();
    return res_wrapped;
  }
  return res;
}

template <typename _ClassA, typename _ClassB>
::testing::AssertionResult isConvertible(
  const std::string& c_a_name,
  const std::string& c_b_name)
{
  if(std::is_convertible<_ClassA, _ClassB>())
  {
    ::testing::AssertionResult res(true);
    res << "\n    Conversion between class " << c_a_name
        << " and " << c_b_name << " possible\n  ";
    return res;
  }
  else
  {
    ::testing::AssertionResult res(false);
    res << "\n    Conversion between class " << c_a_name
        << " and " << c_b_name << " not possible\n  ";
    return res;
  }
  return ::testing::AssertionFailure();
}

template <typename _Base, typename _Derived>
::testing::AssertionResult isBaseOf(
  const std::string& base_name,
  const std::string& derived_name)
{
  if(std::is_base_of<_Base, _Derived>())
  {
    ::testing::AssertionResult res(true);
    res << "\n    Class " << derived_name
        << " is derived of " << base_name << "\n  ";
    return res;
  }
  else
  {
    ::testing::AssertionResult res(false);
    res << "\n    Class " << derived_name
        << " is not derived of " << base_name << "\n  ";
    return res;
  }
  return ::testing::AssertionFailure();
}



}

#endif // OPEN_WALKER_CORE_GTEST_UTILITIES_H