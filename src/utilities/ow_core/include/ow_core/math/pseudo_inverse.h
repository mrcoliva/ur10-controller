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

#ifndef OPEN_WALKER_CORE_MATH_PSEUDO_INVERSE_H
#define OPEN_WALKER_CORE_MATH_PSEUDO_INVERSE_H

#include <Eigen/Dense>

// the namespace for the project
namespace ow{

/*!
 * \brief Moore-Penrose Pseudo Inverse. With tol on singular values
 *
 * \param m
 *      Matrix,
 *
 * \param tol
 *      tol for singular values.
 *
 * \return
 *      inverse.
 */
template <class Derived>
Eigen::Matrix<typename Derived::Scalar, Derived::ColsAtCompileTime, Derived::RowsAtCompileTime>
inline pinvSVD(const Eigen::MatrixBase<Derived>& m, 
     typename Derived::Scalar tol = typename Derived::Scalar(1e-8))
{
  // general case
  Eigen::JacobiSVD<Derived> svd = 
    m.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

  double tolerance = tol*std::max(m.cols(), m.rows())
    * svd.singularValues().array().abs()(0);

  return svd.matrixV()
          * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal()
          * svd.matrixU().adjoint();
}

/*!
 * \brief Dampled Moore-Penrose Pseudo Inverse.
 *
 * \param m
 *      Matrix,
 *
 * \param lamda
 *      lamda for damping
 * 
 * \param tol
 *      tol when to apply damping on singular values.
 *
 * \return
 *      inverse.
 */
template <class Derived>
Eigen::Matrix<typename Derived::Scalar, Derived::ColsAtCompileTime, Derived::RowsAtCompileTime>
inline dampedPinvSVD(const Eigen::MatrixBase<Derived>& m, 
     typename Derived::Scalar lamda = typename Derived::Scalar(1e-8),
     typename Derived::Scalar tol = typename Derived::Scalar(1e-8))
{
  // general case
  Eigen::JacobiSVD<Derived> svd = 
    m.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

  return svd.matrixV()
          * (svd.singularValues().array().abs() > tol).select( 
              svd.singularValues().array().inverse(), 
              svd.singularValues().array()/(svd.singularValues().array()*svd.singularValues().array() + lamda)).matrix().asDiagonal()
          * svd.matrixU().adjoint();
}

/*!
 * \brief Moore-Penrose Pseudo Inverse
 * 
 * See: Fast Computation of Moore-Penrose Inverse Matrices 
 * Neural Information Processing
 *
 * \param m
 *      Matrix,
 *
 * \param tol
 *      tol for singular values.
 *
 * \return
 *      inverse.
 */
template <class Derived>
Eigen::Matrix<typename Derived::Scalar, Derived::ColsAtCompileTime, Derived::RowsAtCompileTime>
inline pinvGen(const Eigen::MatrixBase<Derived>& m, 
     typename Derived::Scalar tol = typename Derived::Scalar(1e-8))
{
  typedef typename Derived::Scalar Scalar;
  typedef typename Eigen::Matrix<Scalar, Derived::RowsAtCompileTime, Derived::RowsAtCompileTime> LeftMatrix;
  typedef typename Eigen::Matrix<Scalar, Derived::ColsAtCompileTime, Derived::ColsAtCompileTime> RightMatrix;
  typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> MatrixX;

  unsigned int rows = m.rows();
  unsigned int cols = m.cols();
  int cc;

  // general case
  if(rows < cols)
  {
    LeftMatrix a = m*m.transpose();
    Eigen::LDLT<LeftMatrix> ldlt(a);
    Eigen::LLT<LeftMatrix> llt(a);
    LeftMatrix l = llt.matrixL();

    for(cc=m.diagonal().size(); cc > 1; cc--) 
    {
      if(ldlt.vectorD().array().abs()(cc - 1) > tol) 
        break;
    }
    MatrixX c = l.block(0, 0, rows, cc);
    Eigen::ColPivHouseholderQR<MatrixX> qr(c.transpose()*c);
    return m.transpose()*c*qr.inverse()*qr.inverse()*c.transpose();
  }
  else
  {
    RightMatrix a = m.transpose()*m;
    Eigen::LDLT<RightMatrix> ldlt(a);
    Eigen::LLT<RightMatrix> llt(a);
    RightMatrix l = llt.matrixL();

    for(cc=m.diagonal().size(); cc > 1; cc--) 
    {
      if(ldlt.vectorD().array().abs()(cc - 1) > tol) 
        break;
    }
    MatrixX c = l.block(0, 0, cols, cc);
    Eigen::ColPivHouseholderQR<MatrixX> qr(c.transpose()*c);
    return c*qr.inverse()*qr.inverse()*c.transpose()*m.transpose();
  }
}

/*!
 * \brief Moore-Penrose Pseudo Inverse
 *
 * \param m
 *      Matrix,
 *
 * \param tol
 *      tol for singular values.
 *
 * \return
 *      inverse.
 */
template <class Derived>
Eigen::Matrix<typename Derived::Scalar, Derived::ColsAtCompileTime, Derived::RowsAtCompileTime>
inline pinv(const Eigen::MatrixBase<Derived>& m, 
     typename Derived::Scalar tol = typename Derived::Scalar(1e-8))
{
  // special case zero matrix
  if(m.isZero())
  {
    return m.transpose();
  }

  // special case vector
  if(m.cols() == 1 || m.rows() == 1)
  {
    return m.transpose()/m.squaredNorm();
  }

  // select svd method
  if(m.cols() < 7 && m.rows() < 7)
  {
    return pinvSVD(m, tol);
  }
  else
  {
    return pinvGen(m, tol);
  }
};

}

#endif