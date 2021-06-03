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


#ifndef OPEN_WALKER_CORE_SCALAR_POLYNOMIAL_H
#define OPEN_WALKER_CORE_SCALAR_POLYNOMIAL_H

#include <ow_core/trajectory/i_trajectory.h>

namespace ow_core{

/*!
 * \brief The Polynomial class.
 *
 * Takes the coefficients of a polynomial 
 * and offers function to evaluate it.
 * This class operates with scalar types only.
 * For multiple dimensions use polynomial_trajectory.
 * 
 * Polynomial coeffs are in the form: a0 + a1*x + a2*x^2 + ...
 * and stored as coeffs = [a0, a1, a2, ...]
 * 
 * Polynomials start at time_start = 0 and end at time period.
 * 
 */
template <typename _Scalar>
class Polynomial : 
  public ITrajectory<_Scalar, _Scalar>
{
public:
  typedef _Scalar Scalar;

  typedef std::vector<Scalar> ScalarVec;                // [time][Scalar]
  typedef std::vector<Scalar> ValueVec;                 // [time][value]
  typedef std::vector<ValueVec> ValueMatrix;            // [derivative][time][value]

  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorX;
  typedef std::vector<VectorX> VectorXVec;

public:

  /*!
   * \brief Builds a third order polynomial with time period.
   * 
   * The function passes through points x_start and x_end,
   * with an inital velocity of xP_start and a final velocity of xP_end.
   * 
   * Usage:
   * 
   *  ow_core::Polynomial poly3 = 
   *    ow_core::Polynomial<ow::Scalar>::Polynomial3Order(
   *      period, x_start, xP_start, x_end, xP_end);
   */
  static Polynomial Polynomial3Order(
    Scalar period,
    Scalar xP_start,    Scalar x_start,

    Scalar x_end,
    Scalar xP_end)
  {
    Eigen::Matrix<Scalar, 4, 4> A;
    Eigen::Matrix<Scalar, 4, 1> b;

    b <<
      x_start, xP_start, x_end, xP_end;
    A << 
      1, 0, 0, 0,
      0, 1, 0, 0,
      1, period, std::pow(period, 2), std::pow(period, 3),
      0, 1, 2*period, 3*std::pow(period, 2);
    VectorX c = A.householderQr().solve(b);
    return Polynomial(c, period);
  }

  /*!
   * \brief Builds a six order polynomial.
   * 
   * The function passes through points x_start and x_end,
   * with an inital velocity of xP_start and a final velocity of xP_end. 
   * The inital acceleration is xPP_start and the final acceleration xPP_end.
   * 
   * Usage:
   * 
   *  ow_core::Polynomial poly5 = 
   *    ow_core::Polynomial<ow::Scalar>::Polynomial5Order(
   *      period, x_start, xP_start, xPP_start, x_end, xP_end, xPP_end);
   */
  static Polynomial Polynomial5Order(
    Scalar period,
    Scalar x_start,
    Scalar xP_start,
    Scalar xPP_start,
    Scalar x_end,
    Scalar xP_end,
    Scalar xPP_end)
  {
    Eigen::Matrix<Scalar, 6, 6> A;
    Eigen::Matrix<Scalar, 6, 1> b;

    b <<
      x_start, xP_start, xPP_start, x_end, xP_end, xPP_end;
    A << 
      1, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0,
      0, 0, 2, 0, 0, 0,
      1, period, std::pow(period,2), std::pow(period,3), std::pow(period,4), std::pow(period,5),
      0, 1, 2*period, 3*std::pow(period,2), 4*std::pow(period,3), 5*std::pow(period,4),
      0, 0, 2, 6*period, 12*std::pow(period,2), 20*std::pow(period,3);
    VectorX c = A.householderQr().solve(b);
    return Polynomial(c, period);
  }

  /*!
   * \brief Builds a six order polynomial.
   * 
   * The function passes through points x_start, x_middle and x_end,
   * with an inital velocity of xP_start and a final velocity of xP_end. 
   * The inital acceleration is xPP_start and the final acceleration xPP_end.
   * 
   * Usage:
   * 
   * ow_core::Polynomial poly6 = 
   *  ow_core::Polynomial<ow::Scalar>::Polynomial6Order(
   *   period, x_start, xP_start, xPP_start, x_end, xP_end, xPP_end, x_middle);
   */
  static Polynomial Polynomial6Order(
    Scalar period,
    Scalar x_start,
    Scalar xP_start,
    Scalar xPP_start,
    Scalar x_end,
    Scalar xP_end,
    Scalar xPP_end,
    Scalar x_middle)
  {
    Eigen::Matrix<Scalar, 7, 7> A;
    Eigen::Matrix<Scalar, 7, 1> b;

    b <<
      x_start, xP_start, xPP_start, x_end, xP_end, xPP_end, x_middle;
    A << 
      1, 0, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0, 0,
      0, 0, 2, 0, 0, 0, 0,
      1, period, std::pow(period,2), std::pow(period,3), std::pow(period,4), std::pow(period,5), std::pow(period,6),
      0, 1, 2*period, 3*std::pow(period,2), 4*std::pow(period,3), 5*std::pow(period,4), 6*std::pow(period,5),
      0, 0, 2, 6*period, 12*std::pow(period,2), 20*std::pow(period,3), 30*std::pow(period,4),
      1, (period/2), std::pow(period/2,2), std::pow(period/2,3), std::pow(period/2,4), std::pow(period/2,5), std::pow(period/2,6);
    VectorX c = A.householderQr().solve(b);
    return Polynomial(c, period);
  }


protected:

  VectorX coeffs_;
  int degree_;
  int num_coeffs_;
  Scalar end_time_;

public:

  /*!
   * \brief Construct a polynomial.
   */
  Polynomial(const VectorX& coeffs, const Scalar& end_time) : 
    end_time_(end_time)
  {
    setCoefficients(coeffs);
  }

  /*!
   * \brief Copy constructor.
   */
  Polynomial(const Polynomial& other) :
    end_time_(other.end_time_)
  {
    setCoefficients(other.coeffs_);
  }

  /*!
   * \brief set the coefficients
   */
  void setCoefficients(const VectorX& coeffs)
  {
    coeffs_ = coeffs;
    degree_ = coeffs.size() - 1;
    num_coeffs_ = coeffs.size();
  }

  /*!
   * \brief get the coefficients
   */
  const VectorX& coefficients() const
  {
    return coeffs_;
  }

  /*!
   * \brief get end time of the trajectory
   */
  Scalar startTime() const
  {
    return 0.0;
  }

  /*!
   * \brief get end time of the trajectory
   */
  Scalar endTime() const
  {
    return end_time_;
  }

  /*!
   * \brief get the degree
   */
  int degree() const
  {
    return degree_;
  }

  /*!
   * \brief get the numm coeffs
   */
  int numCoeffs() const
  {
    return num_coeffs_;
  }

  /*!
   * \brief evaluate the polynomial at given value x
   * 
   * Uses Horner's Method
   * returns the value x
   */
  Scalar evaluate(const Scalar& x) const
  {
    Scalar v = Scalar(0);
    for(int i = 0; i <= degree_; ++i)
    {
      v = v*x + coeffs_[degree_-i];
    }
    return v;
  }

  /*!
   * \brief evaluate trajectory at at all times t_vec
   * 
   * \return A vector of values with the solution at all t in t_vec.
   */
  ScalarVec evaluate(const ScalarVec& t_vec) const
  {
    ScalarVec v_vec(t_vec.size());
    for(size_t i = 0; i < t_vec.size(); ++i)
    {
      v_vec[i] = evaluate(t_vec[i]);
    }
    return v_vec;
  }

  /*!
   * \brief evaluate the derivative of the polynomial at given value x
   * 
   * Uses Horner's Method
   * returns the d-th derivative at x
   */
  Scalar evaluateDiff(const Scalar& x, int d = 1) const
  {
    ValueVec values = evaluateAll(x, d);
    return values[0];
  }

  /*!
   * \brief evaluate the trajectory derivatives at all t in t_vec.
   * 
   * \return A vector of values with the d order derivative at time t
   */
  ScalarVec evaluateDiff(const ScalarVec& t_vec, int d = 1) const
  {
    ScalarVec v_vec(t_vec.size());
    for(size_t i = 0; i < t_vec.size(); ++i)
    {
      v_vec[i] = evaluateDiff(t_vec[i], d);
    }
    return v_vec;
  }

  /*!
   * \brief evaluate trajectory at a single time t and its derivatives up to d
   * returns vector with [derivative, derivative-1, ..., value]
   */
  ValueVec evaluateAll(const Scalar& x, int d = 1) const
  {
    ValueVec values(d+1, 0);

    // run horners method
    for(int i = 0; i <= degree_; ++i)
    {
      // d derivatives
      for(int j = 0; j < d-1; ++j)
      {
        values[j] = values[j]*x + values[j+1];
      }
      // value v
      values[d-1] = values[d-1]*x + values.back();
      values.back() = values.back()*x + coeffs_[degree_-i];
    }

    // multiply derivatives with correct pre factors
    for(int i = 2; i <= d; ++i)
    {
      for(int j = 0; j < i; j++)
      {
        values[j] *= i;
      }
    }
    return values;
  }

  /*!
   * \brief evaluate trajectory at all times t_vec and its derivatives up to d
   * returns vector with [time][derivative][value]
   */
  ValueMatrix evaluateAll(const ScalarVec& t_vec, int d = 1) const
  {
    ValueMatrix values_vec(t_vec.size());

    for(size_t i = 0; i < t_vec.size(); ++i)
    {
      values_vec[i] = evaluateAll(t_vec[i], d);
    }
    return values_vec;
  }
};

} // namespace ow_core

#endif // OPEN_WALKER_CORE_SCALAR_POLYNOMIAL_H
