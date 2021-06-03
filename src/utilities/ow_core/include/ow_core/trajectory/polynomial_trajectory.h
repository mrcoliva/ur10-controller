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


#ifndef OPEN_WALKER_CORE_POLYNOMIAL_TRAJECTORY_H
#define OPEN_WALKER_CORE_POLYNOMIAL_TRAJECTORY_H

#include <ow_core/trajectory/piecewise_trajectory_base.h>
#include <ow_core/trajectory/polynomial.h>


namespace ow_core{

template<typename ValueType, typename ScalarType>
struct PolynomialTrajectoryTraits
{
  // specialize on matrix case: ValueTypeMod is original matrix type
  typedef ValueType ValueTypeMod;
};

template<typename ScalarType>
struct PolynomialTrajectoryTraits<ScalarType, ScalarType>
{
  // specialize on scalar case: ValueTypeMod is a 1x1 matrix
  typedef Eigen::Matrix<ScalarType,1,1> ValueTypeMod;
};


/*!
 * \brief The PiecewiseTrajectoryBase class.
 *
 * A PolynomialTrajectory is set of polynomial segements that form a Trajectory.
 * 
 * This class can handle Vector/Matrix and scalar trajectories.
 */
template<typename _Value, typename _Scalar>
class PolynomialTrajectory :
  public PiecewiseTrajectoryBase<_Value, _Scalar>
{
public:

  typedef _Scalar Scalar;
  typedef _Value Value;

  // this is eigher the original matrix type or a 1x1 matrix type
  // 1x1 matrix allows to overload functions, since c++ can seperate scalar from 1x1
  typedef typename PolynomialTrajectoryTraits<Value, Scalar>::ValueTypeMod ValueMod;
  typedef std::vector<ValueMod> ValueModVec;

  // polynomials are allways scalar
  typedef Polynomial<Scalar> PolynomialType;

  // result types
  typedef std::vector<Scalar> ScalarVec;                // [time][Scalar]
  typedef std::vector<Value> ValueVec;                  // [time][value]
  typedef std::vector<ValueVec> ValueMatrix;            // [derivative][time][value]

  // polynomials in spatial and time dimension 
  typedef std::vector<PolynomialType> PolynomialVec;    // [time]
  typedef std::vector<PolynomialVec> PolynomialMatrix;  // [dim][time]

  typedef PiecewiseTrajectoryBase<Value, Scalar> Base;

protected:
  PolynomialMatrix polynomials_;    // matrix of polynomials

public:

  /*!
   * \brief Constructor.
   * 
   * segments is a vector that contrains the start times of all trajectory
   * segemnts and the end time of the last segments
   * segemnts = [t_start, t_1, t_2, ..., t_end]
   * 
   * polynomials is a matrix of polynomials
   * with time segments in the first axis and spatial dim in the second
   * polynomials = [time segments][spatial dim]
   */
  PolynomialTrajectory(
    const ScalarVec& segments,
    const PolynomialMatrix& polynomials) :
    Base(segments),
    polynomials_(polynomials)
  {
    bool is_valid = Base::segments_.size()-1 == polynomials_.size();
    ROS_ASSERT_MSG(is_valid,
                  "segments size %ld must match polynomials size %ld",
                  Base::segments_.size()-1, polynomials_.size());
  }

  PolynomialTrajectory(
    const PolynomialVec& polynomials) :
    Base({0.0, polynomials[0].endTime()}),
    polynomials_({polynomials})
  {
    bool is_valid = Base::segments_.size()-1 == polynomials_.size();
    ROS_ASSERT_MSG(is_valid,
                  "segments size %ld must match polynomials size %ld",
                  Base::segments_.size()-1, polynomials_.size());
  }

  /*!
   * \brief Virtual destructor.
   */
  ~PolynomialTrajectory()
  {
  }

  /*!
   * \brief evaluate trajectory at a single time t
   */
  Value evaluate(const Scalar& t) const
  {
    Value val;

    // compute the segment index idx
    int idx = Base::computeSegmentIndex(t);

    // evaluate the segment at time t
    Scalar tt = enforceTimeRange(t, idx);
    evaluateSpecialized(tt, idx, val);
    return val;
  }

  /*!
   * \brief evaluate trajectory at at all times t_vec
   * 
   * \return A vector of values with the solution at all t in t_vec.
   */
  ValueVec evaluate(const ScalarVec& t_vec) const
  {
    ValueVec v_vec(t_vec.size());
    for(size_t i = 0; i < t_vec.size(); ++i)
    {
      // compute the segment index idx
      int idx = Base::computeSegmentIndex(t_vec[i]);

      // evaluate the segment at time t
      Scalar tt = enforceTimeRange(t_vec[i], idx);
      evaluateSpecialized(tt, idx, v_vec[i]);
    }
    return v_vec;
  }

  /*!
   * \brief evaluate the trajectory derivatives at a single time t.
   * 
   * \return The value of the d order derivative at time t
   */
  Value evaluateDiff(const Scalar& t, int d = 1) const
  {
    Value val;

    // compute the segment index idx
    int idx = Base::computeSegmentIndex(t);

    // evaluate the segment at time t
    Scalar tt = enforceTimeRange(t, idx);
    evaluateDiffSpecialized(tt, d, idx, val);
    return val;
  }

  /*!
   * \brief evaluate the trajectory derivatives at all t in t_vec.
   * 
   * \return A vector of values with the d order derivative at time t
   */
  ValueVec evaluateDiff(const ScalarVec& t_vec, int d = 1) const
  {
    ValueVec v_vec(t_vec.size());
    for(size_t i = 0; i < t_vec.size(); ++i)
    {
      // compute the segment index idx
      int idx = Base::computeSegmentIndex(t_vec[i]);

      // evaluate the segment at time t
      Scalar tt = enforceTimeRange(t_vec[i], idx);
      evaluateDiffSpecialized(tt, d, idx, v_vec[i]);
    }
    return v_vec;
  }

  /*!
   * \brief evaluate trajectory at a single time t and all of its 
   * derivatives up to d
   * 
   * \return Vector with [derivative][value]
   */
  ValueVec evaluateAll(const Scalar& t, int d = 1) const
  {
    ValueVec values;

    // compute the segment index idx
    int idx = Base::computeSegmentIndex(t);

    // evaluate the segment at time t
    Scalar tt = enforceTimeRange(t, idx);
    evaluateAllSpecialized(tt, d, idx, values);
    return values;
  }

  /*!
   * \brief evaluate trajectory at all times in t_vec and all 
   * of its derivatives up to d.
   * 
   * \return 2d vector with [derivative][time][value]
   */
  ValueMatrix evaluateAll(const ScalarVec& t_vec, int d = 1) const
  {
    ValueMatrix values_vec(t_vec.size());
    for(size_t i = 0; i < t_vec.size(); ++i)
    {
      // compute the segment index idx
      int idx = Base::computeSegmentIndex(t_vec[i]);

      // evaluate the segment at time t
      Scalar tt = enforceTimeRange(t_vec[i], idx);
      evaluateAllSpecialized(tt, d, idx, values_vec[i]);
    }
    return values_vec;
  }

private:
  /*!
   * \brief make sure time not exceeds the limits for the first and last segment
   */
  Scalar enforceTimeRange(const Scalar& t, int segment_idx) const
  {
    Scalar tt = t - Base::segments_[segment_idx];

    // first segment
    if(segment_idx == 0)
    {
      tt = std::max(tt, Base::startTime());
    }

    // last segment
    if(segment_idx == Base::numberOfSegments()-1)
    {
      tt = std::min(tt, Base::endTime());
    }
    return tt;
  }

  void evaluateSpecialized(const Scalar& tt, int idx, ValueMod& output) const
  {
    // for each dimension
    for(size_t i = 0; i < polynomials_[0].size(); ++i)
    {
      output(i) = polynomials_[idx][i].evaluate(tt);
    }
  }

  void evaluateSpecialized(const Scalar& tt, int idx, Scalar& output) const
  {
    output = polynomials_[idx][0].evaluate(tt);
  }

  void evaluateDiffSpecialized(const Scalar& tt, int d, int idx, ValueMod& output) const
  {
    // for each dimension
    for(size_t i = 0; i < polynomials_[0].size(); ++i)
    {
      output(i) = polynomials_[idx][i].evaluateDiff(tt, d);
    }
  }

  void evaluateDiffSpecialized(const Scalar& tt, int d, int idx, Scalar& output) const
  {
    output = polynomials_[idx][0].evaluateDiff(tt, d);
  }

  void evaluateAllSpecialized(const Scalar& tt, int d, int idx, ValueModVec& output) const
  {
    std::vector<Scalar> values;

    // resize to hold d+1 matrices
    output.resize(d+1);

    // for each dimension
    for(size_t i = 0; i < polynomials_[0].size(); ++i)
    {
      // compute the derivatives + the value
      std::vector<Scalar> values = polynomials_[idx][i].evaluateAll(tt, d);

      // sort this into the correct matrix elements
      // (values[j] in matrix[j] cell (i))
      for(size_t j = 0; j < values.size(); ++j)
      {
        output[j](i) = values[j];
      }
    }
  }

  void evaluateAllSpecialized(const Scalar& tt, int d, int idx, ScalarVec& output) const
  {
    // returns a vector of size d+1 with derivatives + value
    output = polynomials_[idx][0].evaluateAll(tt, d);
  }

};

} // namespace ow_core

#endif // OPEN_WALKER_CORE_POLYNOMIAL_TRAJECTORY_H
