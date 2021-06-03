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


#ifndef OPEN_WALKER_CORE_MATRIX_ALGORITHM_BASE_H
#define OPEN_WALKER_CORE_MATRIX_ALGORITHM_BASE_H

#include <ros/assert.h>

#include <ow_core/utilities/type_not_assignable.h>
#include <ow_core/algorithms/i_scalar_algorithm.h>

namespace ow_core{

/*!
 * \brief The MatrixAlgorithm class.
 * 
 * This class provieds the base to apply Algorithms for all vectors 
 * and matrices by applying them elementwise.
 * The _Derived template parameter has to be derived from Eigen::Matrix.
 */
template <typename _Derived>
class MatrixAlgorithm
{
  OW_TYPE_NOT_ASSIGNABLE(MatrixAlgorithm)

public:
  enum
  {
    Rows = _Derived::RowsAtCompileTime,
    Cols = _Derived::ColsAtCompileTime,
  };

  typedef _Derived Derived;
  typedef typename Derived::Scalar Scalar;
  typedef typename Derived::Index Index;

  typedef IScalarAlgorithm<Scalar> IAlgorithm;

protected:
  std::vector<IAlgorithm*> algos_;  //!< vector of algos_.
  Derived y;                        //!< output

  Index rows_;            //!< The number of rows of the matrix
  Index cols_;            //!< The number of columns of the matrix
  Index size_;            //!< The size that is product of the rows and columns.

public:
  /*!
   * \brief Constructor for fixed size vectors and matrices.
   *
   * For now the size cannot be changed after construction.
   */
  MatrixAlgorithm(const IAlgorithm& algo)
  {
    bool is_valid = ((Rows != Eigen::Dynamic) && (Cols != Eigen::Dynamic));
    ROS_ASSERT_MSG(is_valid,
                   "Invalid constructor for matrix with dynamic rows or cols. "
                   "The size has to be known at construction.");

    rows_ = Rows;
    cols_ = Cols;
    size_ = Derived::SizeAtCompileTime;

    setup(algo);
  }

  /*!
   * \brief Constructor for dynamic vectors and partially dynamic matrices.
   *
   * For now the size cannot be changed after construction.
   */
  MatrixAlgorithm(
    Index size,
    const IAlgorithm& algo)
  {
    bool is_valid = ((Rows == Eigen::Dynamic) || (Cols == Eigen::Dynamic));
    ROS_ASSERT_MSG(is_valid,
                   "Invalid constructor. "
                   "The rows or cols must only be specified for matrices with "
                   "dynamic rows or cols.");
  
    rows_ = Rows;
    cols_ = Cols;

    if(Rows == Eigen::Dynamic)
    {
      rows_ = size;
    }

    if(Cols == Eigen::Dynamic)
    {
      cols_ = size;
    }

    size_ = rows_*cols_;

    setup(algo);
  }

  MatrixAlgorithm(
      Index rows,
      Index cols,
      const IAlgorithm& algo)
  {
    bool is_row_valid = (((Rows != Eigen::Dynamic) && (rows == Rows))
                       || Rows == Eigen::Dynamic);
    ROS_ASSERT_MSG(is_row_valid,
                   "Invalid constructor.");

    bool is_col_valid = (((Cols != Eigen::Dynamic) && (cols == Cols))
                       || Cols == Eigen::Dynamic);
    ROS_ASSERT_MSG(is_col_valid,
                   "Invalid constructor.");
    
    rows_ = rows;
    cols_ = cols;
    size_ = rows_*cols_;

    setup(algo);
  }

  /*!
   * \brief Deconstructor.
   */
  virtual ~MatrixAlgorithm()
  {
    cleanup();
  }

  /*!
   * \brief Returns whether the ouput is valid
   * 
   * Return true if algorithm accumulated enouth samples to fill buffer.
   */
  bool valid() const 
  {
    return algos_[0]->valid(); 
  }

  /*!
   * \brief Reset the algorithm to inital state.
   *
   * Use the current settings and start from scratch.
   * Sets input as inital value.
   */
  virtual const Derived& reset(const Derived& x)
  {
    for(size_t i = 0; i < algos_.size(); ++i)
    {
      y(i) = algos_[i]->reset(y(i));
    }
    return y;
  }

  /*!
   * \brief Reset the algorithm to inital state.
   *
   * Use the current settings and start from scratch.
   */
  virtual const Derived& reset()
  {
    for(size_t i = 0; i < algos_.size(); ++i)
    {
      y(i) = algos_[i]->reset();
    }
    return y;
  }

  /*!
   * \brief Adds new sample to algorithm
   *
   * Could be used to populate buffer, before algorithm is used by update.
   */
  virtual void add(const Derived& x)
  {
    for(size_t i = 0; i < algos_.size(); ++i)
    {
      algos_[i]->add(x(i));
    }
  }

  /*!
   * \brief Update the filter for the next step.
   * 
   */
  virtual const Derived& update(const Derived& x) 
  {
    for(size_t i = 0; i < algos_.size(); ++i)
    {
      y(i) = algos_[i]->update(x(i));
    }
    return y;
  }

protected:

  /*!
   * \brief Populate algos_ vector with a copy of input algorithm. 
   * 
   */
  virtual void setup(const IAlgorithm& algo)
  {
    algos_.resize(size_);
    for(size_t i = 0; i < algos_.size(); ++i)
    {
      algos_[i] = algo.copy();
    }
  }

  /*!
   * \brief Cleanup memory.
   * 
   */
  void cleanup()
  {
    for(size_t i = 0; i < algos_.size(); ++i)
    {
      delete algos_.at(i);
    }
    algos_.clear();
  }

};

}

#endif // OPEN_WALKER_CORE_MATRIX_ALGORITHM_BASE_H