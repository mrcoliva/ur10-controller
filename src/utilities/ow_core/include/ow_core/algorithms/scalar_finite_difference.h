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


#ifndef OPEN_WALKER_CORE_SCALAR_FINITE_DIFFERENCE_H
#define OPEN_WALKER_CORE_SCALAR_FINITE_DIFFERENCE_H

#include <ros/assert.h>

#include <ow_core/algorithms/i_scalar_differentiation.h>
#include <ow_core/math.h>

namespace ow_core{

/*!
 * \brief The static ScalarFiniteDifference class.
 *
 * Additional information on the implemented algorithm:
 * - https://en.wikipedia.org/wiki/Finite_difference_coefficient
 * - http://web.media.mit.edu/~crtaylor/calculator.html
 *
 * Implements a finite difference numeric derivative using the
 * backward form.
 *
 * The order of the derivatrive \f$\mathScalar{d}\f$, and
 * the stencil \f$\mathVec{s}\f$ can be specified.
 *
 * Since we implement a backward differentiation the stencil must have only
 * integer coefficients \f$\mathScalarI{s}{i} \leq 0\f$.
 *
 * The order of accuracy is defined by the stencil length \f$\mathScalar{N}\f$.
 * The longer the stencil, the higher the accuracy. The stencil
 * length \f$\mathScalar{N}\f$ has to be greather than the order of
 * the derivative \f$\mathScalar{d}\f$: \f$\mathScalar{d} < \mathScalar{N}\f$.
 * 
 */
template <typename _Scalar>
class ScalarFiniteDifference :
    public IScalarDifferentiation<_Scalar>
{
public:
  typedef _Scalar Scalar;
  typedef IScalarDifferentiation<_Scalar> IDiff;
  typedef typename Eigen::Matrix<int, Eigen::Dynamic, 1> Stencil;
  typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector;
  typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;

public:

  /*!
   * \brief Construct first order derivative with accuracy two
   */
  static ScalarFiniteDifference FirstOrderAccurarcyTwo(Scalar f_s)
  {
    return ScalarFiniteDifference(f_s,1,2);
  }

  static ScalarFiniteDifference FirstOrderAccurarcyThree(Scalar f_s)
  {
    return ScalarFiniteDifference(f_s,1,3);
  }

  static ScalarFiniteDifference FirstOrderAccurarcyForth(Scalar f_s)
  {
    return ScalarFiniteDifference(f_s,1,4);
  }

  static ScalarFiniteDifference SecondOrderAccurarcyTwo(Scalar f_s)
  {
    return ScalarFiniteDifference(f_s,2,2);
  }

  static ScalarFiniteDifference SecondOrderAccurarcyThree(Scalar f_s)
  {
    return ScalarFiniteDifference(f_s,2,3);
  }

  static ScalarFiniteDifference SecondOrderAccurarcyForth(Scalar f_s)
  {
    return ScalarFiniteDifference(f_s,2,4);
  }

protected:
  bool is_init_;      //!< initalization flag
  bool is_valid_;     //!< valid filter ouput flag
  int fill_cnt_;      //!< counts buffer elements
  int order_;         //!< order
  int accuracy_;      //!< accuracy
  int size_;          //!< buffer size   
  Scalar h_;          //!< step width
  Scalar fac_;        //!< pre factor   
  Scalar y;           //!< filter ouput

  /*!
   * \brief The stencil. All coefficients greater than zero are
   * internally treated as negative coefficients.
   */
  Stencil s_;

  /*!
   * \brief The coefficients for the stencils.
   *
   * - \f$\mathScalarI{c}{i}\f$ corresponds to \f$\mathScalarI{s}{i}\f$, etc.
   * - \f$\mathScalarI{c}{i}\f$ is the coefficient for sample
   *        \f$\mathScalar{x}(\mathScalar{t} + \mathScalarI{s}{i} \,
   *            \mathScalar{h})\f$.
   */
  Vector c_;

  /*!
   * \brief The history of the signal used for the backward differences.
   *
   * - \f$\mathScalarI{x}{0}\f$: the current value, 0 steps into the past
   * - \f$\mathScalarI{x}{1}\f$: previous value, 1 step into the past
   * - \f$\mathScalarI{x}{\mathScalar{M}}\f$: \f$\mathScalar{M}\f$ steps
   *      into the past
   *
   * \f$\mathScalar{M}\f$ is the biggest absolute value of the stencil
   * coefficients \f$\mathScalarI{s}{i}\f$.
   */
  Vector x_;

public:
  /*!
   * \brief Constructor
   *
   * Construct a scalar finite difference differentiation for given
   * sampling frequency, derivative order and accuracy.
   *
   * \param f_sample
   *      the sample frequency \f$\mathScalarQ{f}{s}\f$.
   *      The step width \f$\mathScalar{h}\f$ is computed by
   *      \f$\mathScalar{h} = \frac{1}{\mathScalarQ{f}{s}}\f$.
   *
   * \param order
   *      the order of the derivative \f$\mathScalar{d}\f$.
   * 
   * \param accuracy
   *      the accuracy of the derivative \f$\mathScalar{a}\f$.
   */
  ScalarFiniteDifference(Scalar f_sample,
                         int order=1,
                         int accuracy=2) :
    h_(Scalar(1.0)/f_sample),
    order_(order),
    accuracy_(accuracy)
  {
    setup();
    reset();
  }

  /*!
   * \brief Constructor.
   *
   * Construct a scalar finite difference differentiation directly from given
   * Stencil vector.
   *
   * Since the stencil coefficients for backward differences are all
   * smaller or equal to zero \f$\mathScalarI{s}{i} \leq 0\f$ we enforce
   * <tt>unsigned int</tt>. All coefficients greater than zero are treated
   * internally as negative coefficients.
   *
   * \param fSample
   *      the sample frequency \f$\mathScalarQ{f}{s}\f$.
   *      The step width \f$\mathScalar{h}\f$ is computed by
   *      \f$\mathScalar{h} = \frac{1}{\mathScalarQ{f}{s}}\f$.
   *
   * \param stencil
   *      the stencil \f$\mathVec{s}\f$. All coefficients
   *      \f$\mathScalarI{s}{i} \geq 0\f$ are internally treated
   *      as negative coefficients. A stencil must not contain the same
   *      coefficient twice.
   *
   * \param order
   *      the order of the derivative \f$\mathScalar{d}\f$.
   */
  ScalarFiniteDifference(Scalar f_sample,
                         const Stencil& stencil,
                         int order = 1) :
    h_(Scalar(1.0)/f_sample),
    order_(order)
  {
    setup(stencil);
    reset();
  }


  /*!
   * \brief Copy constructor.
   */
  ScalarFiniteDifference(const ScalarFiniteDifference& other) :
    h_(other.h_),
    order_(other.order_),
    accuracy_(other.accuracy_),
    s_(other.s_)
  {
    setup(s_);
    reset();
  }

  /*!
   * \brief Destructor.
   */
  virtual ~ScalarFiniteDifference()
  {
  }

  /*!
   * \brief Returns whether the ouput is valid
   * 
   * Return true if algorithm accumulated enouth samples to fill buffer.
   */
  bool valid() const
  {
    return is_valid_;
  }

  /*!
   * \brief Returns num of samples required for valid ouput
   * 
   * Return number of samples required for valid ouput.
   */
  int validCount() const
  {
    return size_;
  }

  /*!
   * \brief Create a copy of this object using the new operator.
   *
   * Use this function to get a new object of the implemented
   * differentiation algorithm. This function returns a pointer
   * and the caller of this function has to manage the allocated
   * memory on the heap. When the object is no longer needed
   * the caller has to delete it.
   */
  IDiff* copy() const
  {
    return new ScalarFiniteDifference(*this);
  }

  /*!
   * \brief Set Sample Frequency
   *
   * Sample frequency is a paramer that applies to all time based algorithms
   * (Special case we consider here)
   */
  void setSampleFrequency(Scalar f_s)
  {
    h_ = Scalar(1.0)/f_s;
    setup();
    reset();
  }


  /*!
   * \brief Reset the algorithm to inital state.
   *
   * Ouput is initalized set in next call to update()
   */
  virtual Scalar reset()
  {
    is_init_ = false;
    is_valid_ = false;
    fill_cnt_ = 0;
    y = 0;

    return y;
  }

  /*!
   * \brief Reset the algorithm to inital state \f$\mathScalarQ{x}{}\f$.
   *
   * Algorithm uses \f$\mathScalarQ{x}{}\f$ as inital value.
   */
  Scalar reset(Scalar x)
  {
    is_init_ = false;
    is_valid_ = false;
    fill_cnt_ = 0;

    // init to user specific value now
    return update(x); 
  }

  /*!
   * \brief Adds new sample \f$\mathScalarQ{x}{}\f$ wihtout computation.
   * 
   * Could be used to populate buffer, before algorithm is started.
   */
  void add(Scalar x)
  {
    // initalize buffer if is not initalized
    if(!is_init_) 
    {
      x_.setConstant(x);
      is_init_ = true;
    }
    
    // check if next ouput is valid
    if(!is_valid_)
    {
      fill_cnt_++;
      if(fill_cnt_ >= size_)
      {
        is_valid_ = true;
      }
    }

    // add sample to buffer by shifting everything to the right
    Vector tmp = x_.head(x_.size()-1);
    x_.tail(x_.size()-1) = tmp;
    x_[0] = x;
  }

  /*!
   * \brief Update the differentiation for the next step.
   *
   * - Takes the new sample \f$\mathScalarQ{x}{new}\f$,
   * - Updates the history, and
   * - computes and updates the derivative.
   *
   * The algorithm is:
   *
   \f{align*}{
   \mathScalarOp{x}{(d)}(t) &= \mathVecOp{c}{\mathOpT} \,
      \mathRMatrix{
        \mathScalar{x}(\mathScalar{t} + \mathScalarI{s}{0} \, \mathScalar{h})\\
        \vdots \\
        \mathScalar{x}(\mathScalar{t} +
          \mathScalarI{s}{\mathScalar{N}-1} \, \mathScalar{h})\\
      }
      \, \frac{1}{\mathScalarOp{h}{\mathScalar{d}}}
   \f}
   *
   * \returns the newly calculated derivative.
   */
  Scalar update(Scalar x)
  {
    // add sample
    add(x);

    // filter
    y = fac_*c_.dot(x_);
    return y;
  }

  /*!
   * \brief The current value of the signal.
   */
  Scalar output() const
  {
    return y;
  }

  /*!
   * \brief the signal sampling frequency
   */
  Scalar sampleFrequency() const 
  {
    return Scalar(1.0)/h_;
  }

  /*!
   * \brief the step width
   */
  Scalar stepWidth() const 
  {
    return h_;
  }

  /*!
   * \brief the filters order
   */
  int order() const
  {
    return order_;
  }

  /*!
   * \brief the filters order
   */
  int accuracy() const
  {
    return accuracy_;
  }

private:

  /*!
   * \brief Default setup the finite coefficients.
   */
  void setup() 
  {
    size_ = order_ + accuracy_;

    // default stencil
    Stencil stencil = Stencil::LinSpaced(size_, 0, size_-1);

    // complete setup
    setup(stencil);
  }

  /*!
   * \brief Setup the finite coefficients.
   *
   * Computes the coefficients \f$\mathScalarI{c}{i}\f$ for the given stencil
   * \f$\mathVec{s}\f$ and order of the derivative \f$\mathScalar{d}\f$:

   \f{align*}{
   \mathVec{c} &= \mathRMatrix{
      \mathScalarI{c}{0}\\
      \vdots\\
      \mathScalarI{c}{\mathScalar{N}-1}
      } =
      \mathRMatrix{
        \mathScalarOpI{s}{0}{0} &
        \dots &
        \mathScalarOpI{s}{0}{\mathScalar{N}-1}\\
        \vdots &
        \ddots &
        \vdots\\
        \mathScalarOpI{s}{\mathScalar{N}-1}{0} &
        \dots &
        \mathScalarOpI{s}{\mathScalar{N}-1}{\mathScalar{N}-1}\\
      }^{-1} \,
      d! \,
      \mathRMatrix{
        \mathScalarI{\delta}{0,d}\\
        \vdots\\
        \mathScalarI{\delta}{N-1,d}
      }
      &&
      \textnormal{with} \quad \mathScalarI{\delta}{i,j} =
      \begin{cases}
        0 & i \neq j\\
        1 & i = j
      \end{cases}

   \f}
   *
   */
  void setup(const Stencil& stencil) 
  {
    // test if step size is corect
    bool isValidStepWidth = (h_ > 0.0);
    ROS_ASSERT_MSG(isValidStepWidth,
                "Invalid step width: h = %f",
                h_);

    // test if all coefficients in the stencil are unique
    std::vector<Stencil::Scalar> s(
          stencil.data(),
          stencil.data() + stencil.size());

    std::vector<Stencil::Scalar>::iterator it =
        std::unique(s.begin(),s.end());

    bool isUnique = (it == s.end());
    ROS_ASSERT_MSG(isUnique,
                  "Invalid stencil, contains equal entries: s = [%s]",
                  eigenToString(stencil.transpose()).c_str());

    // test if the stencil lenght fits to the desired order of the deriviative
    bool isValidConfig = (order_ < stencil.size());
    ROS_ASSERT_MSG(isValidConfig,
                  "Invalid configuration: stencil is too short:\n"
                  "d = %d < len(s = [%s]) = %d is not fulfilled.",
                  order_,
                  eigenToString(stencil.transpose()).c_str(),
                  int(stencil.size()));

    // save
    s_ = stencil;

    // setup sizes
    size_ = stencil.size();
    accuracy_ = size_ - order_;

    fac_ = Scalar(1.0)/std::pow(h_, order_);

    // Fill up the stencil matrix S
    // Each row is the stencil vector elementswise raised to the power of
    // the row number minus one.
    Matrix S(size_, size_);
    for(int i = 0; i < size_; ++i)
    {
      S.row(i) = s_.cast<Scalar>()*(-1.0);
      S.row(i) = S.row(i).array().pow(i);
    }

    // invert S
    Matrix S_inv = S.inverse();

    // compute the factorial of d
    Scalar d_fact = ow::factorial(order_);

    // Vector of Kronecker deltas: all entries are zero besides
    // the row where row-1 == d
    // and then multiply by dFact.
    Vector b = Vector(size_);
    b.setZero();
    b[order_] = d_fact;

    // now we can compute the coefficient vector via matrix vector multiplication
    c_ = S_inv*b;

    // setup memory buffer
    x_.resize(size_);
  }

};

} // namespace ow_core

#endif
