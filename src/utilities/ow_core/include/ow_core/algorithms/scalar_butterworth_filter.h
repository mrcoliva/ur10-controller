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


#ifndef OPEN_WALKER_CORE_SCALAR_BUTTERWORTH_H
#define OPEN_WALKER_CORE_SCALAR_BUTTERWORTH_H

#include <ros/assert.h>
#include <ow_core/algorithms/i_scalar_filter.h>

namespace ow_core{

/*!
 * \brief The ScalarButterWorthFilter class.
 *
 * ScalarButterWorthFilter is a lowpass filter for scalar values with variable filter ORDER.
 * Implementation is based on: http://www.exstrom.com/journal/sigproc/
 *
 */
template <typename _Scalar>
class ScalarButterWorthFilter :
  public IScalarFilter<_Scalar>
{
public:
  typedef _Scalar Scalar;
  typedef IScalarFilter<_Scalar> IFilter;
  typedef typename Eigen::Matrix<_Scalar, Eigen::Dynamic, 1> Vector;
  typedef typename Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;

enum FilterType
{
  HP, //!< high pass
  LP, //!< low pass
  BP, //!< band pass
  BS, //!< band stop
};

public:
  static ScalarButterWorthFilter
    LowPassSecondOrder(Scalar f_s, Scalar f_cut)
  {
    return ScalarButterWorthFilter(f_s, f_cut, 0.0, 2.0, LP);
  }

  static ScalarButterWorthFilter
    LowPassForthOrder(Scalar f_s, Scalar f_cut)
  {
    return ScalarButterWorthFilter(f_s, f_cut, 0.0, 4.0, LP);
  }

  static ScalarButterWorthFilter
    HighPassSecondOrder(Scalar f_s, Scalar f_cut)
  {
    return ScalarButterWorthFilter(f_s, f_cut, 0.0, 2.0, HP);
  }

  static ScalarButterWorthFilter
    HighPassForthOrder(Scalar f_s, Scalar f_cut)
  {
    return ScalarButterWorthFilter(f_s, f_cut, 0.0, 4.0, HP);
  }

  static ScalarButterWorthFilter
    BandPassForthOrder(Scalar f_s, Scalar f_cut_1, Scalar f_cut_2)
  {
    return ScalarButterWorthFilter(f_s, f_cut_1, f_cut_2, 4.0, BP);
  }

  static ScalarButterWorthFilter
    BandPassEighthOrder(Scalar f_s, Scalar f_cut_1, Scalar f_cut_2)
  {
    return ScalarButterWorthFilter(f_s, f_cut_1, f_cut_2, 8.0, BP);
  }

  static ScalarButterWorthFilter
    BandStopForthOrder(Scalar f_s, Scalar f_cut_1, Scalar f_cut_2)
  {
    return ScalarButterWorthFilter(f_s, f_cut_1, f_cut_2, 4.0, LP);
  }

  static ScalarButterWorthFilter 
    BandStopEighthOrder(Scalar f_s, Scalar f_cut_1, Scalar f_cut_2)
  {
    return ScalarButterWorthFilter(f_s, f_cut_1, f_cut_2, 8.0, LP);
  }

protected:
  bool is_init_;                            //!< initalization flag
  bool is_valid_;                           //!< valid filter ouput flag
  int fill_cnt_;                            //!< counts buffer elements

  FilterType type_;                         //!< filter type
  int order_;                               //!< filter order
  int n_filters_;                           //!< number of filters
  int n_hist_;                              //!< number of element in history

  Scalar fcut_1_, fcut_2_;                  //!< cutoff frequencies
  Scalar f_s_;                              //!< sample frequency    

  Matrix A_, B_;                            //!< coeff matrix
  Matrix S_;                                //!< filter state

  Scalar y_;                                //!< filter ouput

public:
  /*!
   * \brief Construct a LP, HP, BP or BS filter for given sampling
   * frequency, cutoff frequencies and filter order.
   *    
   * \param f_sample
   *    Sample frequency of the signal
   *
   * \param fcut_1
   *    Cutoff frequency of the filter
   *    Sampling Theorem: fcut_1 < f_sample/2
   * 
   * \param fcut_2
   *    Second cutoff frequency of the filter
   *    Only applies to bandpass or bandpass filters
   *    Sampling Theorem: fcut_2 < f_sample/2 
   *
   * \param order
   *    Order of the filter. High order will introduce delay
   *  
   * \param type
   *    Type of filter, LP, HP, BW, BS
   */
  ScalarButterWorthFilter(Scalar f_s_,
                          Scalar fcut_1, 
                          Scalar fcut_2 = 0.0,
                          int order = 2, 
                          FilterType type = LP) : 
    f_s_(f_s_),
    fcut_1_(fcut_1),
    fcut_2_(fcut_2),
    order_(order),
    type_(type),
    y_(0.0)
  {
    setup();
    reset();
  }

  /*!
   * \brief Constructor a filter from a second-order sections form.
   * A Matrix containing the coefficients of multiple second-order filters.
   * Matrix dimension must be Nx6, with N the numer of second-order systems
   * and 6 the 3 nominator coefficients b + 3 denominator coefficients a.
   * 
   * See matlab function: sos = ss2sos(...)
   *    
   * \param A_sos
   *    Matrix of second-order sections
   */
  ScalarButterWorthFilter(const Matrix& A_sos,
                          Scalar f_s_ = 1.0,
                          Scalar fcut_1 = 0.0,
                          Scalar fcut_2 = 0.0,
                          int order = 0,
                          FilterType type = LP) : 
    f_s_(f_s_),
    fcut_1_(fcut_1),
    fcut_2_(fcut_2),
    order_(order),
    type_(type),
    y_(0.0)
  {
    setup(A_sos);
    reset();
  }

  /*!
   * \brief Copy constructor.
   * Copy Coeffecients, no need to recompute everything.
   */
  ScalarButterWorthFilter(const ScalarButterWorthFilter& other) :
    f_s_(other.f_s_),
    fcut_1_(other.fcut_1_),
    fcut_2_(other.fcut_2_),
    order_(other.order_),
    type_(other.type_),
    A_(other.A_),
    B_(other.B_),
    y_(0.0)
  {
    reset();
  }

  /*!
   * \brief Destructor.
   */
  virtual ~ScalarButterWorthFilter()
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
    return order_;
  }

  /*!
   * \brief Create a copy of this object using the new operator.
   */
  IFilter* copy() const
  {
    return new ScalarButterWorthFilter(*this);
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
    y_ = Scalar(0);
    return y_;
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
      initialize(x);
      is_init_ = true;
    }
    
    // check if next ouput is valid
    if(!is_valid_)
    {
      fill_cnt_++;
      if(fill_cnt_ >= order_)
      {
        is_valid_ = true;
      }
    }

    // in this algorithm: seperating add() and update() makes no sense,
    // since new value changes buffer state completely. So do complete update.
    y_ = filter(x);
  }

  /*!
   * \brief Update the filter for the next step.
   */
  Scalar update(Scalar x)
  {
    // add sample
    add(x);
    return y_;
  }

  /*!
   * \brief The current value of the signal.
   */
  Scalar output() const
  {
    return y_;
  }

  /*!
   * \brief the filters order
   */
  int order() const
  {
    return order_;
  }

  /*!
   * \brief the signal sampling frequency
   */
  Scalar sampleFrequency() const 
  {
    return f_s_;
  }

  /*!
   * \brief the filters cutoff frequency
   */
  Scalar lowerCutoffFrequency() const
  {
    return fcut_1_;
  }

  /*!
   * \brief the filters cutoff frequency
   */
  Scalar upperCutoffFrequency() const
  {
    return fcut_2_;
  }

protected:
  /*!
    * \brief Setup the filter for given filter type.
    * 
    * Resize buffer arrays and compute filter coefficients.
    */
  void setup() 
  {
    // check input
    bool is_valid_freq1 = (fcut_1_ >= 0.0) && (fcut_1_ <= f_s_/2.0);
    bool is_valid_freq2 = (fcut_2_ >= 0.0) && (fcut_2_ <= f_s_/2.0);
    ROS_ASSERT_MSG(is_valid_freq1&is_valid_freq2,
                  "Invalid cutoff freq: 0.0 <= f_cutoff < f_sample/2 = %f",
                  f_s_/2.0);

    // coorect frequencies
    if(fcut_1_ > fcut_2_)
    {
      Scalar tmp = fcut_2_;
      fcut_2_ = fcut_1_;
      fcut_1_ = tmp;
    }

    Scalar wc = std::tan(M_PI*(fcut_2_-fcut_1_)/f_s_);
    Scalar ac = std::cos(M_PI*(fcut_2_+fcut_1_)/f_s_)/
                std::cos(M_PI*(fcut_2_-fcut_1_)/f_s_);
  
    // setup filter coefficients
    switch (type_)
    {
      case HP:
        setupHighPass(wc);
        break;
      case BP:
        setupBandPass(wc, ac);
        break;
      case BS:
        setupBandStop(wc, ac);
        break;
      default:
        setupLowPass(wc);
        break;
    }
  }

  /*!
    * \brief Setup the filter from given sos matrix
    * 
    * Resize buffer arrays and compute filter coefficients.
    */
  void setup(const Matrix& A_sos)
  {
    bool is_valid = (A_sos.cols() == 6);
    ROS_ASSERT_MSG(is_valid,
                  "Invalid A_sos matrix: A_sos.cols()=%d but needs to be 6",
                  A_sos.cols());
    
    B_ = A_sos.leftCols(3);
    A_ = A_sos.RightCols(3);
    A_.col(0).setZero();
  }

  /*!
    * \brief Initialize the filter state.
    * 
    * Fill buffer arrays such that ouput is x in the first few iterations
    */
  void initialize(Scalar x)
  {
    // filter state is stored in matrix S_
    // State vector of i-th filter is stored in i-th colum
    n_filters_ = A_.rows();
    n_hist_ = A_.cols();
    S_.setZero(n_hist_, n_filters_);

    // init filter state S_ to ouput x in first few iterations
    Vector ones(A_.cols());
    ones.setOnes();
    ones[0] = 0.0;

    for(size_t i = 0; i < n_filters_; ++i) 
    {
      Scalar p = (1.0 - B_(i,0))/
        (B_.row(i).dot(ones) - B_(i,0)*A_.row(i).dot(ones));
      S_.col(i).setConstant(p*x);
    }
  }

  /*!
    * \brief Apply the filter algorithm.
    * 
    * Filter is implemented as a cascate of Direct form 2. 
    * With the circuit structure as in: 
    * https://en.wikipedia.org/wiki/Digital_biquad_filter
    */
  Scalar filter(Scalar x)
  {
    // update filter
    for(size_t i = 0; i < n_filters_; ++i) 
    {
      S_(0,i) = x - A_.row(i)*S_.col(i);
      x = B_.row(i)*S_.col(i);
    }

    // shift state
    Eigen::MatrixXd tmp = S_.topRows(n_hist_-1);
    S_.bottomRows(n_hist_-1) = tmp;
    return x;
  }

  /*!
    * \brief Setup coefficients for low pass filter
    */
  void setupLowPass(Scalar w)
  {
    int M = 2;              // sub filter order
    int N = order_/M;       // number of filters

    bool is_valid = (N > 0);
    ROS_ASSERT_MSG(is_valid,
                  "Invalid filter order=%d: order needs to be > 1",
                  order_);

    // store the coefficients of i-th filter in i-th row
    // of these two matrices 
    A_.setZero(N,M+1);
    B_.setZero(N,M+1);

    Scalar w2 = w*w;
    Scalar r, l, a0;
    for(int i = 0; i < N; ++i)
    {
      r = std::sin(M_PI*(2.0*i+1.0)/(4.0*N));
      l = w2 + 2.0*w*r + 1.0;
      a0 = w2/l;

      // denumerator coefficients i-th filter
      A_(i,1) = -2.0*(1-w2)/l;            
      A_(i,2) = (w2 - 2.0*w*r + 1.0)/l;

      // numerator coefficients i-th filter
      B_(i,0) = 1.0*a0;
      B_(i,1) = 2.0*a0;    
      B_(i,2) = 1.0*a0;
    }
  }

  /*!
    * \brief Setup coefficients for low pass filter
    */
  void setupHighPass(Scalar w)
  {
    int M = 2;                // sub filter order
    int N = order_/M;         // number of filters

    bool is_valid = (N > 0);
    ROS_ASSERT_MSG(is_valid,
                  "Invalid filter order=%d: order needs to be > 1",
                  order_);

    // store the coefficients of i-th filter in i-th row
    // of these two matrices 
    A_.setZero(N,M+1);
    B_.setZero(N,M+1);

    Scalar w2 = w*w;
    Scalar r, l, a0;
    for(int i = 0; i < N; ++i)
    {
      r = std::sin(M_PI*(2.0*i+1.0)/(4.0*N));
      l = w2 + 2.0*w*r + 1.0;
      a0 = 1.0/l;

      // denumerator coefficients i-th filter
      A_(i,1) = -2.0*(1-w2)/l;            
      A_(i,2) = (w2 - 2.0*w*r + 1.0)/l;

      // numerator coefficients i-th filter
      B_(i,0) = 1.0*a0;
      B_(i,1) = -2.0*a0;    
      B_(i,2) = 1.0*a0;
    }
  }

  /*!
    * \brief Setup coefficients for low pass filter
    */
  void setupBandPass(Scalar w, Scalar a)
  {
    int M = 4;            // sub filter order
    int N = order_/M;     // number of filters

    bool is_valid = (N > 0);
    ROS_ASSERT_MSG(is_valid,
                  "Invalid filter order=%d: order needs to be multiple of 4",
                  order_);

    // store the coefficients of i-th filter in i-th row
    // of these two matrices 
    A_.setZero(N,M+1);
    B_.setZero(N,M+1);

    Scalar w2 = w*w;
    Scalar a2 = a*a;
    Scalar r, l, a0;
    for(int i = 0; i < N; ++i)
    {
      r = std::sin(M_PI*(2.0*i+1.0)/(4.0*N));
      l = w2 + 2.0*w*r + 1.0;
      a0 = w2/l;

      // denumerator coefficients i-th filter
      A_(i,1) = -4.0*a*(1.0+w*r)/l;
      A_(i,2) = -2.0*(w2-2.0*a2-1.0)/l;
      A_(i,3) = -4.0*a*(1.0-w*r)/l;
      A_(i,4) = (w2 - 2.0*w*r + 1.0)/l;

      // numerator coefficients i-th filter
      B_(i,0) = 1.0*a0;
      B_(i,2) = -2.0*a0;    
      B_(i,4) = 1.0*a0;
    }
  }

  /*!
    * \brief Setup coefficients for low pass filter
    */
  void setupBandStop(Scalar w, Scalar a)
  {
    int M = 4;            // sub filter order
    int N = order_/M;     // number of filters

    bool is_valid = (N > 0);
    ROS_ASSERT_MSG(is_valid,
                  "Invalid filter order=%d: order needs to be multiple of 4",
                  order_);

    // store the coefficients of i-th filter in i-th row
    // of these two matrices 
    A_.setZero(N,M+1);
    B_.setZero(N,M+1);

    Scalar w2 = w*w;
    Scalar a2 = a*a;
    Scalar r, l, a0;
    for(int i = 0; i < N; ++i)
    {
      r = std::sin(M_PI*(2.0*i+1.0)/(4.0*N));
      l = w2 + 2.0*w*r + 1.0;
      a0 = w2/l;

      // denumerator coefficients i-th filter
      A_(i,1) = -4.0*a*(1.0+w*r)/l;
      A_(i,2) = -2.0*(w2-2.0*a2-1.0)/l;
      A_(i,3) = -4.0*a*(1.0-w*r)/l;
      A_(i,4) = (w2 - 2.0*w*r + 1.0)/l;
      
      // numerator coefficients i-th filter
      B_(i,0) = 1.0*a0;
      B_(i,1) = -4.0*a*a0;
      B_(i,2) = (4.0*a2+2.0)*a0;
      B_(i,3) = -4.0*a*a0;
      B_(i,4) = 1.0*a0;
    }
  }
};

} // namespace ow_core

#endif
