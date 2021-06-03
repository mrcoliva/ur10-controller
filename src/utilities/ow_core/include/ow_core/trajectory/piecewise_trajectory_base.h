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


#ifndef OPEN_WALKER_CORE_PIECEWISE_TRAJECTORY_BASE_H
#define OPEN_WALKER_CORE_PIECEWISE_TRAJECTORY_BASE_H

#include <ow_core/trajectory/i_trajectory.h>
#include <ros/assert.h>

namespace ow_core{

/*!
 * \brief The PiecewiseTrajectoryBase class.
 *
 * The Base class for piecewise trajectories.
 * This Base class has functions for managing trajectories with multiple 
 * segments.
 * 
 * Manages the segments of a trajectory.
 * Segments are stored in a vector containing their start time.
 * The last element in the vector contains the endTime of the complete trajector
 * segemnts = [t_start, t_1, t_2, ..., t_end]
 * 
 */
template<typename _Value, typename _Scalar>
class PiecewiseTrajectoryBase :
  public ITrajectory<_Value, _Scalar>
{
public:
  typedef _Scalar Scalar;
  typedef std::vector<Scalar> ScalarVec;

protected:
  ScalarVec segments_;  //!< vector with segments start time and traj end time

public:

  /*!
   * \brief Constructor.
   * 
   * segments is a vector that contrains the start times of all trajectory
   * segemnts and the end time of the last segments
   * segemnts = [t_start, t_1, t_2, ..., t_end]
   */
  PiecewiseTrajectoryBase(const ScalarVec& segments) :
    segments_(segments)
  {
  }

  /*!
   * \brief Virtual destructor.
   */
  virtual ~PiecewiseTrajectoryBase()
  {
  }

  /*!
   * \brief get the number of segments.
   */
  int numberOfSegments() const
  {
    // size-1 since last index stores end time.
    return segments_.size()-1;
  }

  /*!
   * \brief get start time of a segment
   */
  Scalar startTime(int idx) const
  {
    segmentRangeCheck(idx);
    return segments_[idx];
  }

  /*!
   * \brief get end time of the trajectory
   */
  Scalar startTime() const
  {
    return segments_[0];
  }

  /*!
   * \brief get end time of a segment
   */
  Scalar endTime() const
  {
    return segments_.back();
  }

  /*!
   * \brief get end time of a segment
   */
  Scalar endTime(int idx) const
  {
    segmentRangeCheck(idx);
    if(idx < segments_.size()-1) 
    {
      return segments_[idx+1];
    }
    return endTime();
  }

  /*!
   * \brief compute the segment index that contains time t
   */
  int computeSegmentIndex(Scalar t) const
  {
    // check if we have a choice
    if(segments_.size() <= 2)
    {
      return 0;
    }

    // check if at outside of start and end time
    if(t < startTime())
    {
      return 0;
    }
    else if(t >= endTime())
    {
      return segments_.size()-1;
    }

    // compute the correct index
    return get_semgent_idx_recursive(t, 0, segments_.size()-1);
  }

protected:
  bool segmentRangeCheck(int idx) const
  {
    bool is_valid = (idx >= 0) && (idx < segments_.size());

    ROS_ASSERT_MSG(is_valid,
                  "Invalid segmend idx idx: 0 <= idx < %d",
                  segments_.size());
    return is_valid;
  }

  int get_semgent_idx_recursive(Scalar t, int start, int end) const
  {
    int mid = (end + start)/2;
    if(end - start <= 1)
      return start;

    if(t < segments_[mid])
    {
      return get_semgent_idx_recursive(t, start, mid);
    }
    if(t > segments_[mid])
    {
      return get_semgent_idx_recursive(t, mid, end);
    }
    return mid;
  }

};

} // namespace ow_core

#endif // OPEN_WALKER_CORE_I_PIECEWISE_TRAJECTORY_H
