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


#ifndef OPEN_WALKER_CORE_ROS_H
#define OPEN_WALKER_CORE_ROS_H

#include <ros/ros.h>
#include <ow_core/types.h>

namespace ow
{

/*!
 * \brief Publish a msg if there is a subsriber
 */
template<typename _Msg>
inline bool publish_if_subscribed(ros::Publisher& pub, const _Msg& msg)
{
  if(pub.getNumSubscribers())
  {
    pub.publish(msg);
    return true;
  }
  return false;
}

/*!
 * \brief Check if ros has parameter.
 */
inline bool has(const std::string& name, bool verbose=true)
{
  if(!ros::param::has(name)) 
  {
    if(verbose)
    {
      ROS_ERROR("has: parameter '%s' is missing.", name.c_str());
    }
    return false;
  }
  return true;
}

/*!
 * \brief load bool value
 */
inline bool load(const std::string& name, bool& val, bool verbose=true)
{
  if(!has(name, verbose))
  {
    return false;
  }
  if(!ros::param::get(name, val))
  {
    if(verbose)
    {
      ROS_ERROR("load: could not load '%s'.", name.c_str());
    }
    return false;
  }
  return true;
}

/*!
 * \brief load int value
 */
inline bool load(const std::string& name, int& val, bool verbose=true)
{
  if(!has(name, verbose))
  {
    return false;
  }
  if(!ros::param::get(name, val))
  {
    if(verbose)
    {
      ROS_ERROR("load: could not load '%s'.", name.c_str());
    }
    return false;
  }
  return true;
}

/*!
 * \brief load unsigned int value
 */
inline bool load(const std::string& name, size_t& val, bool verbose=true)
{
  int val_;
  if(!load(name, val_, verbose))
  {
    return false;
  }
  if(val_ < 0)
  {
    if(verbose)
    {
      ROS_ERROR("load: could not load '%s'.", name.c_str());
    }
    return false;
  }
  val = val_;
  return true;
}

/*!
 * \brief load scalar value
 */
inline bool load(const std::string& name, ow::Scalar& val, bool verbose=true)
{
  if(!has(name, verbose))
  {
    return false;
  }
  if(!ros::param::get(name, val))
  {
    if(verbose)
    {
      ROS_ERROR("load: could not load '%s'.", name.c_str());
    }
    return false;
  }
  return true;
}

/*!
 * \brief load string value
 */
inline bool load(const std::string& name, std::string& val, bool verbose=true)
{
  if(!has(name, verbose))
  {
    return false;
  }
  if(!ros::param::get(name, val))
  {
    if(verbose)
    {
      ROS_ERROR("load: could not load '%s'.", name.c_str());
    }
    return false;
  }
  return true;
}

/*!
 * \brief load std::vector value.
 * vector size is variable if force_dim=0
 */
inline bool load(
  const std::string& name, 
  std::vector<ow::Scalar>& val, 
  size_t force_dim = 0,
  bool verbose=true)
{
  if(!has(name, verbose))
  {
    return false;
  }
  if(!ros::param::get(name, val))
  {
    if(verbose)
    {
      ROS_ERROR("load: could not load '%s'.", name.c_str());
    }
    return false;
  }
  if(force_dim > 0)
  {
    if(val.size() != force_dim)
    {
      ROS_ERROR("load: '%s' has dim=%ld != desired=%ld", 
        name.c_str(), val.size(), force_dim);
      return false;
    }
  }
  return true;
}

/*!
 * \brief load generic eigen matrix value.
 * If fixed size: loaded dimension must match
 * If variable size and rows and cols specified: used as size
 * Else resized by loaded param.
 */
template<typename _Derived>
inline bool load(
  const std::string& name, 
  _Derived& val,
  typename _Derived::Index rows=Eigen::Dynamic,
  typename _Derived::Index cols=Eigen::Dynamic,
  bool verbose=true)
{
  typedef typename _Derived::Index Index;

  Index Rows, Cols, Size;
  Rows = _Derived::RowsAtCompileTime;
  Cols = _Derived::ColsAtCompileTime;
  Size = _Derived::SizeAtCompileTime;

  std::vector<ow::Scalar> v;
  if(Rows == Eigen::Dynamic || Cols == Eigen::Dynamic)
  {
    if(rows != Eigen::Dynamic && cols != Eigen::Dynamic)
    {
      // desired size specified
      Rows = rows;
      Cols = cols;
      Size = rows*cols;
    }

    // matrix is dynamic, resize to correct dimension
    if(!load(name, v, verbose))
    {
      return false;
    }
    if(Rows != Eigen::Dynamic && Cols != Eigen::Dynamic)
    {
      // desired size given
      val.resize(Rows, Cols);
      if(Size != v.size())
      {
        if(verbose)
        {
          ROS_ERROR("load: '%s' has dim=%ld != desired=%ld", 
            name.c_str(), v.size(), Size);
        }
        return false;
      }
    }
    else if(Rows == Eigen::Dynamic && Cols != Eigen::Dynamic)
    {
      // rows are free
      val.resize(v.size(), Cols);
    }
    else if(Cols == Eigen::Dynamic && Rows != Eigen::Dynamic)
    {
      // cols are free
      val.resize(Rows, v.size());
    }
    else
    {
      // default case
      val.resize(v.size(), 1);
    }
  }
  else
  {
    // matrix is fixed size, check dimension
    if(!load(name, v, Size, verbose))
    {
      return false;
    }
  }

  // store data
  val.setZero();
  for(size_t i = 0; i < val.size(); ++i)
  {
    val(i) = v[i];
  }
  return true;
}

}

#endif // OPEN_WALKER_CORE_ROS_H