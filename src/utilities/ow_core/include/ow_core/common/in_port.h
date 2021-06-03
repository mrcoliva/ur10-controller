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


#ifndef OPEN_WALKER_CORE_IN_PORT_H
#define OPEN_WALKER_CORE_IN_PORT_H

#include <ow_core/common/port.h>
#include <ow_core/common/out_port.h>

#include <functional>
#include <unordered_map>

namespace ow
{

template<typename _T>
class InPort : 
  public Port
{
public:
  typedef _T T;
  typedef std::function<const T&()> PortAccessFunction;

public:
  PortAccessFunction fnc_;    // function pointer
  T val_default_;             // default

public:
  InPort(const std::string& name) : 
    Port(name),
    fnc_(nullptr)
  {
  }

  InPort(const std::string& name, const T& val_default) : 
    Port(name),
    fnc_(nullptr),
    val_default_(val_default)
  {
  }

  virtual ~InPort()
  {
  }

  void set(PortAccessFunction fnc)
  {
    fnc_ = fnc;
  }

  bool connect(const std::shared_ptr<Port>& out)
  {

    if(!out->isValid())
    {
      ROS_ERROR("connect: OutPort '%s' not valid.", out->name().c_str());
      return false;
    }

    OutPort<T>* out_typed = dynamic_cast<OutPort<_T>*>(out.get());
    if(!out_typed)
    {
      ROS_ERROR("connect: Wrong template type for '%s' and '%s'.", 
        out->name().c_str(), Port::name().c_str());
      return false;
    }

    fnc_ = out_typed->get();
    Port::setValid(true);
    return true;
  }

  void setConstant(const T& val_default)
  {
    val_default_ = val_default;
    Port::setValid(true);
  }

  const T& call()
  {
    if(fnc_)
    {
      return fnc_();
    }
    else
    {
      return val_default_;
    }
  }
};

template<typename _T>
using InPortPtr = std::shared_ptr<ow::InPort<_T> >;

}

#endif // OPEN_WALKER_CORE_PORT_H