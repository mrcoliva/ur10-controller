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


#ifndef OPEN_WALKER_CORE_I_OUT_PORT_H
#define OPEN_WALKER_CORE_I_OUT_PORT_H

#include <ow_core/common/out_port.h>

namespace ow
{

class IOutPort
{
public:
  typedef std::shared_ptr<Port> PortPtr;
  typedef std::unordered_map<std::string, PortPtr> Ports;
  typedef Ports::iterator PortsIter;
  typedef Ports::const_iterator CPortsIter;

private:
  Ports ports_;
  std::string name_;

public:
  IOutPort(const std::string& name)
    : name_(name)
  {
  }

  virtual ~IOutPort()
  {
  }

  bool hasOutPort() const
  {
    return !ports_.empty();
  }

  size_t numOutPorts() const
  {
    return ports_.size();
  }

  PortPtr outPort(const std::string& name)
  {
    PortsIter it = ports_.find(prefix(name));
    if(it == ports_.end())
    {
      ROS_ERROR("IOutPort: '%s' has no port named '%s'.",
        name_.c_str(), name.c_str());
      return nullptr;
    }
    return it->second;
  }

  std::string printOutPort()
  {
    std::ostringstream out;
    out << name_ << ".outPorts = {\n";
    for(CPortsIter it = ports_.begin(); it != ports_.end(); ++it)
    {
      if(it->second->isValid())
        out << "\t" << it->second->name() << ": connected\n";
      else
        out << "\t" << it->second->name() << ": not connected\n";
    }
    out << "}\n";
    return out.str();
  }

protected:
  template<typename _T, class _M>
  bool registerOutPort(const std::string& name, const _T&(_M::*fp)() const, _M* obj)
  {
    ports_[prefix(name)] = std::make_shared<OutPort<_T> >(
      name, std::bind(fp, obj));
    return true;
  }

private:
  std::string prefix(const std::string& name) const
  {
    return "out:" + name;
  }
};

}

#endif // OPEN_WALKER_CORE_PORT_H