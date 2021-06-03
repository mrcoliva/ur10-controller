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


#ifndef OPEN_WALKER_CORE_PARAMETER_H
#define OPEN_WALKER_CORE_PARAMETER_H

#include <ow_core/common/ros.h>
#include <memory>
#include <unordered_map>

namespace ow{

/*!
 * \brief The Entry Class
 */
class Entry
{
public:
  std::string name;
  std::string identifier;
  bool use_ns;

public:
  Entry(const std::string& identifier, bool use_ns=true) :
    identifier(identifier),
    use_ns(use_ns)
  {
  }

  virtual bool load(const std::string& ns) = 0;

  virtual std::string toString() = 0;
};

/*!
 * \brief The TypedEntry Class
 */
template<typename T>
class TypedEntry : public Entry 
{
public:
  T val;

public:
  TypedEntry(const std::string& identifier) :
    Entry(identifier)
  {
  }

  TypedEntry(const std::string& identifier, const T& val, bool use_ns=true) :
    Entry(identifier, use_ns),
    val(val)
  {
  }

  bool load(const std::string& ns)
  {
    bool ret;
    if(use_ns)
      ret = ow::load(ns + identifier, val);
    else
      ret = ow::load('/' + identifier, val);
    return ret;
  }

  std::string toString()
  {
    std::ostringstream out;
    out << val;
    return out.str();
  }
};

/*!
 * \brief The Parameter Class
 */
class Parameter
{
public:
  typedef std::unordered_map<std::string, std::shared_ptr<Entry> > Entries;
  typedef Entries::iterator EntriesIter;
  typedef Entries::const_iterator CEntriesIter;

protected:
  bool is_loaded_;
  std::string ns_;
  Entries entries_;
  
public:
  Parameter(const std::string& ns = "") : 
    ns_(prepareNs(ns)),
    is_loaded_(false)
  {
  }

  Parameter(const ros::NodeHandle& nh, const std::string& sub_ns = "") : 
    ns_(prepareNs(nh.getNamespace(), sub_ns)),
    is_loaded_(false)
  {
  }

  bool isLoaded() const 
  {
    return is_loaded_;
  }

  std::string nameSpace() const
  {
    return ns_;
  }

  template<typename T>
  bool add(const std::string& name, const T& val, bool use_ns=true)
  {
    std::string key = prepareName(name);

    EntriesIter it = entries_.find(key);
    if(it != entries_.end())
    {
      ROS_WARN("Parameter add: key '%s' already exsists, overwriting", name.c_str());
      it->second = std::make_shared<TypedEntry<T> >(key, val, use_ns);
    }
    else
    {
      entries_[key] = std::make_shared<TypedEntry<T> >(key, val, use_ns);
    }
    return true;
  }

  template<typename T>
  bool add(const std::string& name, bool use_ns=true)
  {
    std::string key = prepareName(name);

    EntriesIter it = entries_.find(key);
    if(it != entries_.end())
    {
      ROS_WARN("Parameter add: key '%s' already exsists, overwriting", name.c_str());
      it->second = std::make_shared<TypedEntry<T> >(key);
    }
    else
    {
      entries_[key] = std::make_shared<TypedEntry<T> >(key);
    }
    return true;
  }

  template<typename T>
  bool add(const std::vector<std::string>& names, bool use_ns=true)
  {
    bool ret = true;
    for(size_t i = 0; i < names.size(); ++i)
    {
      ret &= add<T>(names[i], use_ns);
    }
    return ret;
  }

  template<typename T>
  bool add(const std::vector<std::string>& names, const std::vector<T>& vals, bool use_ns=true)
  {
    bool ret = true;
    for(size_t i = 0; i < names.size(); ++i)
    {
      ret &= add<T>(names[i], vals[i], use_ns);
    }
    return ret;
  }

  bool load(const std::string& ns)
  {
    ns_ = prepareNs(ns);
    return load();
  }

  bool load(const ros::NodeHandle& nh, const std::string& sub_ns)
  {
    ns_ = prepareNs(nh.getNamespace(), sub_ns);
    return load();
  }

  bool load()
  {
    bool ret = true;
    for(EntriesIter it = entries_.begin(); it != entries_.end(); ++it)
    {
      ret &= it->second->load(ns_);
    }
    is_loaded_ = true;
    return ret;
  }

  template<typename T>
  bool get(const std::string& name, T& val) const
  {
    if(!is_loaded_)
    {
      ROS_ERROR("Parameter get: Load function was not called.");
      return false;
    }

    std::string key = prepareName(name);
    CEntriesIter it = entries_.find(key);
    if(it == entries_.end())
    {
      ROS_ERROR("Parameter get: Key '%s' not found.", name.c_str());
      return false;
    }
    else
    {
      std::shared_ptr<TypedEntry<T> > 
      ptr = std::dynamic_pointer_cast<TypedEntry<T> >(it->second);
      if(ptr)
      {
        val = ptr->val;
        return true;
      }
    }
    ROS_ERROR("Parameter get: Key '%s' found, but wrong template type.", name.c_str());
    return false;
  }

  template<typename T>
  T get(const std::string& name) const
  {
    T val = T();
    get(name, val);
    return val;
  }

  std::string toString()
  {
    std::ostringstream out;

    int i = 0;
    out << "[" << ns_ << "]:" << std::endl;
    for(EntriesIter it = entries_.begin(); it != entries_.end(); ++it)
    {
      out << "\t[" << it->first << "] = [" << it->second->toString() << "]\n";
      i++;
    }
    return out.str();
  }

protected:
  std::string prepareNs(const std::string& ns, const std::string& sub_ns = "") const
  {
    std::string ns_mod = ns;
    std::string sub_ns_mod = prepareName(sub_ns);

    if(ns_mod.empty())
      ns_mod.push_back('/');
    if(ns_mod.front() != '/')
      ns_mod.insert(ns_mod.begin(),'/');
    if(ns_mod.back() != '/')
      ns_mod.push_back('/');
    if(!ns.empty())
    {
      ns_mod.append(sub_ns_mod);
      ns_mod.push_back('/');
    }
    return ns_mod;
  }

  std::string prepareName(const std::string& name) const
  {
    std::string name_mod = name;
    if(!name_mod.empty())
    {
      while(name_mod.front() == '/')
        name_mod.erase(name_mod.begin());
      while(name_mod.back() == '/')
        name_mod.pop_back();
    }
    return name_mod;
  }
};

}

#endif // OPEN_WALKER_CORE_PARAMETER_H
