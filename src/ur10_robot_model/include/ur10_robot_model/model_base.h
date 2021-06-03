#ifndef MODEL_BASE_H_
#define MODEL_BASE_H_

#include <ros/node_handle.h>
#include <boost/shared_ptr.hpp>

namespace model_interface
{

/**
 * @brief Base class for all models.
 * Gives a generic interface that can be used weather or not 
 * 
 */
class ModelBase
{
public:
    enum State {CONSTRUCTED, INITIALIZED};

public:
    /** 
    * @brief constructor
    *
    * @param controller name
    */
    ModelBase(const std::string& name)
        : name_(name),
          state_(CONSTRUCTED) {}

    virtual ~ModelBase() {} 

    /** 
    * @brief init interal parameter
    *
    * @param ros nh for namespace
    */
    bool initRequest(ros::NodeHandle& nh) {
        if(state_ == INITIALIZED)
            return true;

        if(init(nh)) {
            state_ = INITIALIZED;
            return true;
        }
        return false;
    }

    /**
     * @brief return true if model initialized
     * 
     * @return true 
     * @return false 
     */
    bool isInitialized() const {
        return state_ == INITIALIZED;
    }

    /** 
    * @brief get the controller name
    *
    * @return controller name
    */
    std::string name() const { 
        return name_; 
    }

private:
    /** 
    * @brief init interal parameter
    *
    * @param ros nh for namespace
    */
    virtual bool init(ros::NodeHandle& nh) = 0;

private:
    State state_;
    std::string name_;
};

typedef boost::shared_ptr<model_interface::ModelBase> ModelBasePtr;
typedef boost::shared_ptr<const model_interface::ModelBase> ModelBaseConstPtr;

}

#endif