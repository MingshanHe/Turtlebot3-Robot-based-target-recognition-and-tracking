/**
 * @file NodeBase.hpp
 * @author mingshan(hemingshan_1999@163.com)
 * @brief Node based, provide base function
 * @date 2020-8-3
 * 
 * @copyright Copyright (c) 2020
 * 
**/
#ifndef __NODE_BASE_HPP__
#define __NODE_BASE_HPP__

#include <ros/ros.h>

/** @brief ROS Node base **/
class NodeBase
{
private:
    /* data */
public:
    NodeBase(int nArgc, char** ppcArgv, const char* pcNodeName)
    {
        // init current ROS node
        ros::init(nArgc, ppcArgv, pcNodeName);
        // generate current ROS node hundle
        NodeHandlePtr.reset(new ros::NodeHandle());
    }
    ~NodeBase(){};
    //Disable copy construction
    NodeBase(NodeBase& node)        = delete;
    NodeBase(const NodeBase& node)  = delete;

    //Realize the main loop of subclass tasks
    virtual void Run(void) = 0;
protected:
    std::unique_ptr<ros::NodeHandle> NodeHandlePtr; //The pointer for the NodeHandle
};//NodeBase
#endif

