#ifndef SENSING_NODE
#define SENSING_NODE

#include <string>
#include <vector>

#include "ros/ros.h"
#include "sensing_drivers.h"
#include "common_defines.h"

//comment below line to disable wrist sensor (in case it is not plugged in)
//#define _FT17_AVAIL

enum Param
{
    ArduinoPort,
	ParamNum
};




class SensingNode{

    static const double pause;   //period of operation (human-readable)

    ros::NodeHandle node;   //ros node
    ros::Publisher tactile_pub;  //publisher
    ros::Publisher proximity_pub;  //publisher
    ros::Publisher prox_pub;  //publisher for shadow - 3 values
    ros::Rate* loop_rate;    //execution frequency (for ros)

    Driver* sensor; //this variable is the concrete sensor being used

    std::string name;

public:
    SensingNode(const std::string& name, const std::vector<std::string>& portnames);
    ~SensingNode();

    void run(); //this function will make the node loop as long as ros::ok() is true

    //getters and setters
    std::string& getName();
};





#endif
