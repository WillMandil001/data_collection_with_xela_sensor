#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <exception>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>

#include "../include/squirrel_sensing_node/node.h"
#include "../include/squirrel_sensing_node/sensing_drivers.h"


using namespace ros;
using namespace std;


// The rate at which the sensors publish. 100 Hz seem enough.
const double SensingNode::pause=100.0;    //this might be became a constructor parameter

//consider instantiating everything in a configure() function instead of the constructor
SensingNode::SensingNode(const std::string& name, const std::vector<std::string>& portnames){

    this->name=name;

    prox_pub=(node.advertise<std_msgs::Float64MultiArray>("proximityShadow",1));    //1 is maximum number of messages sent before going in overflow

    loop_rate=new Rate(pause);

    sensor=new Tactile(portnames[ArduinoPort]);      //the port name is given from command line
}

SensingNode::~SensingNode(){

    if(sensor!=NULL){
        delete sensor;
    }

    if(loop_rate!=NULL){
        delete loop_rate;
    }

}

void SensingNode::run(){ //this function will make the node loop as long as ros::ok() is true

    //here can be added every function needed to set up the sensor execution (i.e. config file reading)

    sensor->flush();
    while(ros::ok()){
        //read from sensor

        //obatin proximity and tactile
        vector<double> vals;
        bool isValid=sensor->readData(vals);

        //obtain torque data
        Tactile* tac=dynamic_cast<Tactile*>(sensor);

        if(isValid && tac!=NULL && tac->isSensorInit()) //publish only if we read valid data
        {
            //prepare msg for prox
            vector<double> padProxs;
            padProxs.push_back(vals.at(ProximityPadFing1));    //sensor id 10
            padProxs.push_back(vals.at(ProximityPadFing2));    //sensor id 12
            padProxs.push_back(vals.at(ProximityPadFing3));    //sensor id 14

            //prepare msg for nine proximity Shadow
            vector<double> shadowProx;
            shadowProx.push_back(vals.at(ForceFing1));
            shadowProx.push_back(vals.at(TorqueXFing1));
            shadowProx.push_back(vals.at(TorqueYFing1));
            shadowProx.push_back(vals.at(ForceFing2));
            shadowProx.push_back(vals.at(TorqueXFing2));
            shadowProx.push_back(vals.at(TorqueYFing2));
            shadowProx.push_back(vals.at(ForceFing3));
            shadowProx.push_back(vals.at(TorqueXFing3));
            shadowProx.push_back(vals.at(TorqueYFing3));
            std_msgs::Float64MultiArray msgProx;
            msgProx.data.resize(shadowProx.size());
            //fill in msg for force/proximity
            for(int i=0;i<shadowProx.size();i++){
                msgProx.data[i]=shadowProx.at(i);
            }
            //--------
            prox_pub.publish(msgProx);

        }
        //if ready

        ros::spinOnce();

        loop_rate->sleep();
    }

}

string& SensingNode::getName(){
    return name;
}

