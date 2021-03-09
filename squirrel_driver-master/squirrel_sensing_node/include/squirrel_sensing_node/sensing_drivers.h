#ifndef SENSING_DRIVERS
#define SENSING_DRIVERS

#include <string>
#include <vector>
#include <queue>
#include "common_defines.h"



class Driver{   //abstract class: everything in common across sensors is here
//nothing here (no private data)
protected:
    std::string m_portname; //name of the arduino port
    int m_fileDesc;         //file descriptor for termios
    std::vector<double> m_sensor_values;

    static const int NUM_PROX; //num of proximity sensors
    static const int NUM_VALS; //num of readings
    static const double MAX_VOLTS;  //beyond this value does not make sense
    static const int MAX_RETRIES;

    //config matrix (filename?)

    virtual bool setup();   //assuming setup is equal for 2 sensors on 3


public:
    virtual bool readData(std::vector<double>&)=0;  //this function reads the data from the sensors and returns a vector (double[][])
    virtual void flush();
    RES_COMMS arduRead(std::vector<double>& data);

private:
    static const char CMD_GETDATA[5];  //command to fetch data from arduino


};

//------------------------------


class Tactile : public Driver{

    // calibration coefficients
    //---
    static const double MAX_PROX; //proximity

    static const double A_PROX;
    static const double B_PROX;
    static const double C_PROX;
    static const double D_PROX;

    static const int NUM_HISTORY_VALS; //number of readings to use for stationary check
    static const double STATIONARY_PROXIMITY_THREASHOLD; //voltage threashold, if passed data is stationary

    std::vector<double> divider; //vecotr containing voltage divider numbers read from file
    std::vector<double> history_val_prox; //sum previous voltage values
    std::vector<std::queue<double>* > history_prox; //list of previous voltage values
    std::vector<std::vector<double> > mean; //first 10 values used for calculating the bias
    std::vector<double> m_accumulator_fing;	//numerators of the mean
    std::vector<double> m_biases;   //biases for the arduino readings, 1 value per sensor
    std::vector<double> m_lastLegals;   //list of last legal values
    static std::vector<double> m_maximumForce;  //vector containing the maximum force values for each tactile sensor (3x1) //made static for brevity

    int m_divider;              //counts the number of samples read for flattening the torque
    //those three guys are used to discriminate which components of the driver are initialised
    bool m_isBiased;
    bool m_hasHistoryProx;

    double bias(const int idx,const double val);
    double convertProx(const double num);
    bool isStationaryProx(const double val,const int idx);
    void patchData(std::vector<double>& data);
    void updateLegals(std::vector<double>& data);

public:
    Tactile(const std::string& portname);
    ~Tactile();
    virtual bool readData(std::vector<double>&);
    std::vector<double>& readTorquePerc();
    bool isSensorInit() const;  //returns true if all the components of the sensor are initialised

    //normalises a force reading, throws runtime error if cannot be done yet
    static std::vector<double> normaliseForce(const std::vector<double>& force);
};

#endif
