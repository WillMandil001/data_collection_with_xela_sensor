#ifndef COMMON_DEFINES_H
#define COMMON_DEFINES_H

enum RES_COMMS
{
    RES_SUCCESS,
    RES_CANNOT_WRITE,
    RES_CANNOT_READ,
    RES_INVALID_DATA,
    RES_RECIEVED_GARBAGE
};


enum Decision{
  CLASS_UNDECIDED,
  CLASS_SOFT,
  CLASS_HARD,
  DECISION_NUM
};


enum SensorName
{
    ForceFing1,
    TorqueXFing1,
    TorqueYFing1,
    ForceFing2,
    TorqueXFing2,
    TorqueYFing2,
    ForceFing3,
    TorqueXFing3,
    TorqueYFing3,
    ProximityTipFing1,
    ProximityPadFing1,
    ProximityTipFing2,
    ProximityPadFing2,
    ProximityTipFing3,
    ProximityPadFing3,
    SensorNameNum
};

enum Fingers
{
  FINGER1,
  FINGER2,
  FINGER3,
  FINGERS_NUM
};

extern const int INVALID_DATA;  //to flag illegal data (defined in sensing_drivers)
extern const int NUM_INTIALISATION_VALS; //number of values to accumulate for calculating the bias and setting up the classifier (defined in sensing_drivers)

#endif // COMMON_DEFINES_H
