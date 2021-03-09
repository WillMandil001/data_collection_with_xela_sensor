#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <cmath>
#include <assert.h>
#include <ros/package.h>
#include <ros/ros.h>


#include "../include/squirrel_sensing_node/sensing_drivers.h"

//#define TEST    //undefine to remove testing code

using namespace std;

const char Driver::CMD_GETDATA[]="g"; //arduino command to get data
const int INVALID_DATA=-100;
const int NO_SIGNAL=-100;
const int Driver::MAX_RETRIES=5;   //we try 5 times to read
const double Driver::MAX_VOLTS=5.0;

//makes the connection with m_portname (e.g. tty0), communication functions start here
bool Driver::setup(){    //assuming setup is equal for 2 sensros on 3

    m_fileDesc=open(m_portname.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    fcntl(m_fileDesc, F_SETFL,0);//reset file status flags

    /* Set up the control structure (the port speed etc.)*/
     struct termios toptions;
     /* Get currently set options for the tty */
     tcgetattr(m_fileDesc, &toptions);
     /* 9600 baud */
      cfsetispeed(&toptions, B38400);   //enter here B38400 for max baudrate for this system, arduino must match this speed, B9600 was the previous, slowest, value which was proven to work in the larger system
      cfsetospeed(&toptions, B38400);   //enter here B38400 for max baudrate for this system, arduino must match this speed (both lines must have the same speed set here)
      /* 8 bits, no parity, no stop bits */
       toptions.c_cflag &= ~PARENB;
       toptions.c_cflag &= ~CSTOPB;
       toptions.c_cflag &= ~CSIZE;
       toptions.c_cflag |= CS8; //should be one stop bit
       /* Canonical mode */
        toptions.c_lflag |= ICANON;

    /* saves the options on the PC port */
     tcsetattr(m_fileDesc, TCSANOW, &toptions);
     /* Flush anything already in the serial buffer */
     tcflush(m_fileDesc, TCIFLUSH);

     return true;
}

//forces a message to be sent to the PC port
void Driver::flush(){
    /* Flush anything already in the serial buffer */
    tcflush(m_fileDesc, TCIFLUSH);
}

//handles the communication with the arduino, the communication protocol
//(how fast a response must be and which response is expected) is implemented here
RES_COMMS Driver::arduRead(vector<double>& data)
{
    const int CMD_BUF_LEN=3;
    char rpy_buff[255];

    fd_set read_fds, write_fds, except_fds;
    FD_ZERO(&read_fds);  //initialises read and write buffers
    FD_ZERO(&write_fds);    //if we want to check if we can write, we have to add to this set
    FD_ZERO(&except_fds);   //we don't care about communication errors
    FD_SET(m_fileDesc, &read_fds);  //monitor stream for reading
    // Set timeout to 1.5 seconds
    struct timeval timeout;
    timeout.tv_sec = 1;     //wait for 1.5 seconds
    timeout.tv_usec = 500000;// this means 0.5 seconds

    tcflush(m_fileDesc, TCIFLUSH);	//cleans up the write buffer
  //decomment for more reliable communication (request/reply style)

    //write a request to GETDATA to the PC communication buffer to the arduino
    ssize_t wrbytes=write(m_fileDesc,CMD_GETDATA,CMD_BUF_LEN);
    if(wrbytes!=CMD_BUF_LEN)  //we couldn't write
    {
        cout << "Driver::arduRead> ERROR! Couldn't issue request to arduino" << endl;
        for(uint i=0;i<NUM_VALS;++i)
        {
            data.push_back(INVALID_DATA);   //return failed data
        }
        return RES_CANNOT_WRITE;
    }


    //read response from arduino
    ssize_t rdbytes=-2; //termios returns -1 if error
    //"first argument of select is the highest-numbered file descriptor in any of the three sets, plus 1." (cit.)
    if (select(m_fileDesc + 1, &read_fds, &write_fds, &except_fds, &timeout) > 0)   //checks if is possible to read
    {
        rdbytes=read(m_fileDesc,rpy_buff,255);
    }

    if(rdbytes<=0)  //either we couldn't read or there was an error
    {
        cout << "Driver::arduRead> ERROR! Couldn't read anything from arduino or error while reading: "  << (rdbytes==-2? "no data" : "read error") << endl;
        for(uint i=0;i<NUM_VALS;++i)
        {

            data.push_back(INVALID_DATA);
        }
        return RES_CANNOT_READ;
    }
    rpy_buff[rdbytes]='\0'; //now we have a string as we appended the "string terminator"

    //parse the received string (makes sense out of it)
    stringstream parser(rpy_buff);
    RES_COMMS result=RES_SUCCESS;
    double val=0.0;




    //reads a string and breaks it in pieces (e.g. "101 12 1" -> "101", "12", "1")
    while(parser >> val || !parser.eof())
    {
        if(parser.fail())   //if we received garbage due to loss of sync
        {

            parser.clear(); //clear error
            string alien;
            parser >> alien;
            if(alien!="\n" || alien!="\r")  //arduino sends this all the time
            {
                cout << "WARNING: received surprised: " << alien << endl;
            }
            continue;
        }

        //converts a sensor gain in volts, this is what we care about
        val=val*(5.0 / 1023.0);

        if(val>MAX_VOLTS || val <=0)
        {

            val=INVALID_DATA;
            result=RES_INVALID_DATA; //there was a parsing error
        }
        data.push_back(val); //conversion gain to volts
 
    }

    return result;

}

//------------------------------TACTILE and PROXIMITY

const int Driver::NUM_PROX=15;
const int Driver::NUM_VALS=15;  //as many as proximity for now

const int Tactile::NUM_HISTORY_VALS=10;
const int NUM_INTIALISATION_VALS=50;
const double Tactile::STATIONARY_PROXIMITY_THREASHOLD=0.02;    //volts


//calibration coefficients proximity sensor
const double Tactile::A_PROX=20.74;
const double Tactile::B_PROX=-0.1808; //exponent
const double Tactile::C_PROX=-17.28;

//calibration maximum for proximity
const double Tactile::MAX_PROX=2.5;

std::vector<double> Tactile::m_maximumForce;



Tactile::Tactile(const std::string& portname){

    m_portname=portname;
    m_divider=0;	//we shall never divide by 0
    //for(int i=0;i<(3*2);++i)
   // {
    //	m_accumulator_fing.push_back(0.0);	//stores numerators of the mean for each finger and torque sensor
    //}

    //sensor is uninitialised at the beginning
    m_isBiased=false;
    m_hasHistoryProx=false;

    //initialising history

    for(int i=0;i<NUM_PROX;i++)
    {
        history_val_prox.push_back(0);
        history_prox.push_back(new queue<double>());
        mean.push_back(vector<double>() );
        m_biases.push_back(0);
        m_lastLegals.push_back(0);
    }


    //read from conf.ini the divider values
    string filepath=ros::package::getPath("squirrel_sensing_node");
    filepath+="/tactile_calibration.ini";
    ifstream config(filepath.c_str());
    bool isReadMaximums=false;  //if more items will be added to the ini file, this will became an enum
    if(config.good()){
        string line;
        int sensId=0;
        while(getline(config,line) && sensId<=NUM_VALS){    //one divider per sensor reading

            if(line.at(0)=='@') //this caracther changes the parsing
            {
              isReadMaximums=true;
            }

            if(!isReadMaximums && line.length()!=0 && line.at(0)!='#' && line.at(0)!='@'){ //if line is not a comment or blank
                istringstream iss(line);
                double val;
                iss >> val;                         //parsing dividers
                divider.push_back(val);
                sensId++;

            }
            else if(isReadMaximums && line.length()!=0 && line.at(0)!='#' && line.at(0)!='@'){ //if line is not a comment or blank
                istringstream iss(line);
                double val;                         //parsing maximums
                iss >> val;
                static int torqueCnt=0;
                if(torqueCnt<6) //cross fingers...
                {
                    //TODO: not clear what's happening here, maybe is useless stuff now
                    torqueCnt++;
                }
                else
                {
                    m_maximumForce.push_back(val);
                }
            }
        }

    }else{
        ROS_INFO("ERROR: Could not locate file config.ini, assumed no dividers (==1)");
        for(int i=0;i<NUM_VALS;++i){
            divider.push_back(1);
        }
    }

    setup();

}

Tactile::~Tactile(){

    close(m_fileDesc);


    for(int i=0;i<NUM_PROX;i++)
    {

        delete history_prox[i];
    }
}



bool Tactile::readData(std::vector<double>& res)    //we pass a NULL pointer, but we could just give a reference{
{
    res=vector<double>(NUM_VALS,INVALID_DATA); //assume we read only garbage
    bool readRes=false;

    char buff[255];
#ifndef TEST

    vector<double> vals;

    RES_COMMS commsRes=RES_CANNOT_WRITE;
    for(uint i=0;i<MAX_RETRIES && commsRes!=RES_SUCCESS;++i)
    {
        commsRes=arduRead(vals);
        if(commsRes!=RES_SUCCESS){vals.clear();}


    }

    if(commsRes!=RES_SUCCESS && commsRes!=RES_INVALID_DATA )
    {
        cout << "FAIL: Maximum of number of attempts to read from arduino reach, returning failed data" << endl;
    }

    switch(commsRes)
    {
    case RES_CANNOT_WRITE:
    case RES_CANNOT_READ:
    case RES_RECIEVED_GARBAGE:
        return  false;   //reading failed

    case RES_INVALID_DATA:
        patchData(vals);    //if we got an invalid value, we swap it with the last valid value
        readRes=true;
    
        break;
    case RES_SUCCESS:
        updateLegals(vals);

        readRes=true;   //this reading is usable
        break;
    default:
        break;

    }

#else
    buff[0]='\n';
    cout << "Input volts: " ;
   for(int i=0;i<NUM_VALS;i++)
  {
       res->at(i)=0.1;       //artificial testing volt value
        cout << res->at(i) << " ";
  }
    cout << endl;
   //testing
#endif

    //bias values read from arduino
    for(int i=0, readingNum=0;i<vals.size() && readingNum<NUM_VALS;i++,++readingNum){

        if(vals[i]!=INVALID_DATA)
        {
            
           res.at(readingNum)=vals[i]-bias(readingNum,vals[i]);  //biasing to zero
        }
        //if an arduino value is invalid, the corresponding value in res would be left as invalid

    }//for stuff in buff

    ///--------------------PROXIMITY CALCULATIONS
    for(int i=0;i<NUM_VALS;i++){//biasing to calibration distance
        res.at(i)= res.at(i)*((divider[i])/MAX_PROX); //calibartion curve maximum is accounted
   //cout<<res.at(i)<<" "<<endl; 
	 }


    for(int i=0;i<NUM_VALS;i++){//last 6 values are proximity

        //checks if value is not stationary
        if(!isStationaryProx(res.at(i),i)){      //if values changed
            //if not stationary, convert volts in proximity values
            res.at(i)=convertProx(res.at(i));
        }else{
            //otherwise return invalid value
            res.at(i)=NO_SIGNAL; //This is when there is no signal detected
        }

    }

    //checks for NaN
    for(int i=0;i<NUM_VALS;i++)
    {
        if(res[i]!=res[i])  //if this happens, IEEE says we have a nan
        {
            res[i]=m_lastLegals[i];
        }
    }

    ///-----------------------------------------------

    return readRes;
}

//returns true if data is stationary, input is the value and the sensor id
//WARNING as it is now, only the first value in a set of 3 is checked for stationarity
bool Tactile::isStationaryProx(const double val,const int idx){

    if(val==INVALID_DATA)   //assume invalid data as stationary
    {
        return false;
    }

    history_prox.at(idx)->push(val);
    history_val_prox.at(idx)+=val;
    if(history_prox.at(idx)->size()<NUM_HISTORY_VALS){ //if we have not enough values yet
        return false;
    }
    m_hasHistoryProx=true;

    history_val_prox.at(idx)-=history_prox.at(idx)->front(); //subtract oldest value
    history_prox.at(idx)->pop();
    return ( (history_val_prox.at(idx)/NUM_HISTORY_VALS) < STATIONARY_PROXIMITY_THREASHOLD);

}

//calculates mean of first 10 values and return the bias value
double Tactile::bias(const int idx,const double val)
{

    if(mean[idx].size()==0){
        mean[idx].push_back(val);
        return 1.0;
    }
    //else calculate mean of past values
    if(mean[idx].size()<NUM_INTIALISATION_VALS){
        mean[idx].push_back(val);
    }

    if(mean[idx].size()==NUM_INTIALISATION_VALS)
    {
        m_isBiased=true;

        //calculate biases
        double accumulator=0.0;
        for(int i=0;i<mean[idx].size();++i){
            accumulator+=mean[idx].at(i);
        }
        m_biases[idx]=(accumulator/mean[idx].size());
    }


    return m_biases[idx];
}

bool Tactile::isSensorInit() const
{

    return ( m_isBiased && m_hasHistoryProx);
}



void Tactile::patchData(std::vector<double>& data)
{
    for(int i=0;i<data.size() && i<NUM_VALS;++i)
    {
        if(data[i]==INVALID_DATA)
        {
            data[i]=m_lastLegals[i];
           
        }
    }

}

void Tactile::updateLegals(std::vector<double>& data)
{
   

    for(int i=0;i<data.size();++i)
    {
        m_lastLegals.at(i)=data[i];

    }

}



//convert volts into distance
double Tactile::convertProx(const double num){

    if(num==INVALID_DATA) {   //if value not valid (shouldn't happen anymore)
        return INVALID_DATA;
    }

    double val=num;   //subtract 1 volt

    return (-((A_PROX*pow(val,B_PROX))+C_PROX));


}





