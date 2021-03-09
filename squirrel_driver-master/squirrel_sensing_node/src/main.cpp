#include <string>

#include "../include/squirrel_sensing_node/node.h"

using namespace std;


int main(int argc,char** argv){

    if(argc<ParamNum){
        cout << "Error starting sensing node. Correct syntax:" << endl;
        cout << "rosrun squirrel_sensing_node sensing [arduino_port_name] [ft17_port_name]" << endl;	//<----Liza: check if the node name is correct
        return 1;
    }

    string name="sensing";  //can be read from argv

    ros::init(argc, argv, name);

    cout << "Creating " << name << " node " << endl;

	vector<string> pars;	//better to use a vector than an array of strings
    
    cout << "Arguments: " << endl;
    for(unsigned int i=0; i<argc-1; ++i)	//we skip the executable name
	{
        if(i<ParamNum)
		{
            pars.push_back(argv[i+1]);
		}
        else if(i==ParamNum)	//+1 because the char* paramters are 1-indexed
		{
			cout << "----------" << endl; //anything under this line will be ignored
        }
        cout << argv[i+1] << endl;
		
	}
    cout << "#######" << endl;   //for beauty

    if(pars.size()!=ParamNum)
    {
        cout << "ERROR: Wrong number of parameter received! Expected: " << ParamNum << " Received: " << pars.size() << endl;
        cout << "Is time to fail gently" << endl;
        return 2;
    }

    SensingNode sensing(name,pars); 

    cout << "Executing " << name << " node " << endl;

    sensing.run();

    cout << name << " node has terminated his execution" << endl;

    return 0;
}
