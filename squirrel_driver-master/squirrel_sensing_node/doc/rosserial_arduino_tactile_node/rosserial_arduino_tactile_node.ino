/*
 * Tactile sensors
 */

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle  nh;

std_msgs::Int16MultiArray output;
ros::Publisher value_publisher("/tactileShadow/raw", &output);

int16_t output_array[3];

void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(value_publisher);
  output.data = output_array;
  output.data_length = 3;
}

void loop()
{
  // a pin for each 1-dof force
  for(byte i=0;i<3;i++){ //reads from pin 0 to pin 3
        output_array[i] = analogRead(i);
  }

  value_publisher.publish( &output);
  nh.spinOnce(); 
}
