/*
 * Proximity sensor
 */

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle  nh;

std_msgs::Int16MultiArray output;
ros::Publisher value_publisher("/proximityShadow/raw", &output);

int16_t output_array[9];

void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(value_publisher);
  output.data = output_array;
  output.data_length = 5;
}

void loop()
{
  // 3 values for each sensor
  for(byte i=0;i<5;i++){ //reads from pin 0 to pin 9
        output_array[i] = analogRead(i);
  }

  value_publisher.publish( &output);
  nh.spinOnce(); 
}
