const int g=103;  //ascii code for 'g', ros sends this automatically
const int c=99;   //ascii code for 's'
const int C=67;   //ascii code for 'S'
const int r=114;  //ascii code for 'r'
const int R=82;   //ascii code for 'R'
int bCalibMode;  //boolean, if 1 arduino is place in calibration mode
                //when put in calibration mode it cannot be used with ros
                //to exit calibration mode you must restart
void setup(){
  
  pinMode(LED_BUILTIN, OUTPUT);
  
  //led on: not connected
  digitalWrite(LED_BUILTIN, HIGH);

  bCalibMode=0;  //ready for ros

  //NOTE: the fastest we can go in this system is 38400 bauds, this value
  //can be given to the function below (and matched in ros) but to calibrate it
  //you would need to use something like telnet, teraterm or hyperterminal as the
  //arduino serial monitor struggles to cope with that speed
  Serial.begin(38400); //initializes the serial port with the pc
  
  while(!Serial);  //get stuck until you are connected to PC
  
  //led off: connected
  digitalWrite(LED_BUILTIN, LOW);
}


void loop(){
  
//as arduino sdk stripped out proper interrupts, here we don't do anything
  if(bCalibMode)  //stream voltages forever
  {
    for(byte i=0;i<15;i++){ //reads from pin 0 to pin 14
        double volt=analogRead(i)*(5.0/1023.0);
        Serial.print(volt);
        Serial.print(" ");
    }
    Serial.println();
  }

}

//surrogate of interrupt
void serialEvent()
{
  
  //only 1 char is allowed
  int cmd=Serial.read();  //you have 1 sec to do something
  
  switch(cmd)
  {
  
    case g:
    {
      if(!bCalibMode)
      {
        digitalWrite(LED_BUILTIN, HIGH);
        //we write as quickly as we can, not to upset the PC, since every call is slow
        for(byte i=0;i<15;i++){ //reads from pin 0 to pin 14
            Serial.print(analogRead(i));
            Serial.print(" ");
        }
        Serial.println();
        digitalWrite(LED_BUILTIN, LOW);  //led blink: command received and executed
      }
      break;
    }
    case 'c':
    case 'C':
    {
      bCalibMode=1;
      Serial.println("Switching to calibration mode, enter 'R' to revert to ROS mode, I can't be plugged into ROS");
      delay(1500);
      break;
    }
    case 'r':
    case 'R':
    {
      bCalibMode=0;
      Serial.println("Switching to ROS mode, enter 'C' to revert to calibration mode, I can work with ROS now");
      delay(10);
      break;
    }
  }
    
 


}

