/* 
  Description:
  this is bluetooth ctrl temporary version. this is used to ascertain the values
  used to control the motors allowing the vehicle to move forward straight
  
  Newest version of this code is always stored on Github in my account NoOne2246
  under Off-Earth-Project as the file Mining.ino
  https://github.com/NoOne2246/Off-Earth-Project/blob/master/Mining.ino
  
  Author:
  Christopher Chun-Hung Ho
  z5019205
  
  History:

  v1.0  28/10/14  Write out the basics of the code
*/

//-----------------------------------------------------------------------------------------------------
//Include
//-----------------------------------------------------------------------------------------------------
//#include <NewPing.h>
/*http://playground.arduino.cc/Code/NewPing
This is a ping library written by Tim Eckel to use the
HC-S04 ultrasound sensor. The library can return a median
of pings as well as time in microseconds*/

#include <Servo.h>
/*servo library that comes with arduino software*/
//-----------------------------------------------------------------------------------------------------
//Define
//-----------------------------------------------------------------------------------------------------

//timing
#define ROTATION 100               // milli seconds to turn one degree
#define PAUSE 50                   //pause for motor to make sure it stops movement
#define TURNSPEED 10               //turn speed of servo

//movement
#define FORWARD 1
#define LEFT 2
#define RIGHT 3
#define STOP 4

//-----------------------------------------------------------------------------------------------------
//Const pins
//-----------------------------------------------------------------------------------------------------

/*Interrupt is on pin 2, this pin is referred to as interupt 0 and will not be
  used within the control of the vehicle as it is an interrupt pin to shutdown
  the machine after it has been initiated.
*/
//drill pin
const int dForPin = 3;             //turn drill forward
const int dBakPin = 5;             //turn drill reverse

//clamp pin
const int rota1 = 7;               //rotate ball clockwise
const int rota2 = 8;               //rotate ball counter clockwise

//Motor Pins
const int motPin1 = 10;            //Left motor pin
const int motPin2 = 11;            //Right Motor Pin


//on/off overide
const int Override = 0;            //Shutdown interrupt

//Trigger Pins
const int trigOn = 2;              //on signal pin
const int trigFor = 4;             //on signal pin
const int trigBak = 6;             //on signal pin
const int trigClos = 12;           //on signal pin



//-----------------------------------------------------------------------------------------------------
//Declaring
//-----------------------------------------------------------------------------------------------------

//Variables
int leftSpd = 200;
int righSpd = 200;

//Functions                   Pretty much self explanatory, their use is written in the Functions Section
void drillCtrl();
void turnOff();
void forward();
void moveCtrl(int dir);
void rotate(int angle);
void manualCtrl(int locat);

//Servo
//Servo drillMove;

//-----------------------------------------------------------------------------------------------------
// main
//-----------------------------------------------------------------------------------------------------
//setup function
void setup(){

  Serial.begin(9600);                              //initialise serial communications
  
  delay(1000);
  Serial.println("AT+NAMEBlackhole");
  delay(1000);
  Serial.print("AT+PIN1123");
  delay(1000);
  
  /*
  //Set up output pins
  pinMode(driPin, OUTPUT);
  pinMode(dForPin, OUTPUT);
  pinMode(dBakPin, OUTPUT);
  pinMode(cloPin, OUTPUT);
  pinMode(clcPin, OUTPUT);
  pinMode(rota, OUTPUT);
  pinMode(motPin1, OUTPUT);
  pinMode(motPin2, OUTPUT);
  pinMode(motEnable, OUTPUT);
  pinMode(LTrig, OUTPUT);
  pinMode(CTrig, OUTPUT);
  pinMode(RTrig, OUTPUT);
  pinMode(closTrig, OUTPUT);
  
  //set up input pins
  pinMode(LRec, INPUT);
  pinMode(CRec, INPUT);
  pinMode(RRec, INPUT);
  pinMode(On, INPUT);
  */
  
  //Set up output pins
  pinMode(dForPin, OUTPUT);
  pinMode(dBakPin, OUTPUT);
  pinMode(rota1, OUTPUT);
  pinMode(rota2, OUTPUT);
  pinMode(motPin1, OUTPUT);
  pinMode(motPin2, OUTPUT);
  
  //set up input pins
  pinMode(trigOn, INPUT);
  pinMode(trigFor, INPUT);
  pinMode(trigBak, INPUT);
  pinMode(trigClos, INPUT);
  
  /*
  //set up servo
  drillMove.attach(serPin);
  drillMove.write(serVal);
  */
  /*
  attachInterrupt(Override, manualCtrl, CHANGE);   //switch to manual control the vehicle
  
  noInterrupts();                                  //disable interrupts as we wait for start signal
  */
  
  Serial.println("Initialisation Complete");
  Serial.println("use keys to vary speed");
  analogWrite(motPin1, leftSpd);
  analogWrite(motPin1, righSpd);

}

//loop function
void loop (){                                      //loop has been simplified as it may change to manual control
  if(Serial.available()){
    switch(Serial.read()){
      case '1':
        leftSpd = constrain(leftSpd++, 0, 255);
        analogWrite(motPin1, leftSpd);
        break;
      case '2':
        leftSpd = constrain(leftSpd--, 0, 255);
        analogWrite(motPin1, leftSpd);
        break;
      case '3':
        righSpd = constrain(righSpd++, 0, 255);
        analogWrite(motPin1, righSpd);
        break;
      case '4':
        righSpd = constrain(righSpd--, 0, 255);
        analogWrite(motPin1, righSpd);
        break;
      case 'r':
        Serial.print("Left:");
        Serial.print(leftSpd);
        Serial.print(" Right:");
        Serial.println(righSpd);
        break;
      default:
        analogWrite(motPin1, 0);
        analogWrite(motPin1, 0);
        break;
    }
  }
  
}
