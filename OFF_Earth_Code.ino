/* 
  Description:
  This program was written based off the system analysis written by the
  two other members of my team.
  The program is a fully automated system with a complete set of coded
  instructions for the drill.
  This version however will have some lines of code that are specific for
  debugging which may or may not be found in the final piece of code.
  Version 1.3 is a code for which the defined values may be wrong as we
  have not done testing and do not have the exact values for which the
  vehicle can run at, the code is purely theoretical at the moment.
  
  Newest version of this code is always stored on Github in my account NoOne2246
  under Off-Earth-Project as the file Mining.ino
  https://github.com/NoOne2246/Off-Earth-Project/blob/master/Mining.ino
  
  Author:
  Christopher Chun-Hung Ho
  z5019205
  
  History:
  
  v2.2  29/10/14 Added new methods of going from the ball as well and changed pins.
  v2.1  27/10/14 Changed method of moving the drill to actuator.
  v2.0  25/10/14 Major Revision of code
  v1.8  25/10/14 Changed movement to the ball
  v1.7	25/10/14 Added feedback messaging
  v1.6  23/10/14 Removed Locate function, added error correction to detect function
                 also added the new rotation system
  v1.5	13/9/14  Changed the movement to another function of its own
                 fixed up Ping control with the ultrasound library
  v1.4  10/9/14  added manual control to the code.
  v1.3  5/9/14   initial implementation
		 changed ClampControl to boolean
  v1.2  3/9/14   changed the format of the layout to Arduino style
  v1.1  28/8/14	 outline the main parts
  v1.0  21/8/14  Write out the basics of the code
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
//sensor value
#define LIGHT 80                   //value of ball in the sensor

//timing
#define ROTATION 100               // milli seconds to turn one degree
#define PAUSE 50                   //pause for motor to make sure it stops movement
#define TURNSPEED 22               //turn speed of servo

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
const int dForPin = 7;             //turn drill forward
const int dBakPin = 8;             //turn drill reverse

//clamp pin
const int rota1 = 5;               //rotate ball clockwise
const int rota2 = 6;               //rotate ball counter clockwise

//Motor Pins
const int motPin1 = 4;             //Left motor pin
const int motPin2 = 3;             //Right Motor Pin


//on/off overide
const int Override = 0;            //Shutdown interrupt

//Trigger Pins
const int trigOn = 2;              //on signal pin
const int trigFor = 9;             //on signal pin
const int trigBak = 10;            //on signal pin
const int trigClos = 11;           //on signal pin

//light
const int Ball = A0;               //ball light switch


//-----------------------------------------------------------------------------------------------------
//Declaring
//-----------------------------------------------------------------------------------------------------

//Functions                   Pretty much self explanatory, their use is written in the Functions Section
void drillCtrl();
void turnOff();
void forward();
void moveCtrl(int dir);
void rotate(int angle);
void manualCtrl(int locat);
void printPorts();


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
  
  Serial.println("Initialisation Complete");
  Serial.println("Hit button or any key to begin");
  
  //Wait for start signal
  while(digitalRead(trigOn)==0||!Serial.available()){                         //wait until it receives start signal
  }
  Serial.read();
  
  attachInterrupt(Override, turnOff, CHANGE);        //switch off vehicle through interrupt
  
  
  forward();					     //move to the ball
  
  rotate(90);
}

//loop function
void loop (){                                      //loop has been simplified as it may change to manual control

  drillCtrl();                                     //Run the ball drilling function.
  
  rotate(3);                                       //run rotate ball function
}



//-----------------------------------------------------------------------------------------------------
// subroutines
//-----------------------------------------------------------------------------------------------------


//move forward
void forward(){
  moveCtrl(FORWARD);               	     //move forward
  
  while(trigClos==LOW && !Serial.available() && analogRead(Ball) > LIGHT){                      //wait until vehicle hits ball
    if(Serial.available()){
      manualCtrl(1);
    }
  }
  Serial.read();
  
  moveCtrl(STOP);                            //Stop vehicle
}

//rotate ball
void rotate(int angle){
  int time;				   //create time variable
  
  Serial.print("Angle to rotate:");
  Serial.println(angle);
  
  time = angle*ROTATION;                   //calculate the time to rotate for in ms
  Serial.print("Time to rotate:");
  Serial.println(time);
  
  Serial.println("Rotating ball");
  
  digitalWrite(rota1, HIGH);		   //rotate the ball
    
  delay(time);				   //delay for calculated time
  
  digitalWrite(rota1, LOW);                 //stop rotating ball
  
  Serial.println("Stop rotating ball");
  
  if(Serial.available()){
    manualCtrl(2);
  } 
}

//movement
void moveCtrl(int dir){
  switch(dir){
    case FORWARD:
	
      digitalWrite(motPin1, HIGH);              //Both motors on
      digitalWrite(motPin2, HIGH);
      Serial.println("Moving Forward");
      break;
  		
    case LEFT:
  		
      digitalWrite(motPin1, LOW);               //Right motor on
      digitalWrite(motPin2, HIGH);
      Serial.println("Turning Left");
      break;
  		
    case RIGHT:
    
      digitalWrite(motPin1, HIGH);              //left motor on
      digitalWrite(motPin2, LOW);
      Serial.println("Turning Right");
      break;
  		
    case STOP:
    
      digitalWrite(motPin1, LOW);               //stop both motors
      digitalWrite(motPin2, LOW);
      Serial.println("Stopping");
      break;
	  
    default:
      break;
  }
}

//drill movement
void drillCtrl(){
  
  digitalWrite(dForPin, HIGH);              //drill turn forward
  
  Serial.println("Drill Moving Forward");
  
  while (digitalRead(trigFor)==LOW){        //check if stop switch has been hit
    
    if(Serial.available()){
      manualCtrl(3);
    }
  }
  
  Serial.println("Drill stop Forward");
  digitalWrite (dForPin, LOW);              //stop forward turning of the drill
  
  
  delay(PAUSE);                             //pause for moment
  
  digitalWrite (dBakPin, HIGH);             //reverse drill
  delay(PAUSE);
  Serial.println("Drill Moving Backward");
  
  while (digitalRead(trigBak)==LOW){        //check if stop switch has been hit
    if(Serial.available()){
      manualCtrl(4);
    }
  }
  digitalWrite (dBakPin, LOW);              //stop drill
  Serial.println("Drill Stop Backward");
}


void manualCtrl(int locat){
  Serial.println("Manual Control Mode");
  char input = Serial.read();
  
  for (int pin = 3; pin < 10; pin++){
    digitalWrite(pin, LOW);
  }
  
  while(input!='r'){                    //r is return/resume key
    switch (input) {
      case 'W': case 'w':  // Car Forward
         Serial.print("---Car moving Forward---\n");
         
         if (input == lastCommand) {
           moveCtrl(STOP);
         } else {
           moveCtrl(FORWARD);
         }
         printPorts();
         break;
      case 'A': case 'a':  // Car Left
         Serial.println("---Car moving left---");
         
         if (input == lastCommand) {  
           moveCtrl(STOP);  
         } else {
           moveCtrl(LEFT);
         }
         
         printPorts();
         break;
      case 'D': case 'd':  // Car Right
         Serial.println("---Car moving right---");
         
         if (input == lastCommand) {
           moveCtrl(STOP);
         } else {
           moveCtrl(RIGHT);
         }
         
         printPorts();
         break;
      case 'S': case 's':  // Car stop
        
         moveCtrl(STOP);
         
         printPorts();
         break;
      case 'Z': case 'z':  // Clamp rotateLeft
         Serial.println("---Clamp rotateLeft---");
         if (digitalRead(rota1) == LOW && 
             digitalRead(rota2) == LOW) {
           digitalWrite(rota1, HIGH);
         } else {     
           digitalWrite(rota1, LOW);
           digitalWrite(rota2, LOW);
         }
         printPorts();
         break;
      case 'X':
      case 'x':  // Clamp rotateRight
         Serial.println("---Clamp rotateRight---");
         if (digitalRead(rota1) == LOW && 
             digitalRead(rota2) == LOW) {
           digitalWrite(rota2, HIGH);
         } else {     
           digitalWrite(rota1, LOW);
           digitalWrite(rota2, LOW);
         }
         printPorts();
         break;
      case 'I': case 'i':  // Drill Forward
         Serial.println("---Drill going forward---");
         if (digitalRead(dForPin) == LOW && 
             digitalRead(dBakPin) == LOW) {
           digitalWrite(dForPin, HIGH);
         } else {     
           digitalWrite(dForPin, LOW);
           digitalWrite(dBakPin, LOW);
         }
         printPorts();
         break;
         
      case 'K':
      case 'k':  // Drill Backward
         Serial.println("---Drill going backward---");
         if (digitalRead(dForPin) == LOW && 
             digitalRead(dBakPin) == LOW) {
           digitalWrite(dBakPin, HIGH);
         } else {     
           digitalWrite(dForPin, LOW);
           digitalWrite(dBakPin, LOW);
         }
         
         printPorts();
         break;
         
      case 'P':
      case 'p': // Print Ports
         Serial.println("---Printing all ports---");
         printPorts();
         break;
      case 'M':
      case 'm':
         turnOff();
         printPorts();
         break;
      case 'E':
      case 'e':
         turnOff();
         break;
      default:
         break;
    }
    
    lastCommand = input;
    
    while(!Serial.available()){
    }
    input = Serial.read();
  }
  
  for (int pin = 3; pin < 10; pin++){
    digitalWrite(pin, LOW);
  }
  
  switch(locat){
    case 1:
      moveCtrl(FORWARD);
      break;
    case 2:
      break;
    case 3:
      if(digitalRead(trigFor)==LOW){
        digitalWrite(dBakPin, LOW);
        digitalWrite(dForPin, HIGH);
      }
      digitalWrite(Drill, HIGH);
      break;
    case 4:
      if(digitalRead(trigBak)==LOW){
        digitalWrite(dForPin, LOW);
        digitalWrite(dBakPin, HIGH);
      }
      digitalWrite(Drill, HIGH);
      break;
    default:
      break;
  }
  Serial.println("Resuming");
  return;
}

void printPorts() {
  Serial.print("Left Wheel: ");
  Serial.println(digitalRead(motPin1));
  
  Serial.print("Right Wheel: ");
  Serial.println(digitalRead(motPin2));
  
  Serial.print("Clamp 1: ");
  Serial.println(digitalRead(rota1));
  
  Serial.print("Clamp 2: ");
  Serial.println(digitalRead(rota2));
    
  Serial.print("Drill Forward: ");
  Serial.println(digitalRead(dForPin));
  
  Serial.print("Drill Backward: ");
  Serial.println(digitalRead(dBakPin));
  Serial.println(" ");
  Serial.println(" ");
  
}

//reset the machine and stop movement
void turnOff(){
  if(trigBak==LOW){
    digitalWrite (dBakPin, HIGH);             //move drill backwards
    delay(PAUSE);                             //give drill time to pick up speed 
    digitalWrite (dDowPin, HIGH);             //drill stop back   
    
    while (digitalRead(trigBak)==LOW){        //check if stop switch has been hit
      /*
      serVal--;                               //increase servo angle
      drillMove.write(serVal);                //write new angle
      delay(10);                              //delay 10ms
      */
    }
    digitalWrite (dDowPin, LOW);              //drill stop back
    digitalWrite (dBakPin, LOW);              //stop backward movement
  }
  while(1!=0){
    if(Serial.available()){
      manualCtrl(2);
    }
  }
}
