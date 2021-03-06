/* 
  Description:
  this is a code written to test the distance moved for the vehicle in 10000ms.
  
  Author:
  Christopher Chun-Hung Ho
  z5019205
  
  History:
  
  v1.0  13/9/14 file created to test.
*/

const int motPin1 = 11;        //motor pin 1 for logic gate
const int motPin2 = 10;        //motor pin 2 for logic gate

void setup(){

  pinMode(motPin1, OUTPUT);
  pinMode(motPin2, OUTPUT);

  delay(1000);
  
  digitalWrite(motPin1, HIGH);
  digitalWrite(motPin2, HIGH);
  delay(10000);
  digitalWrite(motPin1, LOW);
  digitalWrite(motPin2, LOW);
}

void loop(){
}
