#include <SDPArduino.h>


#include <SDPArduino.h>
#include <Wire.h>
#include <SerialCommand.h>
#include <SoftwareSerial.h>   // We need this even if we're not using a SoftwareSerial object
                              // Due to the way the Arduino IDE compiles

#define LED_PIN 13   // Arduino LED on board
#define RADIO_PIN 8
#define LEFT_WHEEL_MOTOR 3
#define RIGHT_WHEEL_MOTOR 5
#define KICKER_MOTOR 4

// int left_speed;
// int right_speed;
// int left_dir;
// int right_dir;

SerialCommand SCmd;   // The demo SerialCommand object

void setup() {

  SDPsetup();
  //helloWorld();
  pinMode(LED_PIN, OUTPUT);   // initialize pin 13 as digital output (LED)
  pinMode(8, OUTPUT);    // initialize pin 8 to control the radio
  digitalWrite(8, HIGH); // select the radio
  Serial.begin(115200);    // start the serial port at 115200 baud (correct for XinoRF and RFu, if using XRF + Arduino you might need 9600)
  
  Serial.print("STARTED");
  // !!!
  // Make sure there's no more than ten of these!
  // Any more and the bottom ones don't work
  // !!!

  SCmd.addCommand("RUN_KICK",kick);            
  SCmd.addCommand("RUN_CATCH", pick_up);         
  SCmd.addCommand("DROP", drop);
  /*SCmd.addCommand("SET_ENG", SET_ENGINE);*/
  SCmd.addCommand("RUN_ENG", RUN_ENGINE);
  SCmd.addDefaultHandler(unrecognized);
}

void loop() {
  SCmd.readSerial();
}

/*void SET_ENGINE() {

  char *lftspd = SCmd.next();
  char *rgtspd = SCmd.next();
  
  
  if (lftspd != NULL && rgtspd != NULL) {
    left_speed = atoi(lftspd);
    right_speed = atoi(rgtspd);
    motorForward(5, left_speed);
    motorForward(3, right_speed);
    delay(1000);
    motorStop(3);
    motorStop(5);
  }
 
}*/

void RUN_ENGINE() {

  char *leftSpeedStr = SCmd.next();
  char *rightSpeedStr = SCmd.next();
  
  if (leftSpeedStr != NULL && rightSpeedStr != NULL) {
    int leftSpeed = atoi(leftSpeedStr);
    int rightSpeed = atoi(rightSpeedStr);
      
    if (leftSpeed > 0)
    {
        motorForward(5, leftSpeed);
    }
    else if (leftSpeed < 0)
    {
        motorBackward(5, leftSpeed);
    }
    else
    {
      motorStop(5);
    }

    if (rightSpeed > 0)
    {
        motorForward(3, rightSpeed);
    }
    else if (rightSpeed < 0)
    {
        motorBackward(3, rightSpeed);
    }
    else
    {
      motorStop(3);
    }
  }
}
        
    
void kick() { //motor 3 not catcher, needs changed
  char *powerStr = SCmd.next();
  if (powerStr != NULL) {
    
    int power = atoi(powerStr);
    if (power != NULL) {
      
      motorForward(3, 100);    //Start opening the catcher first so that the catcher legs don't foul the ball
      motorForward(4, power);
      delay(300);
      motorStop(3);
      motorStop(4);
      motorBackward(4, power); //change return power
      delay(300);
      motorStop(4);
      motorBackward(3, 100); //return the kicker to resting position before closing the catcher first, otherwise the catcher will stop the kicker return
      delay(300);
      motorStop(3);
    }
  }
}


void pick_up() {
  motorForward(3, 100);
  delay(400);
  motorStop(3);
}

void drop() {
  motorBackward(3,100);
  delay(300);
  motorStop(3);
}

void unrecognized() {
  
  Serial.println("What?"); 
}

// LED functions for testing
void LED_on() {
  Serial.println("LED on"); 
  digitalWrite(LED_PIN,HIGH);  
}

void LED_off() {
  
  Serial.println("LED off"); 
  digitalWrite(LED_PIN,LOW);
  
}

void flash(int interval) {
  
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(interval);              // wait for a second
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(interval);              // wait for a second
}

