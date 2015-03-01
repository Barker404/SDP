#include <SerialCommand.h>
#include <SDPArduino.h>
#include <Wire.h>
#include <SerialCommand.h>
#include <SoftwareSerial.h>   // We need this even if we're not using a SoftwareSerial object due to the way the Arduino IDE compiles


#define LED_PIN 13  // Arduino LED on board
#define RADIO_PIN 8
#define LEFT_WHEEL_MOTOR 3
#define RIGHT_WHEEL_MOTOR 5
#define KICKER_MOTOR 4
#define CATCHER_MOTOR 2

SerialCommand SCmd;   // The demo SerialCommand object

void setup() {

  SDPsetup();
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
  SCmd.addCommand("RUN_ENG", RUN_ENGINE);
  SCmd.addDefaultHandler(unrecognized);
}

void loop() {
  SCmd.readSerial();
}


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
        motorBackward(5, -leftSpeed);
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
        motorBackward(3, -rightSpeed);
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
      
      motorForward(CATCHER_MOTOR, 100);    //Start opening the catcher first so that the catcher legs don't foul the ball
      delay(100); //allows the catcher to move out the way so that the kciker can swing out
      motorBackward(KICKER_MOTOR, power); //when kicking, motor is moving backwards
      delay(200);
      motorStop(CATCHER_MOTOR);
      delay(100);
      motorStop(KICKER_MOTOR);
      delay(150);
      motorForward(KICKER_MOTOR, power); //return the kicker to resting position ready for the ball to be caught again
      delay(230);
      motorStop(KICKER_MOTOR);

    }
  }
}


void pick_up() {
  motorBackward(CATCHER_MOTOR, 60);
  delay(200);
  motorStop(CATCHER_MOTOR);
}


void drop() {
  motorForward(CATCHER_MOTOR,100);
  delay(300);
  motorStop(CATCHER_MOTOR);
}


void unrecognized() {
  
  Serial.println("What?"); 
}


/*-----------------------LED functions for testing-------------------*/
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

