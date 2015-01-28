
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

SerialCommand SCmd;   // The demo SerialCommand object

void setup()
{
  SDPsetup();
  helloWorld();
  pinMode(LED_PIN, OUTPUT);   // initialize pin 13 as digital output (LED)
  pinMode(8, OUTPUT);    // initialize pin 8 to control the radio
  digitalWrite(8, HIGH); // select the radio
  Serial.begin(115200);    // start the serial port at 115200 baud (correct for XinoRF and RFu, if using XRF + Arduino you might need 9600)
  
  Serial.print("STARTED");
  SCmd.addCommand("ON",LED_on);          // Turns LED on
  SCmd.addCommand("OFF",LED_off);        // Turns LED off
  SCmd.addCommand("FWD",wheelsForward);
  SCmd.addCommand("BWD",wheelsBackward);
  SCmd.addCommand("TLEFT",wheelsLeft);
  SCmd.addCommand("TRIGHT",wheelsRight);
  SCmd.addCommand("STOP",wheelsStop);
  SCmd.addCommand("KICK",kick);
  SCmd.addCommand("GOAL",goalKick);
  SCmd.addCommand("TESTF",forwardTest);
  SCmd.addCommand("TESTB",backwardTest);
    
  SCmd.addDefaultHandler(unrecognized);
}
void loop()
{
  SCmd.readSerial();
}

void f50cm()
{
    wheelsForward();
    delay(2000);
    wheelsStop();
}
void f10cm()
{
    wheelsForward();
    delay(500);
    wheelsStop();
}
void b20cm()
{
    wheelsBackward();
    delay(1000);
    wheelsStop();
}


void LED_on()
{
  Serial.println("LED on"); 
  digitalWrite(LED_PIN,HIGH);  
}
void LED_off()
{
  Serial.println("LED off"); 
  digitalWrite(LED_PIN,LOW);
}
void flash(int interval)
{
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(interval);              // wait for a second
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(interval);              // wait for a second
}
   
void forwardTest()
{
  char *timeStr = SCmd.next();
  if (timeStr != NULL)
  {
    int time = atoi(timeStr);
    if (time != NULL)
    {
      wheelsForward();
      delay(time);
      wheelsStop();
    }
  }
}
    
void backwardTest()
{
  char *timeStr = SCmd.next();
  if (timeStr != NULL)
  {
    int time = atoi(timeStr);
    if (time != NULL)
    {
      wheelsBackward();
      delay(time);
      wheelsStop();
    }
  }
}

void wheelsForward()
{
  motorForward(3, 100);
  motorForward(5, 100);
}
void wheelsBackward()
{
  motorBackward(3, 100);
  motorBackward(5, 100);
}
void wheelsLeft()
{
  motorBackward(3, 100);
  motorForward(5, 100);
}
void wheelsRight()
{
  motorForward(3, 100);
  motorBackward(5, 100);
}
void wheelsStop()
{
  motorStop(3);
  motorStop(5);
}

void kick()
{
  motorForward(4, 40);
  delay(400);
  motorStop(4);
  delay(200);
  motorBackward(4, 100);
  delay(300);
  motorStop(4);
}

void goalKick()
{
  motorForward(4, 60);
  delay(400);
  motorStop(4);
  delay(200);
  motorBackward(4, 100);
  delay(300);
  motorStop(4);
}

void unrecognized()
{
  Serial.println("What?"); 
}
