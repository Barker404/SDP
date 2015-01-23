#include <SerialCommand.h>

//
// Turn on/off the LED on pin 13 by command received from the radio
//  0 turns the LED off
//  1 turns the LED on
//  any other character sent has no effect
//

#include <SoftwareSerial.h>   // We need this even if we're not using a SoftwareSerial object
                              // Due to the way the Arduino IDE compiles

#define arduinoLED 13   // Arduino LED on board

SerialCommand SCmd;   // The demo SerialCommand object

byte msg;  // the command buffer
void setup()
{
  pinMode(arduinoLED, OUTPUT);   // initialize pin 13 as digital output (LED)
  pinMode(8, OUTPUT);    // initialize pin 8 to control the radio
  digitalWrite(8, HIGH); // select the radio
  Serial.begin(115200);    // start the serial port at 115200 baud (correct for XinoRF and RFu, if using XRF + Arduino you might need 9600)
  
  Serial.print("STARTED");
  SCmd.addCommand("ON",LED_on);       // Turns LED on
  SCmd.addCommand("OFF",LED_off);        // Turns LED off
  SCmd.addDefaultHandler(unrecognized);
}
void loop()
{
  SCmd.readSerial();
}


void LED_on()
{
  Serial.println("LED on"); 
  digitalWrite(arduinoLED,HIGH);  
}

void LED_off()
{
  Serial.println("LED off"); 
  digitalWrite(arduinoLED,LOW);
}
void flash(int interval) {
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(interval);              // wait for a second
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(interval);              // wait for a second
}
void unrecognized()
{
  Serial.println("What?"); 
}