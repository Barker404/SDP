#include <SerialCommand.h>
#include <SoftwareSerial.h>
#include <SDPArduino.h>


/* These macros will have to be updated with the correct motor numbers. */
#define  LEFT_MOTOR   1
#define  RIGHT_MOTOR  2
#define  KICK_MOTOR   3

SerialCommand SCmd;

void setup()
{
	pinMode(8, OUTPUT);				// Set the radio pin to output.
	digitalWrite(8, HIGH);			// Turn on the radio.
	Serial.begin(115200);			// Start the serial port at 115200 baud (correct for XinoRF and RFu, if using XRF + Arduino you might need 9600).
	Serial.print("STARTED");		// Send a message across the radio.
	
	SCmd.addCommand("FORWARDS_50", forwards50);		// Set arduino to run forwards50() on receiving the command "FORWARDS 50".
	SCmd.addCommand("FORWARDS_10", forwards10);		// Set arduino to run forwards10() on receiving the command "FORWARDS 10".
	SCmd.addCommand("BACKWARDS_20", backwards20);	// Set arduino to run backwards20() on reveiving the command "BACKWARDS 20".
	SCmd.addCommand("KICK", kick);					// Set arduino to run kick() on receiving the command "KICK".
	
	SCmd.addDefaultHandler(unrecognized);			// Set arduino to run unrecognised() on receiving an unrecognised command.
}
void loop()
{
	SCmd.readSerial();
}

void forwards50()
{
	Serial.println("Moving forwards 50cm.");
	
	motorForward(LEFT_MOTOR, 100);
	motorForward(RIGHT_MOTOR, 100);
	
	delay(5000);
	
	motorStop(LEFT_MOTOR);
	motorStop(RIGHT_MOTOR);
}

void forwards10()
{
	Serial.println("Moving forwards 10cm.");
	
	motorForward(LEFT_MOTOR, 100);
	motorForward(RIGHT_MOTOR, 100);
	
	delay(1000);
	
	motorStop(LEFT_MOTOR);
	motorStop(RIGHT_MOTOR);
}

void backwards20()
{
	Serial.println("Moving backwards 20cm.");
	
	motorBackward(LEFT_MOTOR, 100);
	motorBackward(RIGHT_MOTOR, 100);
	
	delay(2000);
	
	motorStop(LEFT_MOTOR);
	motorStop(RIGHT_MOTOR);
}

void kick()
{
	Serial.println("Kicking.");
	
	motorForward(KICK_MOTOR, 100);
	
	delay(500);
	
	motorBackward(KICK_MOTOR, 100);
	
	delay(500);
}

void unrecognized()
{
	Serial.println("Command unrecognised."); 
}
