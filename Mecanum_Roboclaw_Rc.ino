#include <SoftwareSerial.h>
#include "RoboClaw.h"
 
RoboClaw roboclaw(&Serial,10000);

#define addressF 0x80
#define addressR 0x81
#define qpps 2750

// *********************
// Define hardware pins
// *********************
// RC mappings -- strafe: aileron, drive: elevation, turn: rudder
int strafePinRC = 12, drivePinRC = 11, turnPinRC = 13;

// *********************
// RC Vars
// *********************
unsigned long DRIVE_PULSE_WIDTH;
unsigned long TURN_PULSE_WIDTH;
unsigned long STRAFE_PULSE_WIDTH;
float pulseLow = 1150, pulseHigh = 1850, deadbandLow = 1450, deadbandHigh = 1550;
int PULSEIN_TIMEOUT = 2000000;

void setup() {
  //Serial.begin(9600);
  roboclaw.begin(9600);

}

void loop() {

  // Read in the RC pulses
  DRIVE_PULSE_WIDTH = pulseIn(drivePinRC, HIGH, PULSEIN_TIMEOUT);
  TURN_PULSE_WIDTH  = pulseIn(turnPinRC, HIGH, PULSEIN_TIMEOUT);
  STRAFE_PULSE_WIDTH  = pulseIn(strafePinRC, HIGH, PULSEIN_TIMEOUT);

  // If pulses too short, Stop Motors
  if(DRIVE_PULSE_WIDTH < 500 || TURN_PULSE_WIDTH < 500 || STRAFE_PULSE_WIDTH < 500 ) {
    roboclaw.SpeedM1M2(addressF,0,0);
    roboclaw.SpeedM1M2(addressR,0, 0);
    return;
  }
  else if(DRIVE_PULSE_WIDTH > 2500 || TURN_PULSE_WIDTH > 2500 || STRAFE_PULSE_WIDTH > 2500 ) {
    roboclaw.SpeedM1M2(addressF,0,0);
    roboclaw.SpeedM1M2(addressR,0, 0);
    return;
  }

 // convert RC signals to continuous values from [-qpps,qpps]
  float driveVal = convertRCtoInt(DRIVE_PULSE_WIDTH);
  float turnVal  = -1*convertRCtoInt(TURN_PULSE_WIDTH);
  float strafeVal = convertRCtoInt(STRAFE_PULSE_WIDTH);
  
  // Calculate each motor power based on drive turn and strafe values.
  int motorFR = constrain(-1*driveVal - turnVal + strafeVal, -qpps, qpps);
  int motorRR = constrain(driveVal + turnVal + strafeVal, -qpps, qpps);
  int motorFL = constrain(-1*driveVal + turnVal - strafeVal, -qpps, qpps);
  int motorRL = constrain(driveVal - turnVal - strafeVal, -qpps, qpps);

  roboclaw.SpeedM1M2(addressF, motorFL, motorFR);
  roboclaw.SpeedM1M2(addressR, motorRR, motorRL);

  // debug print
 /* Serial.print(DRIVE_PULSE_WIDTH); Serial.print(","); Serial.print(TURN_PULSE_WIDTH);
  Serial.print(","); Serial.print(STRAFE_PULSE_WIDTH);
  Serial.print("\t");
  Serial.print(driveVal); Serial.print(","); Serial.print(turnVal);
  Serial.print(","); Serial.print(strafeVal);
  Serial.print("\t");
  Serial.print(motorFL,DEC);  Serial.print(","); Serial.print(motorFR,DEC);  Serial.print(",");
  Serial.print(motorRL,DEC);  Serial.print(","); Serial.print(motorRR,DEC);
  Serial.print("\n");
*/
}

int convertRCtoInt(unsigned long pulseWidth)
{
  int checkVal;
  // deadband
  if(pulseWidth > deadbandLow && pulseWidth < deadbandHigh) { pulseWidth = 1500; checkVal = 0;}
  else if(pulseWidth <= deadbandLow) {checkVal = mapfloat(pulseWidth, deadbandLow, pulseLow, 0, -qpps); }
  else if(pulseWidth >= deadbandHigh) {checkVal = mapfloat(pulseWidth, deadbandHigh, pulseHigh, 0, qpps); }  
  
  checkVal = constrain(checkVal, -qpps, qpps);

  return checkVal;
}

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
 return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}
