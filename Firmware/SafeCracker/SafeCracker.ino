/*
  Opening a safe using a motor and a servo
  By: Nathan Seidle @ SparkFun Electronics
  Date: February 24th, 2017

  We use a motor with encoder feedback to try to glean the internal workings of the
  safe to reduce the overall solution domain. Then we begin running through
  solution domain, pulling on the handle and measuring it as we go.

  Motor current turning dial, speed = 255, ~350 to 560mA
  Motor current turning dial, speed = 50, = ~60 to 120mA

  Modified by Mike L for use with Arduino Due
*/

// #include "nvm.h" //EEPROM locations for settings
// #include <EEPROM.h> //For storing settings and things to NVM

#include <Servo.h>
Servo handleServo;

// // ============== Pin definitions for Arduino Uno ==============
// //outputs (all digital)
// const byte motorPWM = 6; //output: controls motor speed
// const byte motorReset = 8; //output: enable/disables motor
// const byte motorDIR = 10; //output: control motor direction
// const byte servo = 9; //output: PWM signal to control servo
// //inputs
// const byte photo = 5; //input: photo gate
// const byte servoPosition = A1; //analog input: servo position feedback
// const byte encoderA = 2;
// const byte encoderB = 3;
// // const byte servoPositionButton = A1;

// ============== Pin definitions for Arduino Due ==============
//outputs (all digital)
const int motorPWM = 7; //output: controls motor speed
const int motorReset = 6; //output: enable/disables motor
const int motorDIR = 5; //output: control motor direction
const int servo = 4; //output: PWM signal to control servo
const int levelConverterEnable = 8; //output: enables the 3.3/5V level converter
//inputs (all digital except servoPosition)
const int photo = 11; //input: photo gate
const int encoderA = 2; // TIOA0 for onboard decoder
const int encoderB = 13; // TIOB0 for onboard decoder
const unsigned int mask_encoder_A = digitalPinToBitMask(encoderA);
const unsigned int mask_encoder_B = digitalPinToBitMask(encoderB); 
const int servoPosition = A1; //analog input: servo position feedback
const int servoPositionButton = 40;


//Settings for my personal white (20"x17"x17") cubic ft. safe
//High torque (20kg/cm) servo. Decrease command position to pull handle down.
int servoRestingPosition = 100; //Position not pulling/testing on handle. min/max = 0/175
int servoTryPosition = 16; //Position when testing handle
int servoHighPressurePosition = 25; //Position when doing indent measuring
int handleOpenPosition = 110; // set with test data

const int timeServoApply = 350;  //ms for servo to apply pressure. 350 works
const int timeServoRelease = 250;  //Allow servo to release. 250 works
const int timeMotorStop = 1000; //ms for motor to stop spinning after stop command. 200 works

const int stepTolerance = 84; //tolerance for the commanded vs arrived at dial position on either side

int handlePosition; //Used to see how far handle moved when pulled on
volatile boolean buttonWasPushed = false;

const int takeABreakAttempts = 300; //Used to let the motor cool down after so many attempts
//These are here to measure motor position to ensure that it doesn't burn out in case the dial gets stuck somehow
long timeSinceLastMovement;

//Direction detection: If dial is spun without changing direction, encoderA and encoderB edge should fire alternatingly
volatile bool encoderAEdge = false;
//Track motor encoder:
#define A_RISING 0 //encoder A rising edge
#define B_RISING 1 //encoder B rising edge
#define A_FALLING 2 //encoder A falling edge
#define B_FALLING 3 //encoder B falling edge
volatile byte lastEncoderEdge = A_RISING;
volatile byte numErrors = 0;
volatile byte numErrorsAR = 0;
volatile byte numErrorsAF = 0;
volatile byte numErrorsBR = 0;
volatile byte numErrorsBF = 0;

#define CCW 0
#define CW 1
//encoder register for Arduino Due
// #define steps encoderSteps

volatile int encoderSteps = 0; //Keeps track of encoder counts. 8400 per revolution so this can get big.
int lastStep = encoderSteps + 50; //Use for motor stall safety monitoring

int offset = 0; //offset for adjusting the encoder value, since it's read-only
volatile boolean positionFault = false; //flag to detect mis-reads of dial position
const int CCW_FLAG_CROSSOVER = 6543; //this is the positon at which the flag is hit spinning in a CCW direction. Set after manual calibration.
const int CW_FLAG_CROSSOVER = 7602; //this is the positon at which the flag is hit spinning in a CW direction. Set after manual calibration.
const int POS_ERR_TOLERANCE = 10; //tolerance on either side before a positionFault is triggered
// these are used for manual calibration of photo-interrupter fault detection. Option 'm' in menu.
volatile int CCWFlagCrossing = 0;
volatile int CWFlagCrossing = 0;
volatile boolean flagCrossed = false;

boolean direction = CW; //Commanded direction
boolean previousDirection = CW; //Detects when direction changes to add some steps for encoder slack
volatile boolean encoderDirection = CW; //This separately tracks the direction of turn as measured by the encoder
int homeOffsetSteps = 77.9 * 84; //More accurate offset - includes fractional dial value

//Because we're switching directions we need to add extra steps to take up the slack in the encoder
//The greater the adjustment the more negative it goes
int switchDirectionAdjustment = (84 * 0) + 40; //Use 'Test dial control' to determine adjustment size
//84 * 1 - 20 = Says 34 but is actually 33.5 (undershoot)
//84 * 0 = Says 85 but is actually 85.9 (overshoot)

//DiscA goes in CCW fashion during testing, increments 3 each new try.
//We don't know if this is the center of the notch. If you exhaust the combination domain, adjust up or down one.
#define DISCA_START 0

//DiscB goes in CW fashion during testing, increments 3 each new try.
#define DISCB_START (DISCA_START - 3)

//DiscC goes in CCW fashion during testing. 12 indentations, 12 raised bits. Indents are 4.17 numbers wide.
#define DISCC_START 0

//These are the combination numbers we are attempting
int discA = DISCA_START;
int discB = DISCB_START;
int discC = DISCC_START;

//int discA = 30; //Testing only. Don't use.
//int discB = 55;
//int discC = 47;

//Keeps track of the combos we need to try for each disc
//byte maxAAttempts = 33; //Assume solution notch is 3 digits wide
//byte maxBAttempts = 33; //Assume solution notch is 3 digits wide
byte maxCAttempts = 0; //Calculated at startup

//Keeps track of how many combos we've tried on a given disc
//byte discAAttempts = 0;
//byte discBAttempts = 0;
byte discCAttempts = 0;

long startTime; //Used to measure amount of time taken per test

// boolean indentsToTry[12] = {false, true, true, true, true, true, true, true, true, false, false, false}; //Keeps track of the indents we want to try
boolean indentsToTry[12] = {false, false, false, false, false, false, false, false, false, true, true, true}; //Keeps track of the indents we want to try
int indentLocations[12] = {98, 6, 14, 23, 31, 40, 48, 56, 65, 73, 81, 90}; //indent centers as meausured. Set as appropriate
int indentWidths[12]; //Calculated width of a given indent
int indentDepths[12]; //Not really used

int coarseSpeed = 100; //Speed at which we get to coarse window (0-255). 150, 200 works. 210, 230 fails
int fineSpeed = 60; //Less than 50 may not have enough torque.


void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println();
  Serial.println("Safe Cracker");
  delay(100);

  pinMode(levelConverterEnable, OUTPUT);
  digitalWrite(levelConverterEnable, HIGH); // enable 3.3/5V level converter

  pinMode(motorReset, OUTPUT);
  disableMotor();

  //Motor encoder setup for Arduino Due
  // activate peripheral functions for quad pins
  REG_PIOB_PDR = mask_encoder_A;     // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_encoder_A;   // choose peripheral option B   
  REG_PIOB_PDR = mask_encoder_B;     // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_encoder_B;   // choose peripheral option B

  // activate clock for TC0
  REG_PMC_PCER0 = PMC_PCER0_PID27;   //#define PMC_PCER0_PID27 (0x1u << 27) /**< \brief (PMC_PCER0) Peripheral Clock 27 Enable */
  // select XC0 as clock source
  REG_TC0_CMR0 = TC_CMR_TCCLKS_XC0;  //#define   TC_CMR_TCCLKS_XC0 (0x5u << 0) /**< \brief (TC_CMR) Clock selected: XC0 */
  //activate quadrature encoder and position measure mode, no filters
  REG_TC0_BMR = TC_BMR_QDEN          //#define TC_BMR_QDEN (0x1u << 8) /**< \brief (TC_BMR) Quadrature Decoder ENabled */
              | TC_BMR_POSEN         //#define TC_BMR_POSEN (0x1u << 9) /**< \brief (TC_BMR) POSition ENabled */
              | TC_BMR_EDGPHA        //#define TC_BMR_EDGPHA (0x1u << 12) /**< \brief (TC_BMR) Edges are detected on both PHA and PHB */
              | TC_BMR_SWAP;         //#define TC_BMR_SWAP (0x1u << 16) /**< \brief (TC_BMR) SWAP PHA and PHB */
  // enable the clock (CLKEN=1) and reset the counter (SWTRG=1)
  REG_TC0_CCR0 = TC_CCR_CLKEN        //#define TC_CCR_CLKEN (0x1u << 0) /**< \brief (TC_CCR) Counter Clock Enable Command */
               | TC_CCR_SWTRG;       //#define TC_CCR_SWTRG (0x1u << 2) /**< \brief (TC_CCR) Software Trigger Command */


  pinMode(motorPWM, OUTPUT);
  pinMode(motorDIR, OUTPUT);

  pinMode(servoPosition, INPUT);

  pinMode(photo, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(photo), calibrationCheck, FALLING);
  // the following configures debouncing on the photo-interrupter pin
  g_APinDescription[photo].pPort -> PIO_DIFSR |= g_APinDescription[photo].ulPin; // Input Filter Enable Register on photo pin
  PIOB->PIO_DIFSR |= 1<<26; // Debouncing Input Filter Select Register
  PIOB->PIO_SCDR |= 0xff; // Slow Clock Divider Register (to one second?)


  pinMode(servoPositionButton, INPUT);
  // attachInterrupt(digitalPinToInterrupt(servoPositionButton), buttonPushed, FALLING);

  // //Setup the encoder interrupts (for Arduino Uno only)
  // pinMode(encoderA, INPUT);
  // pinMode(encoderB, INPUT);
  // attachInterrupt(digitalPinToInterrupt(encoderA), aChangeSimple, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(encoderB), bChangeSimple, CHANGE);

  Serial.print(F("Home Offset dial/steps: "));
  Serial.print(homeOffsetSteps / 84.0);
  Serial.print(F(" / "));
  Serial.println(homeOffsetSteps);

  Serial.println(F("Indent data"));
  for (int indentNumber = 0 ; indentNumber < 12 ; indentNumber++)
  {
    //Load the yes/no for each indent To Try
    // indentsToTry[indentNumber] = EEPROM.read(LOCATION_TEST_INDENT_0 + indentNumber); //Boolean
    
    //Get the encoder values for each indent
    // EEPROM.get(LOCATION_INDENT_DIAL_0 + (indentNumber * 2), indentLocations[indentNumber]); //Addr, loc. Encoder click for a given indent

    Serial.print(F("IndentNum["));
    Serial.print(indentNumber);
    Serial.print(F("] Encoder["));
    Serial.print(indentLocations[indentNumber]);
    Serial.print(F("] / Dial["));
    Serial.print(indentLocations[indentNumber]);
    // Ignore these, since manually measured and no EEPROM on DUE
    // Serial.print(F("] / Width["));
    // Serial.print(indentWidths[indentNumber]);
    // Serial.print(F("] / Depth["));
    // Serial.print(indentDepths[indentNumber]);
    Serial.print(F("] Test["));

    //Print Test if indent will be tested
    if (indentsToTry[indentNumber] == true) Serial.print("Y");
    else Serial.print("N");
    Serial.print(F("]"));

    Serial.println();

  }

  //Get servo settings
  // EEPROM.get(LOCATION_SERVO_REST, servoRestingPosition);
  // EEPROM.get(LOCATION_SERVO_TEST_PRESSURE, servoTryPosition);
  // EEPROM.get(LOCATION_SERVO_HIGH_PRESSURE, servoHighPressurePosition);


  Serial.print(F("servo: resting["));
  Serial.print(servoRestingPosition);
  Serial.print(F("] try["));
  Serial.print(servoTryPosition);
  Serial.print(F("] HighPressure["));
  Serial.print(servoHighPressurePosition);
  Serial.print(F("]"));
  Serial.println();

  //Setup servo
  handleServo.attach(servo);
  handleServo.write(servoRestingPosition); //Goto the resting position (handle horizontal, door closed)
  delay(timeServoRelease * 3); //Allow servo to release, may be in solution slot

  randomSeed(analogRead(A5));

  //Calculate how many indents we need to attempt on discC
  maxCAttempts = 0;
  for (byte x = 0 ; x < 12 ; x++)
    if (indentsToTry[x] == true) maxCAttempts++;
  Serial.print(F("maxCAttempts: "));
  Serial.print(maxCAttempts);
  Serial.println();

  //At startup discB may be negative. Fix it.
  if (discB < 0) discB += 100;

  //Tell dial to go to zero
  enableMotor(); //Turn on motor controller
  findFlag(); //Find the flag
  setDial(0, false); //Make dial go to zero
  Serial.print(F("Dial should be at 0, is at: "));
  printEncoderToDial(getEncoderSteps());
}

void loop()
{
  char incoming;

  //Main serial control menu
  Serial.println();
  Serial.print(F("Combo to start at: "));
  Serial.print(discA);
  Serial.print("/");
  Serial.print(discB);
  Serial.print("/");
  Serial.print(discC);
  Serial.println();

  Serial.println(F("1) Go home and reset dial"));
  Serial.println(F("2) Test dial control"));
  Serial.println(F("3) View indent positions"));
  Serial.println(F("4) Measure indents"));
  Serial.println(F("5) Set indents to test"));
  Serial.println(F("6) Set starting combos"));
  Serial.println(F("7) Calibrate handle servo"));
  Serial.println(F("8) Test handle servo tryHandle()"));
  Serial.println(F("m) Measure positions of photo detector"));
  Serial.println(F("p) Move dial to a position"));
  Serial.println(F("f) Find flag and recenter dial"));
  Serial.println(F("s) Start cracking"));

  while (!Serial.available()); //Wait for user input
  incoming = Serial.read();

  if (incoming == '1')
  {
    //Go to starting conditions
    findFlag(); //Detect the flag and center the dial

    Serial.print(F("Home offset is: "));
    Serial.println(homeOffsetSteps / 84.0);

    float zeroLocation = 0;
    while (1) //Loop until we have good input
    {
      Serial.print(F("Enter where dial is actually at: "));

      while (!Serial.available()); //Wait for user input

      zeroLocation = Serial.parseFloat(); //Read user input

      Serial.print(zeroLocation);
      if (zeroLocation >= 0 && zeroLocation <= 99) break;
      Serial.println(F(" out of bounds"));
    }

    homeOffsetSteps = zeroLocation * 84;

    Serial.print(F("\n\rSetting home offset (in steps) to: "));
    Serial.println(homeOffsetSteps);

    //Adjust steps with the real-world offset
    encoderSteps = homeOffsetSteps;

    setDial(0, false); //Turn to zero

    Serial.print(F("Dial should be at 0, is at: "));
    printEncoderToDial(getEncoderSteps());
  }
  else if (incoming == '2')
  {
    positionTesting(); //Pick random places on dial and prove we can go to them
  }
  else if (incoming == '3')
  {
    //View indent center values
    for (int x = 0 ; x < 12 ; x++)
    {
      Serial.print(x);
      Serial.print(F(": ["));
      Serial.print(lookupIndentValues(x));
      Serial.print(F("]"));
      Serial.println();
    }
  }
  else if (incoming == '4')
  {
    //Measure indents

    int measurements = 0;
    while (1) //Loop until we have good input
    {
      Serial.print(F("How many measurements would you like to take? (Start with 1)"));
      while (!Serial.available()); //Wait for user input
      measurements = Serial.parseInt(); //Read user input

      if (measurements >= 1 && measurements <= 20) break;

      Serial.print(measurements);
      Serial.println(F(" out of bounds"));
    }
    Serial.println(measurements);

    measureDiscC(measurements); //Try to measure the idents in disc C. Give function the number of tests to run (get average).
  }
  else if (incoming == '5')
  {
    while (1) //Loop until exit
    {
      int indent = 0;
      while (1) //Loop until we have valid input
      {
        Serial.println("Indents to test:");
        for (int x = 0 ; x < 12 ; x++)
        {
          Serial.print(x);
          Serial.print(": ");

          //Print Test if indent will be tested
          if (indentsToTry[x] == true) Serial.print("Y");
          else Serial.print("N");

          //Print dial value of this indent
          Serial.print(" / ");
          Serial.print(indentLocations[x]);

          Serial.println();
        }
        Serial.println("Which indent to change?");
        Serial.println("Type 99 to exit");

        while (!Serial.available()); //Wait for user input

        indent = Serial.parseInt(); //Read user input

        if (indent >= 0 && indent <= 11) break;
        if (indent == 99) break;

        Serial.print(indent);
        Serial.println(F(" out of bounds"));
      }

      if (indent == 99) break; //User wants to exit

      //Flip it
      if (indentsToTry[indent] == true) indentsToTry[indent] = false;
      else indentsToTry[indent] = true;

      // //Record current settings to EEPROM
      // EEPROM.put(LOCATION_TEST_INDENT_0 + indent, indentsToTry[indent]);
    }

    //Calculate how many indents we need to attempt on discC
    maxCAttempts = 0;
    for (byte x = 0 ; x < 12 ; x++)
      if (indentsToTry[x] == true) maxCAttempts++;
  }
  else if (incoming == '6')
  {
    //Set starting combos
    for (byte x = 0 ; x < 3 ; x++)
    {
      int combo = 0;
      while (1) //Loop until we have good input
      {
        Serial.print(F("Enter Combination "));
        if (x == 0) Serial.print("A");
        else if (x == 1) Serial.print("B");
        else if (x == 2) Serial.print("C");
        Serial.print(F(" to start at: "));
        while (!Serial.available()); //Wait for user input

        combo = Serial.parseInt(); //Read user input

        if (combo >= 0 && combo <= 99) break;

        Serial.print(combo);
        Serial.println(F(" out of bounds"));
      }
      Serial.println(combo);
      if (x == 0) discA = combo;
      else if (x == 1) discB = combo;
      else if (x == 2) discC = combo;
    }

  }
  else if (incoming == '7')
  {
    testServo();
  }
  else if (incoming == '8')
  {
    tryHandle();
  }
  else if (incoming == 'p')
  {
    detailedPositionTesting();
  }
  else if (incoming == 'f')
  {
    findFlag(); //Find the flag
    setDial(0, false); //Make dial go to zero
    Serial.print(F("Dial should be at 0, is at: "));
    printEncoderToDial(getEncoderSteps());
    Serial.read(); //clear out remaining byte
  }
  else if (incoming == 'a')
  {
    handleServo.write(servoRestingPosition);
  }
  else if (incoming == 'z')
  {
    handleServo.write(servoTryPosition);
  }
  else if (incoming == 's') //Start cracking!
  {
    startTime = millis();

    buttonWasPushed = false; //reset door open sensor

    //Set the discs to the current combinations (user can set if needed from menu)
    resetDiscsWithCurrentCombo(true); //pause to confirm for some measure of confidence that dials are operating correctly

    while (1)
    {
      nextCombination(); //Try the next combo!

      if (Serial.available())
      {
        byte incoming = Serial.read();
        if (incoming == 'p')
        {
          Serial.println(F("Pausing"));
          while (!Serial.available());
          Serial.read();
          Serial.println(F("Running"));
        }
        else if (incoming == 'x' || incoming == 's')
        {
          Serial.println(F("Cracking stopped"));
          break; //User wants to stop
        }
      }
    } //End eombination loop
  } //End incoming == 's'
  else if (incoming == 't')
  {
    while (1)
    {
      if (Serial.available())
      {
        // exit if any input detected
        Serial.read();
        break;
      }
      Serial.println(getEncoderSteps());
    }
  }
  else if (incoming == 'm')
  {
    // Measure encoder step value at detection of photo interrupter from the CW and CCW rotation

    // First measure running fineSpeed, going CCW
    int sum = 0;
    int count = 0;
    turnCCW();
    setMotorSpeed(coarseSpeed); //Go!
    while (count < 10) {
        while (flagCrossed == false);
        flagCrossed = false; // reset flag
        Serial.print("CCW flag crossing at: ");
        Serial.println(CCWFlagCrossing);
        sum += CCWFlagCrossing;
        count++;
    }
    setMotorSpeed(0);
    Serial.print("CCW flag crossing average: ");
    Serial.println(sum / 10);

    delay(500);

    // Next measure running fineSpeed, going CW
    sum = 0;
    count = 0;
    turnCW();
    setMotorSpeed(coarseSpeed); //Go!
    while (count < 10) {
        while (flagCrossed == false);
        flagCrossed = false; // reset flag
        Serial.print("CCW flag crossing at: ");
        Serial.println(CCWFlagCrossing);
        sum += CCWFlagCrossing;
        count++;
    }
    setMotorSpeed(0);
    Serial.print("CCW flag crossing average: ");
    Serial.println(sum / 10);


  }
//   else if (incoming = 'n')
//   {
//     turnCW();
//     setMotorSpeed(200); //Go!
//     delay(1000);
//     setMotorSpeed(0); //stop
//     Serial.print(F("CW encoder errors (AR/AF/BR/BF): "));
//     Serial.print(numErrorsAR);
//     Serial.print("/");
//     Serial.print(numErrorsAF);
//     Serial.print("/");
//     Serial.print(numErrorsBR);
//     Serial.print("/");
//     Serial.print(numErrorsBF);
//     Serial.print("/");
//     Serial.read(); //clear out remaining byte
//   }
}

// These functions were written for the Uno, but are removed for the Due.
// void writeIntIntoEEPROM(int address, int number)
// { 
//   byte byte1 = number >> 8;
//   byte byte2 = number & 0xFF;
//   EEPROM.write(address, byte1);
//   EEPROM.write(address + 1, byte2);
// }

// int readIntFromEEPROM(int address)
// {
//   byte byte1 = EEPROM.read(address);
//   byte byte2 = EEPROM.read(address + 1);
//   return (byte1 << 8) + byte2;
// }