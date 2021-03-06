/*
  These are the general control functions

  setDial - go to a given dial position (0 to 99)
  resetDial - spin the dial twice to move reset discs A, B, and C. Stop at 0.
  findFlag - tell dial to return to where the flag interruts the photogate. Depending on how the robot is attached to dial, this may be true zero or an offset
  setDiscsToStart - Go home, clear dial, go to starting numbers

  Supporting functions:
  gotoStep - go to a certain step value out of 8400. The DC motor has momentum so this function, as best as possible, goes to a step value
  stepRequired - calcs the steps of the encoder to get from current position to a given position
  convertDialToEncoder - given a dial number, how many encoder steps is that?
  convertEncoderToDial - given encoder value, what is the dial position?
  enable/disableMotor - controls the enable pin on motor driver
  turnCW/CCW - controls the direction pin on motor driver

*/

//Given a step value, go to that step
//Assume user has set direction prior to calling
//Returns the delta from what you asked and where it ended up
//Adding a full rotation will add a 360 degree full rotation
int gotoStep(int stepGoal, boolean addAFullRotation)
{
  //Coarse window control how fast we arrive at the digit on the dial
  //Having too small of a window or too fast of an attack will make the dial overshoot.
  int coarseWindow = 1680; //Once we are within this amount, switch to fine adjustment (because of dial-past logic, must be less than 25*84)
  int fineWindow = 30; //One we are within this amount, stop searching

  //Because we're switching directions we need to add extra steps to take up the slack in the encoder
  if (direction == CW && previousDirection == CCW)
  {
    // encoderSteps += switchDirectionAdjustment;
    // if (encoderSteps > 8400) encoderSteps -= 8400;
    setEncoderSteps(getEncoderSteps() + switchDirectionAdjustment);
    previousDirection = CW;
  }
  else if (direction == CCW && previousDirection == CW)
  {
    // encoderSteps -= switchDirectionAdjustment;
    // if (encoderSteps < 0) encoderSteps += 8400;
    setEncoderSteps(getEncoderSteps() - switchDirectionAdjustment);
    previousDirection = CCW;
  }

  setMotorSpeed(coarseSpeed); //Go!
  while (stepsRequired(getEncoderSteps(), stepGoal) > coarseWindow); //Spin until coarse window is closed
  // while (stepsRequired(steps, stepGoal) > coarseWindow) Serial.println(steps); //log dial position

  //After we have gotten close to the first coarse window, proceed past the goal, then proceed to the goal
  if (addAFullRotation == true)
  {
    int tempStepGoal = getEncoderSteps() + 8400/2; //Move 50 away from current position
    if (tempStepGoal > 8400) tempStepGoal -= 8400;
    
    //Go to temp position
    while (stepsRequired(getEncoderSteps(), tempStepGoal) > coarseWindow); 
    // while (stepsRequired(steps, tempStepGoal) > coarseWindow) Serial.println(steps);
        
    //Go to stepGoal
    while (stepsRequired(getEncoderSteps(), stepGoal) > coarseWindow); //Spin until coarse window is closed
    // while (stepsRequired(steps, stepGoal) > coarseWindow) Serial.println(steps);
  }

  setMotorSpeed(fineSpeed); //Slowly approach

  while (stepsRequired(getEncoderSteps(), stepGoal) > fineWindow); //Spin until fine window is closed
  // while (stepsRequired(steps, stepGoal) > fineWindow) Serial.println(steps);

  setMotorSpeed(0); //Stop

  //log dial behavior at end of a turn after commanded stop
  // for (int i = 0; i < 300; i++) {
  //   Serial.println(steps);
  // }

  delay(timeMotorStop); //Wait for motor to stop

  // Serial.print(F("Encoder errors: "));
  // Serial.println(numErrors);

  int delta = getEncoderSteps() - stepGoal;

  // if |delta| > 90 dial ticks, it's probably just near the 0 rollover point
  if (delta > 90*84) {
    delta -= 8400;
  } 
  else if (delta <  -90*84) {
    delta += 8400;
  }

  /*if (direction == CW) Serial.print("CW ");
    else Serial.print("CCW ");
    Serial.print("stepGoal: ");
    Serial.print(stepGoal);
    Serial.print(" / Final steps: ");
    Serial.print(steps);
    Serial.print(" / delta: ");
    Serial.print(delta);
    Serial.println();*/

  return (delta);
}

//Calculate number of steps to get to our goal
//Based on current position, goal, and direction
//Corrects for 8400 roll over
int stepsRequired(int currentSteps, int goal)
{
  if (direction == CW)
  {
    if (currentSteps >= goal) return (currentSteps - goal);
    else if (currentSteps < goal) return (8400 - goal + currentSteps);
  }
  else if (direction == CCW)
  {
    if (currentSteps >= goal) return (8400 - currentSteps + goal);
    else if (currentSteps < goal) return (goal - currentSteps);
  }

  Serial.println(F("stepRequired failed")); //We shouldn't get this far
  Serial.print(F("Goal: "));
  Serial.println(goal);
  Serial.print(F("currentSteps: "));
  Serial.println(currentSteps);

  return (0);
}

//Given a dial number, goto that value
//Assume user has set the direction before calling
//Returns the delta from gotoStep() 
int setDial(int dialValue, boolean extraSpin)
{
  Serial.print(F("Commanded "));
  direction == CCW ? Serial.print("CCW") : Serial.print("CW");
  Serial.print(F(" to "));
  Serial.print(dialValue);
  Serial.print(", ");

  int encoderValue = convertDialToEncoder(dialValue); //Get encoder value

  // unsigned long start = millis();
  int stepDelta = gotoStep(encoderValue, extraSpin); //Goto that encoder value
  if (stepDelta < -stepTolerance || stepDelta > stepTolerance) {
    Serial.print(F("Dial did not arrive at commanded value, step ∆ = "));
    Serial.println(stepDelta);
  }

  int actualDialValue = convertEncoderToDial(getEncoderSteps()); //Convert back to dial values
  //Serial.print("After movement, dialvalue: ");
  //Serial.println(actualDialValue);

  return (stepDelta);
}

//Find the flag and set the encoder to the predefined homeOffset value
void findFlag()
{
  byte fastSearch = 255; //Speed at which we locate photogate
  byte slowSearch = 60;


  //If the photogate is already detected spin until we are out
  if (flagDetected() == true)
  {
    Serial.println(F("We're too close to the photogate"));
    int currentDial = convertEncoderToDial(getEncoderSteps());
    currentDial += 50;
    if (currentDial > 100) currentDial -= 100;
    setDial(currentDial, false); //Advance to 50 dial ticks away from here
  }

  //Begin spinning
  // turnCW();
  // setMotorSpeed(fastSearch);

  // while (flagDetected() == false) delayMicroseconds(1); //Spin freely

  // //Ok, we just zipped past the gate. Stop and spin slowly backward
  // setMotorSpeed(0);
  // delay(timeMotorStop); //Wait for motor to stop spinning
  
  turnCCW();
  setMotorSpeed(slowSearch);
  while (flagDetected() == false) delayMicroseconds(1); //Find flag

  setMotorSpeed(0);
  delay(timeMotorStop); //Wait for motor to stop

  //Adjust steps with the real-world offset
  // encoderSteps = homeOffsetSteps;
  setEncoderSteps(homeOffsetSteps);

  previousDirection = CCW; //Last adjustment to dial was in CCW direction
  Serial.println(F("Flag found"));
}

//Set the discs to the current combinations (user selectable)
void resetDiscsWithCurrentCombo(boolean pause)
{
  resetDial(); //Clear out everything

  //Set discs to this combo
  turnCCW();
  int discADelta = setDial(discA, false);
  Serial.print(F("DiscA commanded to: "));
  Serial.println(discA);
  Serial.print("DiscA is at: ");
  printEncoderToDial(getEncoderSteps());
  if (pause == true) messagePause("Verify disc position");

  turnCW();
  //Turn past disc B one extra spin
  Serial.print(F("DiscB commanded to: "));
  Serial.println(discB);
  int discBDelta = setDial(discB, true);
  Serial.print("DiscB is at: ");
  printEncoderToDial(getEncoderSteps());
  if (pause == true) messagePause("Verify disc position");

  turnCCW();
  Serial.print(F("DiscC commanded to: "));
  Serial.println(discC);
  int discCDelta = setDial(discC, false);
  Serial.print("DiscC is at: ");
  printEncoderToDial(getEncoderSteps());
  if (pause == true) messagePause("Verify disc position");
  
  // if we detect a mis-actuation of the dial on the last spin, retry the combination
  if (discADelta < -stepTolerance || discADelta > stepTolerance
      || discBDelta < -stepTolerance || discBDelta > stepTolerance
      || discCDelta < -stepTolerance || discCDelta > stepTolerance)
  {
    Serial.print(F("Detected mis-actuation of dial, discA∆ / discB∆ / discC∆: "));
    Serial.print(discADelta);
    Serial.print(F(" / "));
    Serial.print(discBDelta);
    Serial.print(F(" / "));
    Serial.println(discCDelta);
    Serial.println(F("Retrying current combo after re-finding flag."));
    
    findFlag(); //Re-home the dial between large finds
    resetDiscsWithCurrentCombo(false);
  }

  // if we detect a dial fault via flag position, retry the combination
  if (dialFaultDetected()) {
    Serial.println();
    Serial.print(F("Detected dial position fault. Flag did not cross at expected position. CW flag crossing at: "));
    Serial.print(CWFlagCrossing);
    flagCrossed = false; // reset detection flag
    Serial.println(F("Retrying current combo after re-finding flag."));
    
    findFlag(); //Re-home the dial between large finds
    resetDiscsWithCurrentCombo(false);

  }

  discCAttempts = -1; //Reset
}


//Given a dial value, covert to an encoder value (0 to 8400)
//If there are 100 numbers on the dial, each number is 84 ticks wide
int convertDialToEncoder(int dialValue)
{
  int encoderValue = dialValue * 84;

  if (encoderValue > 8400)
  {
    Serial.print("Whoa! Your trying to go to a dial value that is illegal. encoderValue: ");
    Serial.println(encoderValue);
    while (1);
  }

  return (84 * dialValue);
}

//Given an encoder value, tell me where on the dial that equates
//Returns 0 to 99
//If there are 100 numbers on the dial, each number is 84 ticks wide
int convertEncoderToDial(int encoderValue)
{
  int dialValue = encoderValue / 84; //2388/84 = 28.43
  int partial = encoderValue % 84; //2388%84 = 36

  if (partial >= (84 / 2)) dialValue++; //36 < 42, so 28 is our final answer

  if (dialValue > 99) dialValue -= 100;

  return (dialValue);
}

//Reset the dial
//Turn CCW, past zero, then continue until we return to zero
void resetDial()
{
  disableMotor();

  turnCCW();

  setMotorSpeed(coarseSpeed); //Go at coarse speed
  enableMotor();

  //Spin until 8400*2 steps have gone by
  int deltaSteps = 0;
  while (deltaSteps < (8400 * 2))
  {
    int startingSteps = getEncoderSteps(); //Remember where we started
    delay(100); //Let motor spin for awhile

    if (getEncoderSteps() >= startingSteps) deltaSteps += getEncoderSteps() - startingSteps;
    else deltaSteps += (8400 - startingSteps + getEncoderSteps());
  }
  
  setMotorSpeed(0); //Stop
  delay(timeMotorStop); //Alow motor to spin to a stop

  previousDirection = CCW;
}

//Tells the servo to pull down on the handle
//After a certain amount of time we test to see the actual
//position of the servo. If it's properly moved all the way down
//then the handle has moved and door is open!
//If position is not more than a determined amount, then return failure
boolean tryHandle()
{
  //Attempt to pull down on handle
  handleServo.write(servoTryPosition-5); //Have to overshoot a bit and later retract to get a good reading
  delay(timeServoApply); //Wait for servo to move, but don't let it stall for too long and burn out
  // handleServo.write(servoTryPosition); //Retract a bit so that servo doesn't stall
  delay(timeServoApply); //Wait for servo to move

  //Check if we're there
  if (digitalRead(servoPositionButton) == LOW)
  {
    Serial.println("Holy smokes we're there!");
    return (true);
  }


  handlePosition = averageAnalogRead(servoPosition);
  // if (handlePosition > handleOpenPosition) //Used on old servo setup
  if (handlePosition < handleOpenPosition)
  {
    //Holy smokes we're there!
    return (true);
  } else {
    Serial.print(", lowest handlePosition, ");
    Serial.print(handlePosition);
  }

  //Ok, we failed
  //Return to resting position
  handleServo.write(servoRestingPosition);
  delay(timeServoRelease); //Allow servo to release. 200 was too short on new safe

  return (false);
}

//Sets the motor speed
//0 is no turn, 255 is max
void setMotorSpeed(int speedValue)
{
  analogWrite(motorPWM, speedValue);
}

//Tell motor to turn dial clock wise
void turnCW()
{
  direction = CW;
  digitalWrite(motorDIR, HIGH);
}

//Tell motor to turn dial counter-clock wise
void turnCCW()
{
  direction = CCW;
  digitalWrite(motorDIR, LOW);
}

//Turn on the motor controller
void enableMotor()
{
  digitalWrite(motorReset, HIGH);
}

void disableMotor()
{
  digitalWrite(motorReset, LOW);
}

// *************   ISRs for measuring encoder position   *************
// In the event of an ISR taking too long, and missing an alternating edge, it will look like a directional reversal, 
// in which case we will leave step count alone, since it's better to be off by 2 than off by 3. If we miss 2 edges,
// then we'll be off by 4. If we miss 3 edges, then the same encoder edge will appear twice in succession.

// On the Arduino Uno (ATmega328P) encoderA is mapped to pin 2, encoderB to pin 3

// void aChange()
// {
//   // if (digitalRead(encoderA) == HIGH) //consider using direct port read to be faster - http://www.arduino.cc/en/Reference/PortManipulation
//   if (PIND & B00000100) //if pin2 (encoder A) is high
//   {
//     //Encoder A rising edge
//     if (lastEncoderEdge == B_RISING) {
//       //forward
//       // encoderDirection = CW;
//       steps--;
//       if (steps < 0) steps = 8399; //Limit variable to zero
//     } else if (lastEncoderEdge == B_FALLING) {
//       //backward
//       // encoderDirection = CCW;
//       steps++;
//       if (steps > 8399) steps = 0; //Limit variable to 8399
//     // } else if (lastEncoderEdge == A_FALLING) {
//       //direction reversal
//       // encoderDirection ^= true;
//     } else if (lastEncoderEdge == A_RISING) {
//       numErrors++; //should never get in here, it means 4 edges were missed!
//     }
//     lastEncoderEdge = A_RISING;
//   }
//   else
//   {
//     //Encoder A falling edge
//     if (lastEncoderEdge == B_RISING) {
//       //backward
//       // encoderDirection = CCW;
//       steps++;
//       if (steps > 8399) steps = 0; //Limit variable to 8399
//     } else if (lastEncoderEdge == B_FALLING) {
//       //forward
//       // encoderDirection = CW;
//       steps--;
//       if (steps < 0) steps = 8399; //Limit variable to zero
//     // } else if (lastEncoderEdge == A_RISING) {
//       //direction reversal
//       // encoderDirection ^= true;
//     } else if (lastEncoderEdge == A_FALLING) {
//       numErrors++; //should never get in here, it means 4 edges were missed!
//     }
//     lastEncoderEdge = A_FALLING;
//   }
// }

// void bChange()
// {
//   // if (digitalRead(encoderB) == HIGH) //consider using direct port read to be faster - http://www.arduino.cc/en/Reference/PortManipulation
//   if (PIND & B00001000) //if pin3 (encoder B) is high
//   {
//     //Encoder B rising edge
//     if (lastEncoderEdge == A_RISING) {
//       //backward
//       // encoderDirection = CCW;
//       steps++;
//       if (steps > 8399) steps = 0; //Limit variable to 8399
//     } else if (lastEncoderEdge == A_FALLING) {
//       //forward
//       // encoderDirection = CW;
//       steps--;
//       if (steps < 0) steps = 8399; //Limit variable to zero
//     // } else if (lastEncoderEdge == B_RISING) {
//       //direction reversal
//       // encoderDirection ^= true;
//     } else if (lastEncoderEdge == B_RISING) {
//       numErrors++; //should never get in here, it means 4 edges were missed!
//     }
//     lastEncoderEdge = B_RISING;
//   }
//   else
//   {
//     //Encoder B falling edge
//     if (lastEncoderEdge == A_RISING) {
//       //forward
//       // encoderDirection = CW;
//       steps--;
//       if (steps < 0) steps = 8399; //Limit variable to zero
//     } else if (lastEncoderEdge == A_FALLING) {
//       //backward
//       // encoderDirection = CCW;
//       steps++;
//       if (steps > 8399) steps = 0; //Limit variable to 8399
//     // } else if (lastEncoderEdge == B_FALLING) {
//       //direction reversal
//       // encoderDirection ^= true;
//     } else if (lastEncoderEdge == B_FALLING) {
//       numErrors++; //should never get in here, it means 4 edges were missed!
//     }
//     lastEncoderEdge = B_FALLING;
//   }
// }

// void aChangeSimple() {
//   // PIND register specific to Arduino Uno
//   // bool encA = PIND & B00000100;
//   // bool encB = PIND & B00001000;

//   // register specific to Arduino Due
//   bool encA = PIO_PDSR_P25;
//   bool encB = PIO_PDSR_P27;

//   if (encA == encB) {
//     encoderSteps--;
//     if (encoderSteps < 0) encoderSteps = 8399;
//   }
//   else {
//     encoderSteps++;
//     if (encoderSteps > 8399) encoderSteps = 0;
//   }
// }

// void bChangeSimple() {
//   // PIND register specific to Arduino Uno
//   // bool encA = PIND & B00000100;
//   // bool encB = PIND & B00001000;
  
//   // register specific to Arduino Due
//   bool encA = PIO_PDSR_P25;
//   bool encB = PIO_PDSR_P27;

//   if (encA != encB) {
//     encoderSteps--;
//     if (encoderSteps < 0) encoderSteps = 8399;
//   }
//   else {
//     encoderSteps++;
//     if (encoderSteps > 8399) encoderSteps = 0;
//   }
// }

// void aChangeTest() {
//   if (PIND & B00000100) //if pin2 (encoder A) is high
//   {
//     //Encoder A rising edge
//     if (lastEncoderEdge == A_RISING) {
//       numErrorsAR++; //should never get in here, it means 4 edges were missed!
//     }
//     lastEncoderEdge = A_RISING;
//   }
//   else
//   {
//     //Encoder A falling edge
//     if (lastEncoderEdge == A_FALLING) {
//       numErrorsAF++; //should never get in here, it means 4 edges were missed!
//     }
//     lastEncoderEdge = A_FALLING;
//   }
// }

// void bChangeTest() {
//   if (PIND & B00001000) //if pin3 (encoder B) is high
//   {
//     //Encoder B rising edge
//     if (lastEncoderEdge == B_RISING) {
//       numErrorsBR++; //should never get in here, it means 4 edges were missed!
//     }
//     lastEncoderEdge = B_RISING;
//   }
//   else
//   {
//     //Encoder B falling edge
//     if (lastEncoderEdge == B_FALLING) {
//       numErrorsBF++; //should never get in here, it means 4 edges were missed!
//     }
//     lastEncoderEdge = B_FALLING;
//   }
// }
// *************   ISRs for measuring encoder position   *************


//Checks to see if we detect the photogate being blocked by the flag
boolean flagDetected()
{
  if (digitalRead(photo) == LOW) return (true);
  return (false);
}


//Disc C is the safety disc that prevents you from feeling the edges of the wheels
//It has 12 upper and 12 low indents which means 100/24 = 4.16 per lower indent
//So it moves a bit across the wheel. We could do floats, instead we'll do a lookup
//Values were found by hand: What number is in the middle of the indent?
//And later found by using the findIndents() function - which then stores values to EEPROM
int lookupIndentValues(int indentNumber)
{
  // int indentCenterValue;
  // EEPROM.get(LOCATION_INDENT_DIAL_0 + (indentNumber * sizeof(int)), indentCenterValue); //addr, variable to put it in
  // return (convertEncoderToDial(indentCenterValue));

  /*
    switch (indentNumber)
    {
    //Values found by measureIndents() function
    case 0: return (99); //98 to 1 on the wheel
    case 1: return (8); //6-9
    case 2: return (16); //14-17
    case 3: return (24); //23-26
    case 4: return (33); //31-34
    case 5: return (41); //39-42
    case 6: return (50); //48-51
    case 7: return (58); //56-59
    case 8: return (66); //64-67
    case 9: return (74); //73-76
    case 10: return (83); //81-84
    case 11: return (91); //90-93
    case 12: return (-1); //Not valid
    }
  */
 return indentLocations[indentNumber];
}

//Print a message and wait for user to press a button
//Good for stepping through actions
void messagePause(char* message)
{
  Serial.println(message);
  while (!Serial.available()); //Wait for user input
  Serial.read(); //Throw away character
}

//See if user has requested a pause. If so, pause
void checkForUserPause()
{
  if (Serial.available()) //See if user has has requested a pause
  {
    Serial.read(); //Throw out character
    Serial.print("Pausing. Press key to continue.");

    while (!Serial.available()); //Wait for user to press key to continue
  }
}

//Given a spot on the dial, what is the next available indent in the CCW direction
//Takes care of wrap conditions
//Returns the dial position of the next ident
int getNextIndent(int currentDialPosition)
{
  for (int x = 0 ; x < 12 ; x++)
  {
    if (indentsToTry[x] == true) //Are we allowed to use this indent?
    {
      byte nextDialPosition = lookupIndentValues(x);
      if (nextDialPosition > currentDialPosition) return (nextDialPosition);
    }
  }

  //If we never found a next dial value then we have wrap around situation
  //Find the first indent we can use
  for (int x = 0 ; x < 12 ; x++)
  {
    if (indentsToTry[x] == true) //Are we allowed to use this indent?
    {
      return (lookupIndentValues(x));
    }
  }
}

//Takes an average of readings on a given pin
//Returns the average
int averageAnalogRead(byte pinToRead)
{
  byte numberOfReadings = 20;
  unsigned int runningValue = 0;

  for (int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return (runningValue);
}

void motorSafetyTest() {

  if (lastStep != getEncoderSteps()) {
    lastStep = getEncoderSteps();
    timeSinceLastMovement = millis();
  }
  if (millis() - timeSinceLastMovement > 25) {
    Serial.println("Dial stuck. Stopping motor for safety.");
    setMotorSpeed(0); //Stop!

    //Also return to resting position, in case it's being pulled
    handleServo.write(servoRestingPosition);
    delay(timeServoRelease); //Allow servo to release. 200 was too short on new safe

    while (1);
  }

}

// returns the dial steps (between 0 - 8399)
int getEncoderSteps() {
  int dialSteps = ((int) REG_TC0_CV0 + offset) % 8400;

  if (dialSteps > 0) return dialSteps;
  else return dialSteps + 8400; //if negative, roll it over to positive
}

void setEncoderSteps(int steps) {
  // enable the clock (CLKEN=1) and reset the counter (SWTRG=1)
  REG_TC0_CCR0 = TC_CCR_CLKEN        //#define TC_CCR_CLKEN (0x1u << 0) /**< \brief (TC_CCR) Counter Clock Enable Command */
               | TC_CCR_SWTRG;       //#define TC_CCR_SWTRG (0x1u << 2) /**< \brief (TC_CCR) Software Trigger Command */

  offset = steps;
}

// ISR for when servo button pushed (to detect door opening)
// Commenting out because the signal is too sensitive to noise. Running the motor for example causes many falling edges.
// void buttonPushed() {
//   buttonWasPushed = true;
// }

// ISR for checking the position of the dial during a photo-interrupter crossover event
void calibrationCheck() {
  if ((millis() - lastDebounceTime) > 2000) {
    lastDebounceTime = millis();
    int dialSteps = ((int) REG_TC0_CV0 + offset) % 8400;
    if (dialSteps <= 0) dialSteps = dialSteps + 8400; //if negative, roll it over to positive

    if (direction == CCW) CCWFlagCrossing = dialSteps;
    else if (direction == CW) CWFlagCrossing = dialSteps;
    flagCrossed = true;
  }
}

// Checks if there is a fault in the dial position based on wheter the last flag crossing happened near the expected dial position.
// - Returns true if off by more than a half-dial tick in either direction.
bool dialFaultDetected() {
    // for some reason only the CW flag crossings are recording correctly, so will use that
    if (flagCrossed == true && direction == CW) {
        flagCrossed = false; // reset flag
        if (abs(CWFlagCrossing - 6654) > 42) {
            return true;
        }
    }
    return false;
}