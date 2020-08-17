/*
  These functions are used to calibrate the dialer and handle servo
*/

//Spins the motor while we tweak the servo up/down to detect binding force and servo position
//Press x to exit
//How to use: Attach cracker to safe.
//Start the servo at the rest position. As motor spins increase handle pressure using a/z until the gear
//just begins to rub. The gate will have set into one of the indents by now, and motor/dial will be stuck there.
//This will be the servo highPressurePosition. Servo tryPosition will be 10 less than this position, and the 
//openHandle position will be set 30 less than the handle position at servoHighPressurePosition.
void testServo()
{
  Serial.println(F("x to exit"));
  Serial.println(F("a to adjust servo to gear"));
  Serial.println(F("z to adjust servo away from gear"));

  int servo = servoRestingPosition;
  handleServo.write(servo);

  servoHighPressurePosition = servoRestingPosition;

  long timeSinceLastMovement = millis();
  int lastStep = steps;

  // turnCW();
  // setMotorSpeed(50);

  // enableMotor(); //Turn on motor controller

  while (1)
  {
    // if (lastStep != steps)
    // {
    //   lastStep = steps;
    //   timeSinceLastMovement = millis();
    // }
    // if (millis() - timeSinceLastMovement > 25)
    // {
    //   setMotorSpeed(0); //Stop!
    //   Serial.println("Dial stuck");
    //   // while (1);
    // }

    if (Serial.available())
    {
      byte incoming = Serial.read();

      if (incoming == 'a') servo+=3;
      if (incoming == 'z') servo-=3;
      if (incoming == 'x') //Exit
      {
        setMotorSpeed(0); //Stop!
        handleServo.write(servoRestingPosition); //Goto the resting position (handle horizontal, door closed)
        delay(timeServoRelease); //Allow servo to move
        break;
      }

      if (servo < 0) servo = 0;
      if (servo > 255) servo = 255;

      handleServo.write(servo); //Goto the resting position (handle horizontal, door closed)
    }
    int handlePosition = averageAnalogRead(servoPosition); //Old way
    // int handlePosition = digitalRead(servoPositionButton); //Look for button being pressed

    if (servo < servoHighPressurePosition) servoHighPressurePosition = servo;

    Serial.print(F("servo: "));
    Serial.print(servo);
    // Serial.print(F(" / handleButton: "));
    // Serial.print(servoPositionButton);
    Serial.print(F(" / handlePosition: "));
    Serial.print(handlePosition);
    Serial.println();

    delay(100);
  }

  // //Record settings to EEPROM
  // servoRestingPosition = servo; //At the end of calibration, servo position should be set at the desired resting psoition
  servoTryPosition = servoHighPressurePosition - 10;
  // handleOpenPosition = handlePosition - 30; // hopefully 30 less is not too much to identify an opened handle

  // //17 was found in testing to be the min servo position, with 217 the max
  // if (servoHighPressurePosition < 17 || servoTryPosition < 17)
  // {
  //   Serial.println(F("servoHighPressurePosition or servoTryPosition is too small. Adjust servo higher."));
  //   Serial.println(F("Freezing"));
  //   while (1); //Freeze
  // }

  // EEPROM.put(LOCATION_SERVO_REST, servoRestingPosition);
  // EEPROM.put(LOCATION_SERVO_TEST_PRESSURE, servoTryPosition);
  // EEPROM.put(LOCATION_SERVO_HIGH_PRESSURE, servoHighPressurePosition);

  // Serial.print(F("servo: resting["));
  // Serial.print(servoRestingPosition);
  // Serial.print(F("] try["));
  // Serial.print(servoTryPosition);
  // Serial.print(F("] Highpressure["));
  // Serial.print(servoHighPressurePosition);
  // Serial.print(F("]"));
  // Serial.println();

  // Serial.println(F("Servo positions stored."));

}

//Pulls down on handle using current settings
//Reports if button has been pressed
//To use: before running function identify the solution slot
//Dial will turn to slot
//Press any letter to cause servo to pull on handle
//If button is pressed, it will print Pressed!!
//Adjust button down just below level of being pressed when handle is normally pulled on
//Press x to exit
// void testHandleButton(void)
// {
//   //Find the indent to test
//   int solutionDiscC = 0;
//   for (int x = 0 ; x < 12 ; x++)
//   {
//     if (indentsToTry[x] == true)
//       solutionDiscC = convertEncoderToDial(indentLocations[x]);
//   }
//   setDial(solutionDiscC, false); //Goto this dial location

//   Serial.println("x to exit");
//   Serial.println("a to pull on handle");
//   Serial.println("z to release handle");

//   int pressedCounter = 0;
//   while (1)
//   {
//     if (Serial.available())
//     {
//       byte incoming = Serial.read();
//       if (incoming == 'x')
//       {
//         //Release handle
//         handleServo.write(servoRestingPosition); //Goto the resting position (handle horizontal, door closed)
//         delay(timeServoRelease); //Allow servo to release

//         return; //Exit
//       }
//       else if (incoming == 'a')
//       {
//         //Pull on handle
//         handleServo.write(servoTryPosition);
//         delay(timeServoApply); //Allow servo to move
//       }
//       else if (incoming == 'z')
//       {
//         //Release handle
//         handleServo.write(servoRestingPosition); //Goto the resting position (handle horizontal, door closed)
//         delay(timeServoRelease); //Allow servo to release
//       }
//     }

//     if (digitalRead(servoPositionButton) == LOW)
//     {
//       pressedCounter++;
//       Serial.print("Pressed! ");
//       Serial.println(pressedCounter); //To have something that changes
//       delay(100);
//     }

//     //Hang out for 100ms but scan button during that time
//     for (byte x = 0 ; x < 100 ; x++)
//     {
//       if (digitalRead(servoPositionButton) == LOW) break;
//       delay(1);
//     }
//   }

// }

//Test to see if we can repeatably go to a dial position
//Turns dial to random CW and CCW position and asks user to verify.
//How to use: Attach cracker to safe. Home the dial using the menu function. Then run this
//and verify the dial goes where it says it is going. If it's wrong, check homeOffset variable.
//If one direction is off, check switchDirectionAdjustment variable.
void positionTesting()
{
  int randomDial;

  for (int x = 0 ; x < 5 ; x++)
  {
    randomDial = random(0, 100);
    randomDial = 10;
    turnCCW();
    setDial(randomDial, false);

    // Serial.print(F("Dial commanded CCW to: "));
    // Serial.print(randomDial);
    Serial.print(F(", Dial should be at: "));
    Serial.println(convertEncoderToDial(steps));
    messagePause("Verify then press key to continue");

    randomDial = random(0, 100);
    randomDial = 60;
    turnCW();
    setDial(randomDial, false);

    // Serial.print(F("Dial commanded CW to: "));
    // Serial.print(randomDial);
    Serial.print(F(", Dial should be at: "));
    Serial.println(convertEncoderToDial(steps));
    messagePause("Verify then press key to exit");
  }
}

void detailedPositionTesting() {
  Serial.println(F("x to exit"));
  Serial.println(F("d to toggle direction"));
  Serial.println(F("s to set motor speed (0 is no turn, 255 is max)"));
  Serial.println(F("q to set desired dial position"));
  Serial.println(F("w to set desired dial position (with extra spin)"));

  while (1) {
    Serial.flush();
    while (!Serial.available()); //Wait for user input
    Serial.setTimeout(30000); //Must be long enough for user to enter second character
    byte command = Serial.read();
    Serial.flush(); //Throw away CRLF
    // Serial.println(command);

    if (command == 'd') {
      previousDirection = direction;
      direction ^= true; //Toggle direction

      Serial.print(F("New direction: "));
      if (direction == CW) {
        Serial.println("CW");
        turnCW();
      }
      else {
        Serial.println("CCW");
        turnCCW();
      }
    } 
    else if (command == 's') {
      Serial.println(F("Enter new speed"));
      while (!Serial.available()); //Wait for user input
      Serial.setTimeout(30000); //Must be long enough for user to enter second character
      int speed = Serial.parseInt();

      setMotorSpeed(speed);
      Serial.print(F("New motor speed: "));
      Serial.println(speed);
    } 
    else if (command == 'q') {
      Serial.print(F("Enter desired position: "));
      while (!Serial.available()); //Wait for user input
      Serial.setTimeout(30000); //Must be long enough for user to enter second character

      int position = Serial.parseInt();
      Serial.println(position);
      setDial(position, false);
      Serial.print(F("Encoder position: "));
      printEncoderToDial(steps);
    } 
    else if (command == 'w') {
      Serial.print(F("Enter desired position (with extra spin): "));
      while (!Serial.available()); //Wait for user input
      Serial.setTimeout(30000); //Must be long enough for user to enter second character

      int position = Serial.parseInt();
      Serial.println(position);
      setDial(position, true);
      Serial.print(F("Encoder position: "));
      printEncoderToDial(steps);
    } 
    else if (command == 'x') {
      setMotorSpeed(0); //Stop motor
      break;
    } 
    else {
      // Serial.println(F("Unknown command ¯\\_(ツ)_/¯"));
    }
  }
}

void printEncoderToDial(int encoderValue)
{
  int dialValue = encoderValue / 84; //2388/84 = 28.43
  int partial = encoderValue % 84; //2388%84 = 36

  if (dialValue > 99) dialValue -= 100;

  Serial.print(dialValue);
  Serial.print(F(" / "));
  Serial.println(partial);
}


void inputTest() {
  Serial.println("=== Input test ===");

  while(1) {
    while (!Serial.available()); //Wait for user input
    Serial.setTimeout(30000); //Must be long enough for user to enter second character
    Serial.println("input read!");
    byte input = Serial.read();
    Serial.read(); //throw away CRLF

    Serial.println(input);
    if (input == 'x') break;
    else if (input == 'b') Serial.println("b");
  }

}

