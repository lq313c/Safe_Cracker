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

  while (1)
  {
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

//Test to see if we can repeatably go to a dial position
//Turns dial to random CW and CCW position and asks user to verify.
//How to use: Attach cracker to safe. Home the dial using the menu function. Then run this
//and verify the dial goes where it says it is going. If it's wrong, check homeOffsetSteps variable.
//If one direction is off, check switchDirectionAdjustment variable.
void positionTesting()
{
  int randomDial;

  for (int x = 0 ; x < 5 ; x++)
  {
    randomDial = random(0, 100);
    randomDial = 10;
    turnCCW();
    setDial(randomDial, true);

    // Serial.print(F("Dial commanded CCW to: "));
    // Serial.print(randomDial);
    Serial.print(F("Dial should be at 10, is at: "));
    printEncoderToDial(getEncoderSteps());
    messagePause("Verify then press key to continue");

    randomDial = random(0, 100);
    randomDial = 60;
    turnCW();
    setDial(randomDial, true);

    // Serial.print(F("Dial commanded CW to: "));
    // Serial.print(randomDial);
    Serial.print(F("Dial should be at 60, is at: "));
    printEncoderToDial(getEncoderSteps());
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
    while (!Serial.available()); //Wait for user input
    byte command = Serial.read();

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
      int speed = Serial.parseInt();

      setMotorSpeed(speed);
      Serial.print(F("New motor speed: "));
      Serial.println(speed);
    } 
    else if (command == 'q') {
      Serial.print(F("Enter desired position: "));
      while (!Serial.available()); //Wait for user input

      int position = Serial.parseInt();
      Serial.println(position);
      setDial(position, false);
      Serial.print(F("Encoder position: "));
      printEncoderToDial(getEncoderSteps());
    } 
    else if (command == 'w') {
      Serial.print(F("Enter desired position (with extra spin): "));
      while (!Serial.available()); //Wait for user input

      int position = Serial.parseInt();
      Serial.println(position);
      setDial(position, true);
      Serial.print(F("Encoder position: "));
      printEncoderToDial(getEncoderSteps());
    } 
    else if (command == 'x') {
      setMotorSpeed(0); //Stop motor
      break;
    } 
    else {
      Serial.println(F("Unknown command ¯\\_(ツ)_/¯"));
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

