
#include <Wire.h>
#include <Zumo32U4.h>
#include <string.h>



// This is the maximum speed the motors will be allowed to turn.
// A maxSpeed of 400 lets the motors go at top speed.  Decrease
// this value to impose a speed limit.
// slow and slowturn are the tested ones that we know work.
// fast and fastturn are the untested ones we dont know if work,
// but we want to make work as we want to be as fast as possible.
int fast = 300;
//int fastturn = fast*0.2;
//int slow = 75;
//int slowturn = 1100*2.25;
const uint16_t maxSpeed = fast;
//int turn = fastturn;
int neutralSteering = 200;

Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4Motors motors;
Zumo32U4IMU imu;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;

// Change next line to this if you are using the older Zumo 32U4
// with a black and green LCD display:
// Zumo32U4LCD display;
Zumo32U4LCD display;

bool useEmitters = true;

int16_t lastError = 0;

int buttonPressed = 0;

int16_t leftSpeed = 0;
int16_t rightSpeed = 0;

bool found = false;
// left is 1500, right is 500, center is 1000
bool turned = false;
int check = 0;
bool beep = false;
//bool waitForCan = false;


// sets up the front proximity sensors
int frontLeftProx = 0;
int lightLevels[] = {0, 1, 2, 3, 4, 5, 6};

// variable for gyro turning later
int turnAngleprop = 0;
int Gyro = 0;
//double gyroOffset = 0;
int calibrationReadings = 1024;

// file containing most of the gyro funtuions
#include "TurnSensor.h"

// setting up line sensors
// we are using 5 line sensors
#define NUM_SENSORS 5
uint16_t lineSensorValues[5] = { 0, 0, 0, 0, 0};
uint16_t threshhold[5] = { 250, 250, 250, 250, 250 };
int16_t position = lineSensors.readLine(lineSensorValues);



// Sets up special characters for the display so that we can show
// bar graphs.
void loadCustomCharacters()
{
  static const char levels[] PROGMEM = {
    0, 0, 0, 0, 0, 0, 0, 63, 63, 63, 63, 63, 63, 63
  };
  display.loadCustomCharacter(levels + 0, 0);  // 1 bar
  display.loadCustomCharacter(levels + 1, 1);  // 2 bars
  display.loadCustomCharacter(levels + 2, 2);  // 3 bars
  display.loadCustomCharacter(levels + 3, 3);  // 4 bars
  display.loadCustomCharacter(levels + 4, 4);  // 5 bars
  display.loadCustomCharacter(levels + 5, 5);  // 6 bars
  display.loadCustomCharacter(levels + 6, 6);  // 7 bars
}

void printBar(uint8_t height)
{
  if (height > 8) { height = 8; }
  const char barChars[] = {' ', 0, 1, 2, 3, 4, 5, 6, (char)255};
  display.print(barChars[height]);
}

// function to update the gyroscope and convert the data it gets to an angle in degrees
void updateGyro()
{
  turnSensorUpdate();
  turnAngleprop = ((((int32_t)turnAngle >> 16) * 360) >> 16);
  Serial.println("turnAngle:");
  Serial.println(turnAngleprop);
}

// updates the proximity sensor
void updateProxSensors()
{
  proxSensors.read();
  frontLeftProx = proxSensors.countsFrontWithLeftLeds();
}

// calibarates the gyroscope and line sensors
void calibrateSensors()
{
  display.clear();
  buzzer.play(">c32>>g32");
  // Wait 1 second and then begin automatic sensor calibration
  // firstly calibrating the gyroscope
  // then rotating in place to sweep the line sensors sensors over the line
  delay(1000);
  turnSensorSetup();
  delay(500);
  turnSensorReset();
  delay(500);
  turnSensorUpdate();
  proxSensors.setBrightnessLevels(lightLevels, sizeof(lightLevels)/2);
  proxSensors.initThreeSensors();

  // turn while calibrating so the line sensors can differentiate between line and non line
  for(uint16_t i = 0; i < 120; i++)
  {
    if (i > 30 and i <= 90)
    {
      motors.setSpeeds(-100, 100);
    }
    else
    {
      motors.setSpeeds(100, -100);
    }
    updateGyro();
    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}

// Shows a bar graph of sensor readings on the display.
// Returns after the user presses A.
void showReadings()
{
  display.clear();

  while(!buttonA.getSingleDebouncedPress())
  {
    lineSensors.readCalibrated(lineSensorValues);
    updateGyro();

    display.gotoXY(0, 0);
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
      uint8_t barHeight = map(lineSensorValues[i], 0, 1000, 0, 8);
      printBar(barHeight);
    }
    //printserial();
    updateGyro();

    }
  Serial.println("Button Pressed:");
  Serial.println(buttonPressed);
}

void printserial()
{
    char buffer[80];
    sprintf(buffer, "%4d %4d %4d %4d %4d %c\n",
      lineSensorValues[0],
      lineSensorValues[1],
      lineSensorValues[2],
      lineSensorValues[3],
      lineSensorValues[4],
      useEmitters ? 'E' : 'e'
    );
    Serial.print(buffer);
}

void setup()
{
  // setting up all the sensors on the Zumo 38U4
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();
  display.clear();
  lineSensors.initFiveSensors();

  loadCustomCharacters();

  // Play a little welcome song
  buzzer.play(">b32>>g32>>c32>>e32>>g32>>c32");


  // Wait for button A to be pressed and released.
  display.clear();
  display.print(F("Press A"));
  display.gotoXY(0, 1);
  display.print(F("to calib"));
  buttonA.waitForButton();

  calibrateSensors();

  
  showReadings();
  

  // Play music and wait for it to finish before we start driving.
  display.clear();
  display.print(F("Go!"));
  //buzzer.play("L16 cdegreg4");
  buzzer.play(">c32>>g32>>e32>>c32>>g32>>b32");
  while(buzzer.isPlaying());
}

void waitForCan()
{
  // turn on the emitter to activate the conveyor
  lineSensors.emittersOn();
  // does nothing but update the fron sensors until it detects something in front of it
  while(frontLeftProx == 0)
  {
    display.clear();
    display.print(frontLeftProx);
    updateProxSensors();
    updateGyro();
  }
  // waits a bit to allow the can to get into the center of the can
  // that will make it easier to push if it's the one further away
  delay(250);
  // stops the conveyor
  lineSensors.emittersOff();
  updateProxSensors();
  display.clear();
  display.print(frontLeftProx);
  delay(500);
  // if the can is far away, it will drive foward to push it off
  if (frontLeftProx < 5)
  {
    // drives a bit foward with delay
    // this ensures that it will not detect the line it was already on and stop prematurely
    motors.setSpeeds(maxSpeed*0.5, maxSpeed*0.5);
    updateGyro();
    delay(250);
    position = lineSensors.readLine(lineSensorValues);
    //drive foward until finding a line, that is the edge
    while(lineSensorValues[2] >= threshhold[2])
    {
      position = lineSensors.readLine(lineSensorValues);
      updateGyro();
      motors.setSpeeds(maxSpeed*0.5, maxSpeed*0.5);
    }
    updateProxSensors();
    turned == false;
    returnLine1();

  }
  // if the can is close it will turn right, drive forward, turn left, drive a bit forward 
  // then turn left again and drive foward until it sees a line
  else if (frontLeftProx >= 5)
  {
    // turn 90 degrees to the right
    turnAngleprop = 0;
    updateGyro();
    position = lineSensors.readLine(lineSensorValues);
    //can is close
    newAngle = -90;
    turnSensorReset();
    turnAngleprop = 0;
    while (turnAngleprop != newAngle)
    {
      // subtract the current angle from the angle you want to be at
      // the constant outside the () is to increase the speed 
      updateGyro();
      int32_t turnSpeed = (newAngle-turnAngleprop)*20;
      turnSpeed = constrain(turnSpeed, -maxSpeed, maxSpeed);
      motors.setSpeeds(-turnSpeed, turnSpeed);
    }
    // drive forward until it reaches a bit beyond the conveyer
    motors.setSpeeds(maxSpeed*0.5, maxSpeed*0.5);
    delay(1250);
    // turn 90 degrees to the left
    newAngle = 90;
    turnSensorReset();
    turnAngleprop = 0;
    while (turnAngleprop != newAngle)
    {
      updateGyro();
      int32_t turnSpeed = (newAngle-turnAngleprop)*20;
      turnSpeed = constrain(turnSpeed, -maxSpeed, maxSpeed);
      motors.setSpeeds(-turnSpeed, turnSpeed);
    }
    // drive a bit forward onto the conveyor
    motors.setSpeeds(maxSpeed*0.5, maxSpeed*0.5);
    delay(625);
    // turn 90 degrees to the left
    newAngle = 90;
    turnSensorReset();
    turnAngleprop = 0;
    while (turnAngleprop != newAngle)
    {
      updateGyro();
      int32_t turnSpeed = (newAngle-turnAngleprop)*20;
      turnSpeed = constrain(turnSpeed, -maxSpeed, maxSpeed);
      motors.setSpeeds(-turnSpeed, turnSpeed);
    }
    // drive forward until it detects a line
    position = lineSensors.readLine(lineSensorValues);
    while(lineSensorValues[2] >= threshhold[2])
    {
      position = lineSensors.readLine(lineSensorValues);
      updateGyro();
      motors.setSpeeds(maxSpeed*0.5, maxSpeed*0.5);
    }
    updateProxSensors();
    turnAngleprop = 0;
    updateGyro();
    turned == false;
    // return from pushing close can
    returnLine2();
  }

}

// stops the Zumo
void stop()
{
    display.clear();
    buzzer.play(">a32>>g32");
    motors.setSpeeds(0, 0);
    delay(25);
    turnSensorUpdate();
    updateProxSensors();
    // drive a bit back to ensure it alligns with the emitter and reciever
    motors.setSpeeds(-50,-50);
    delay(200);
    // noise to indicate that it has fully stopped
    display.clear();
    display.print(F("STOP!"));
    // play sound to indicate it stopped
    buzzer.play(">a32>>g32>>c32");
    beep = true;

    motors.setSpeeds(0, 0);
    turnSensorUpdate();
    updateGyro();
    // Go to waiting for can phase
    waitForCan();

    
}


// after starting, it will move forward until it find the line
// after it finds the line, it will correct itself and stop when it reaces the end of the T
void findline()
{
  updateGyro();
  int16_t position = lineSensors.readLine(lineSensorValues);
  Serial.println("position:");
  Serial.println(position);
  // Our "error" is how far we are away from the center of the
  // line, which corresponds to position 2000.
  int16_t error = position - 2000;
  Serial.println("error:");
  Serial.println(error);
  // calculate the differense in motorspeed necesarry to correct towards the line
  int16_t speedDifference = error / 4 + 6 * (error - lastError);
  lastError = error;
  int16_t leftSpeed = (int16_t)maxSpeed - speedDifference;
  int16_t rightSpeed = (int16_t)maxSpeed + speedDifference;
  leftSpeed = constrain(leftSpeed, 0, (int16_t)maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, (int16_t)maxSpeed);
  
  // if it finds the line it drives a bit foward to make it easir to line up with the line
  if (check >= 1 and found == false)
    {
      display.clear();
      display.print(F("Found line!"));
      motors.setSpeeds(maxSpeed, maxSpeed);
      found = true;
      buzzer.play(">g32>>c32");
      updateGyro();
      delay(50);
    }
    //stops the zumo if both the edge sensors are detecting a line but the center sensor isn't
    else if (lineSensorValues[0] <= threshhold[0] and lineSensorValues[4] <= threshhold[4] and lineSensorValues[2] >= threshhold[2])
    {
      stop();
    }

    //If the zumo loses the line it was following, it will turn until the center sensor finds a line again.
    else if (check == 0 and found == true)
    {
      display.clear();
      display.print(F("Lost line!"));
      while (lineSensorValues[2] >= threshhold[2])
      {
        updateGyro();
        motors.setSpeeds(neutralSteering, -neutralSteering);
        lineSensors.readLine(lineSensorValues);
      }
      neutralSteering = 200;
    }

    // Drive forward
    else
    {
      updateGyro();
      motors.setSpeeds(leftSpeed*0.75, rightSpeed*0.75);
      display.clear();
      display.print(F("Go!"));
      //buzzer.play(">g32>>c32");
    }
}


// reverses long enough to end up a good bit behind the T line
void returnLine1()
{
  motors.setSpeeds(-200, -200);
  delay(1100);
}


void returnLine2()
{
  position = lineSensors.readLine(lineSensorValues);
  Serial.println("position:");
  Serial.println(position);
  //turn 90 to the left, drive a bit foward then turn to the left again at 90
  turnAngleprop = 0;
  updateGyro();
  if (!turned)
  {
    turned = true;
    newAngle = 90;
    turnSensorReset();
    turnAngleprop = 0;
    while (turnAngleprop != newAngle)
    {
      updateGyro();
      // subtract the current angle from the angle you want to be at
      // the constant outside the () is to increase the speed 
      int32_t turnSpeed = (newAngle-turnAngleprop)*20;
      turnSpeed = constrain(turnSpeed, -maxSpeed, maxSpeed);
      motors.setSpeeds(-turnSpeed, turnSpeed);
    }
    motors.setSpeeds(neutralSteering, neutralSteering);
    delay(700);
    newAngle = 90;
    turnSensorReset();
    turnAngleprop = 0;
    while (turnAngleprop != newAngle)
    {
      updateGyro();
      int32_t turnSpeed = (newAngle-turnAngleprop)*20;
      turnSpeed = constrain(turnSpeed, -maxSpeed, maxSpeed);
      motors.setSpeeds(-turnSpeed, turnSpeed);
    }

    //reverse directions so that later in findline() so it turns to the left instead of right
    neutralSteering = -neutralSteering;
    // resets the check as if it starts this on a line, then without this it will save it for later and instantly trigger the "lost line" functiunality in findline()
    check = 0;
  }
  found = false;
  turned = false;
  findline();
}


void loop()
{
  updateGyro();
  //starts by going through all sensors and checking they detect something and if any of them do, then it found the line
  check = 0;
  for (uint8_t i = 0;  i < NUM_SENSORS; i++)
  {
    if( lineSensorValues[i] < threshhold[i] ) {check = check+1;}
  }
  // runs the starting part of the program
  findline();
}
