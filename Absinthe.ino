/*****************************************************************************************
*********************************** Absinthe v0.9.1 **************************************
******************************************************************************************
** Utilizes infrared emitters and detectors to sense its position relative to a course in 
** the form of a dark line placed on the ground.
******************************************************************************************
** Created by Tyler Cook and Clay Sherrock 
** Copyleft 17 October 2013 by InsignificanTech
******************************************************************************************
** Rependencies:
** * Servo.h (Standard Library)
** * MotorShield.h (Library by Aquilleph)
** * LiquidCrystal.h
******************************************************************************************
** DISCLAIMER: Use at your own risk InsignificantTech is not responsible for injuries that 
** may occur during the use of this sketch, nor for ins0mnia or insanity it may allegedly 
** induce.
******************************************************************************************
*/

#include <Servo.h>
#include "MotorShield.h"

#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

#define I2C_ADDR 0x27
#define LIGHT 3
#define EN 2
#define RW 1
#define RS 0
#define D4 4
#define D5 5
#define D6 6
#define D7 7

// LCD Display
LiquidCrystal_I2C lcd(I2C_ADDR, EN, RW, RS, D4, D5, D6, D7);


/* *******************************************
** Constant value definitions ****************
** ******************************************/

// Define dark thresholds for detectors
const int RO_THRESH = 978;
const int RI_THRESH = 977;

const int LO_THRESH = 980;
const int LI_THRESH = 972;

// Define center position of steering servo
const byte CENTER = 30;

// Noise filter weight value (0.0001 to 1)
const float FILTER_VAL = .01;

// Button debounce delay
const long DEBOUNCE_DELAY = 500;

// Operation Modes
const byte     STANDBY_MODE = 0;
const byte CALIBRATION_MODE = 1;
const byte       DRIVE_MODE = 2;
const byte         END_MODE = 3;

// Speed constants
const int BASE_SPEED = 60;

/* 
** *******************************************
** Other globals *****************************
** *******************************************
*/

// Previous smoothed IR reading data 
int ROsmoothData = 0;
int RIsmoothData = 0;
int LOsmoothData = 0;
int LIsmoothData = 0;

long RO_time = 0;
long RI_time = 0;
long LO_time = 0;
long LI_time = 0;

// Current position of servo (initial set to center)
byte position = CENTER;

// Sent from detector to steering control method
byte action = 0;

// Speed toggle button
long lastSpeedPressTime = 0;
byte speedIter = 0;
int  speedSetting = 0;

// Mode toggle button
long lastModePressTime = 0;
byte modeIter = 0;
long lastFlashTime = 0;
int  ledState = LOW;

// Initial mode setting
byte mode = STANDBY_MODE;

/* 
** *******************************************
** Pin number definitions ********************
** *******************************************
*/

// Speed toggle
const byte SPEED_BUTTON = 7;
const byte LED_1 = 5;
const byte LED_2 = 4;

// Detector/Steering toggle
const byte MODE_TOGGLE = 6;
const byte LED_3_RED   = 2;
const byte LED_3_GREEN = 3;

// Analog steering detector input pins
const byte RI_DETECTOR = A3;
const byte RO_DETECTOR = A2;

const byte LI_DETECTOR = A4;
const byte LO_DETECTOR = A5;


// Steering servo
Servo steerServo;
const byte SERVO_PIN = 10;

// Drive Motor
MotorShield motor(byte(1));


void setup() 
{
  // Initialize LCD
  lcd.begin(20, 4, LCD_5x8DOTS);
  lcd.setBacklightPin(LIGHT,POSITIVE);
  lcd.setBacklight(HIGH);

  Serial.begin(9600);
  
  // schtuff
  pinMode(SPEED_BUTTON, INPUT);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  
  pinMode(MODE_TOGGLE, INPUT);
  pinMode(LED_3_RED, OUTPUT);
  pinMode(LED_3_GREEN, OUTPUT);
  
  // Initiate servo on pin 10
  steerServo.attach(SERVO_PIN);
  
  // Set servo position to center.
  center();
 
} // end setup()


/*****************************************************************************************
** Detector Management *******************************************************************
*****************************************************************************************/

/*
** Read Detectors
** gets filtered value from IR detectors
** returns the action to be performed by the driver
*/
byte readDetectors()
{
  ROsmoothData = smooth(analogRead(RO_DETECTOR), ROsmoothData);
  RIsmoothData = smooth(analogRead(RI_DETECTOR), RIsmoothData);

  LIsmoothData = smooth(analogRead(LI_DETECTOR), LIsmoothData);
  LOsmoothData = smooth(analogRead(LO_DETECTOR), LOsmoothData);

  // Get line crossover times
  if (ROsmoothData > RO_THRESH) {
    RO_time = millis();
  }

  if (RIsmoothData > RI_THRESH) {
    RI_time = millis();
  }

  if (LOsmoothData > LO_THRESH) {
    LO_time = millis();
  }

  if (LIsmoothData > LI_THRESH) {
    LI_time = millis();
  }


  if (ROsmoothData > RO_THRESH && LOsmoothData > LO_THRESH) {
    // Terminate
    action = 3;
  } else if (ROsmoothData > RO_THRESH || RO_time > RI_time) {
    // Turn right
    action = 2;
  } else if (LOsmoothData > LO_THRESH || LO_time > LI_time) {
    // Turn left
    action = 1;
  } else {
    // Center Steering
    action = 0;
  } // end if
  
  return action;
} // end readDetectors()

/*
** Smooth
** Filters the noise from IR readings
*/
int smooth(int data, int smoothData)
{
  smoothData = (data * (1 - FILTER_VAL)) + (smoothData * FILTER_VAL);
  return smoothData;
} // end smooth()


/*****************************************************************************************
** Movement Control **********************************************************************
*****************************************************************************************/

/*
** Toggle speed setting
** Select the speed setting
*/
void toggleSpeed()
{
  if (digitalRead(SPEED_BUTTON) == HIGH && millis() > lastSpeedPressTime + DEBOUNCE_DELAY) 
  {
    lastSpeedPressTime = millis();
    
    switch (++speedIter) {
      case 4:
        speedIter = 0;
        
      case 0:
        speedSetting = 0;
        break;
       
      case 1:
        speedSetting = BASE_SPEED;
        break;
        
      case 2:
        speedSetting = (BASE_SPEED + 127) / 2;
        break;
        
      case 3:
        speedSetting = 127;
        break;
    } // end switch
  } // end if
} // end toggleSpeed()


/*
** Toggle mode
** Select mode of operation
*/
void toggleMode()
{
  if (digitalRead(MODE_TOGGLE) == HIGH && millis() > lastModePressTime + DEBOUNCE_DELAY)
  {
    lastModePressTime = millis();
    
    switch (++modeIter)
    {
      case 3:
        modeIter = 0;
        
      case STANDBY_MODE:
        mode = STANDBY_MODE;
        break;
        
      case CALIBRATION_MODE:
        mode = CALIBRATION_MODE;
        break;
        
      case DRIVE_MODE:
        mode = DRIVE_MODE;
        break;
    } // end switch
  } // end if
} // end toggleMode()


/* 
** Steering *********************
** ******************************
*/

/* 
** Right
** Turns steering servo specified angle to the right (default: 55 degrees)
** Input: byte angle: angle to turn steering servo in integer degrees, must be positive
*/
void right(byte angle = 55) 
{
  if (angle < 0) {
    angle = abs(angle);  
  } // end if

  position = angle;
  
  steerServo.write(position);
} // end right()

/* 
** Left
** Turns steering servo specified angle to the left (default: 5 degrees)
** Input: byte angle: angle to turn steering servo in integer degrees, must be positive
*/
void left(byte angle = 5) 
{
  if (angle < 0) {
    angle = abs(angle);  
  } // end if
 
  // Sets the position to new angle 
  position = angle;
  
  steerServo.write(position);
} // end left()

/* 
** Center Steering
** Centers steering servo position
*/
void center() 
{
  position = CENTER;
  steerServo.write(position);
} // end center()

/*
** Steering Control
** Controls the steering stuff
*/
boolean steeringControl(byte value) 
{
  switch (value) {
    case 0:
      center();

      if (mode == DRIVE_MODE) {
          motor.forward(speedSetting);
      }

      break;
      
    case 1:
      if (mode == DRIVE_MODE) {
          motor.forward(speedSetting + 20);
      } 

      left();
      break;
      
    case 2:  
      if (mode == DRIVE_MODE) {
          motor.forward(speedSetting + 20);
      } 

      right();
      break;          
  } // end switch
  
  return (value != 3);
} // end steeringControl()

/*
** Flash
** Flashes given LED with given interval
*/
void flash(byte led, int interval)
{
  if (millis() > interval + lastFlashTime) {
    lastFlashTime = millis();
    
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    } // end if
    
    digitalWrite(led, ledState);
  } // end if
} // end flash()

/*
** Light Control
** Controls the status LEDs
*/
void lightControl() {
  switch (mode) {
    case STANDBY_MODE:


      digitalWrite(LED_3_RED, LOW);
      digitalWrite(LED_3_GREEN, LOW);
      digitalWrite(LED_1, LOW);
      digitalWrite(LED_2, LOW); 
      break;

    case CALIBRATION_MODE:
      lcd.home();
      lcd.print("CALIBRATE");
      
      lcd.setCursor(0,1);
      lcd.print("Left:");
      lcd.setCursor(6,1);
      lcd.print(LOsmoothData);
      lcd.setCursor(11,1);
      lcd.print(LIsmoothData);

      lcd.setCursor(0,2);
      lcd.print("Right:");
      lcd.setCursor(6,2);
      lcd.print(ROsmoothData);
      lcd.setCursor(11,2);
      lcd.print(RIsmoothData);

      flash(LED_3_GREEN, 500); 
      break;

    case DRIVE_MODE:
      digitalWrite(LED_3_GREEN, HIGH);
      break;

    case END_MODE:
      digitalWrite(LED_3_GREEN, LOW); 
      flash(LED_3_RED, 100);
      break;
  } // end mode switch

  switch (speedIter) {
    case 0:
      digitalWrite(LED_1, LOW); 
      digitalWrite(LED_2, LOW); 
      break;

    case 1:
      digitalWrite(LED_1, HIGH); 
      digitalWrite(LED_2, LOW); 
      break;

    case 2: 
      digitalWrite(LED_1, LOW); 
      digitalWrite(LED_2, HIGH);
      break;

    case 3: 
      digitalWrite(LED_1, HIGH); 
      digitalWrite(LED_2, HIGH);
      break; 
  } // end speed switch
} // end lightControl()


/*
** Driver Loop
** does the stuff for the whole thing
*/
void loop() 
{
  lcd.home();
  lcd.print(mode);

  lcd.setCursor(0, 3);
  lcd.print(millis());


  toggleSpeed();
  toggleMode();
  lightControl();

  switch (mode) {

    case STANDBY_MODE:
      motor.brake();
      speedSetting = 0;
      break;

    case CALIBRATION_MODE:
      // if (!steeringControl(readDetectors())) {
      //   Serial.print("REACHED END OF TRACK\n");
      // }
      
      Serial.print("LO: ");
      Serial.print((int) LOsmoothData);
      Serial.print("\tLI: ");
      Serial.print((int) LIsmoothData);

      Serial.print("\t\tRI: ");
      Serial.print((int) RIsmoothData);
      Serial.print("\tRO: ");
      Serial.print((int) ROsmoothData);

      Serial.print("\tACTION: ");
      Serial.println(readDetectors());
      
      delay(300);

      break;

    case DRIVE_MODE:
      motor.forward(speedSetting);
    
      if (!steeringControl(readDetectors())) {
        // When end has been reached
        mode = END_MODE;
      } // end if
      break;
    
    // End Mode
    case END_MODE:
      speedIter = 0;
      speedSetting = 0;
      
      motor.brake();
      center();
      
      ROsmoothData = 0;
      LOsmoothData = 0;
      RIsmoothData = 0;
      LIsmoothData = 0;

      break;
  } // end switch
} // end loop()
