#include "Arduino.h"
#include <SoftwareSerial.h>
#include <NewPing.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <I2Cdev.h>
#include <MPU6050.h>

/**
 * Define the I/O pins that are connected to the robot.
 */
#define leftMotorForwardPin 3 // Output pin that is connected to the left motor for forward movement.
#define leftMotorBackwardPin 2 // Output pin that is connected to the left motor for backward movement.
#define rightMotorForwardPin 9 // Output pin that is connected to the right motor for forward movement.
#define rightMotorBackwardPin 4 // Output pin that is connected to the right motor for backward movement.

#define leftInfraredSensor 10 // Input pin that is connected to the left infrared sensor.
#define rightInfraredSensor 11 // Input pin that is connected to the right infrared sensor.

#define ultraEchoPin 12 // Input pin that is connected to the ultra echo sensor.
#define ultraEchoTriggerPin 13 // Input pin that is connected to the trigger from the ultra echo sensor.

#define drivingLowerLimit 45 // The under limit for the motor speed.
#define drivingUpperLimit 255 // The upper limit for the motor speed.

enum{ TRUE = 1, FALSE = 0 }; // Simple enumeration for boolean states.

/**
 * An enumeration with the difrent infrared sensor states.
 */
enum TapeDetected{
  NON_SENSOR, // Sensor state when both infrared sensors don't detect any tape.
  LEFT_SENSOR, // Sensor state when the left infrared sensor detects tape.
  RIGHT_SENSOR, // sensor state when the right infrared sensor detects tape.
  BOTH_SENSOR  // Sensor state when both of the infrared sennsors detect tape.
};

/**
 * An enumeration with the symbols of the buttons on the arduino app.
 */
enum Button{
  SQUARE = 11, // This button is for the follow line challenge.
  TRIANGLE = 12, // This button is for the obstacle avoidance challenge.
  CROSS = 13, // This button is for the labyrinth challenge
  CIRCLE = 14, // This button is for the sumo battle challenge.
  SELECT = 15, // This button will break all programs on the battle bot.
  START = 16, // This button will select the next challenge.
  ARROW_UP = 17, // This button will move the battle bot forward.
  ARROW_DOWN = 18, // This button will move the battle bot backward.
  ARROW_RIGHT = 19, // This button will move the battle bot right.
  ARROW_LEFT = 20 // This button will move the battle bot left.
};

/**
 * Declaration of variables that store the commands given to the battle bot.
 */
int incommingBluetouthCommand = 0; // Varialbe that stores the last bluetouth message.
int commandInt = 0;
String commandString = ""; // Variable that stores the program command extracted from the bluetouth message.
String commandArgument = ""; // Variable that stores the program command argument extrated from the bluetouth message.

/**
 * Declaration of variables that store the messages that will get displayed on the LCD screen.
 */
String lcdDisplayText = ""; // Variable used to store the last text that was printed on the first line lcd screen.
String secondLcdDisplayText = ""; // Variable used to store the last text that was printed on the second line of the lcd screen.
String debugMessage = ""; // Variable that stores the last available debug message.

/**
 * Declaration of variables that are needed to initate objects.
 */
int maxPingDistance = 200; // The maximum distance the ultra echo sensor measures.

unsigned long previousMillisSendVelocity; // This variable keeps track of the prevrious velocity data transmision over bluetouth.
unsigned long previousMillisVelocityMessure; // This variable keeps track of the previous velocity messurment.

unsigned long sendVelocityInterval = 1000; // This variable sets the interval of the velocity data that gets send over bluetouth.
unsigned long velocityMessureInterval = 100; // This variable sets the interval of velocity messurments.

unsigned int messurmentCounter = 0; // This variable counts the amount of messurments taken per 100 milliseconds.

/**
 * The initation of of objects that are used to communicate with the battle bot's modules.
 */
SoftwareSerial bluetouthSerial(A0, A1); // Create an new serial communication object for bluetouth communication.
LiquidCrystal_I2C lcd(0x27, 16, 2); // Create an new lcd object for displaying debugging messages on the battle bot.
NewPing sonar(ultraEchoTriggerPin, ultraEchoPin, maxPingDistance ); // Create an new Ping object for measuring the distance to obstacles.
MPU6050 accelgyro; // Create an new MPU6050 object for using the gyroscope and accelerometer.

/**
 * Function to initialize the battlebot.
 */
void setup()
{
    // Initialize the I/O pins
    pinMode( leftMotorBackwardPin, OUTPUT);
    pinMode( leftMotorForwardPin, OUTPUT );
    pinMode( rightMotorBackwardPin, OUTPUT );
    pinMode( rightMotorForwardPin, OUTPUT );

    pinMode( leftInfraredSensor, INPUT );
    pinMode( rightInfraredSensor, INPUT );

    pinMode( ultraEchoTriggerPin, INPUT );
    pinMode( ultraEchoPin, INPUT );

    // Start Serial communication.
    Serial.begin(38400);
    bluetouthSerial.begin(38400);

    // Start the gyroscope and accelerometer.
    Wire.begin();
    accelgyro.initialize();
    accelgyro.setXAccelOffset(-5233);
    accelgyro.setYAccelOffset(-2320);
    accelgyro.setZAccelOffset(1465);
    accelgyro.setXGyroOffset(126);
    accelgyro.setYGyroOffset(28);
    accelgyro.setZGyroOffset(42);

    // Start the LCD screen.
    lcd.begin();
    lcd.backlight();
    lcd.print("Awaiting...");
}

/**
 * This function will take 10 velocity mesurments every second.
 */
void mesureVelocity(  )
{
    unsigned long currentMillis = millis();

    if( currentMillis - previousMillisVelocityMessure == sendVelocityInterval )
    {
        previousMillisVelocityMessure = currentMillis;
        // TODO finish the messurment
    }
}

void sendVelocity()
{
    // Formula (V = Vo + at)
     unsigned long currentMillis = millis();

    if( currentMillis - previousMillisSendVelocity == sendVelocityInterval )
    {
        previousMillisSendVelocity = currentMillis;
        // TODO finish the messurment
    }
}

/**
 * This function will stop all power on the left motor.
 */
void breakLeft( )
{
    digitalWrite( leftMotorBackwardPin, LOW );
    digitalWrite( leftMotorForwardPin, LOW );
}

/**
 * This function will stop all power on the right motor.
 */
void breakRight()
{
    digitalWrite( rightMotorBackwardPin, LOW );
    digitalWrite( rightMotorForwardPin, LOW );
}

/**
 * This method breaks the battle bot.
 */
void breakCart()
{
    breakLeft();
    breakRight();
}

/**
 * This function drives controlles the left motor. You can pass it an value between -200 for moving backward
 * and +200 for moving forward.
 *
 * @param speed
 */
void driveRightMotor( int speed )
{
    breakRight(); // Reset the current motor state.
    if( speed < 0 )
    {
        speed = abs( speed );
        speed += drivingLowerLimit;
        speed = ( speed > drivingUpperLimit ) ? drivingUpperLimit : speed;
        digitalWrite( rightMotorBackwardPin, HIGH ); // set the backward I/O pin to high
        analogWrite( rightMotorForwardPin, 255 - speed ); // limit the backward speed.
    }
    else if( speed == 0 )
    {
        breakRight();
    }
    else
    {
        speed = abs( speed );
        speed += drivingLowerLimit;
        speed = ( speed > drivingUpperLimit ) ? drivingUpperLimit : speed;
        analogWrite( rightMotorForwardPin, speed );
    }
}

/**
 * This function drives controlles the left motor. You can pass it an value between -200 for moving backward
 * and +200 for moving forward.
 *
 * @param speed
 */
void driveLeftMotor( int speed )
{
    breakLeft(); // Reset the current motor state.
    if( speed < 0 )
    {
        speed = abs( speed );
        speed = ( speed > 200 ) ? 200 : speed;
        speed += drivingLowerLimit;
        digitalWrite( leftMotorBackwardPin, HIGH ); // set the backward I/O pin to high
        analogWrite( leftMotorForwardPin, 255 - speed ); // limit the backward speed.
    }
    else if( speed == 0 )
    {
        breakLeft();
    }
    else
    {
        speed = abs( speed );
        speed = ( speed > 200 ) ? 200 : speed;
        speed += drivingLowerLimit;
        analogWrite( leftMotorForwardPin, speed );
    }
}

/**
 * This function drives the BattleBot.
 *
 * @param left range -200 for 255 backward and +200 for 255 forward
 * @param right
 */
void drive( int leftMotorSpeed, int rightMotorSpeed )
{
    driveRightMotor( rightMotorSpeed );
    driveLeftMotor( leftMotorSpeed );
}

/**
 * This method detects if the infrared sensors moved over some tape.
 */
TapeDetected detectTape()
{
    if( digitalRead( leftInfraredSensor ) == HIGH && digitalRead( rightInfraredSensor ) == HIGH )
    {
        // Both infrared sensors detect tape.
        return BOTH_SENSOR;
    }
    else if( digitalRead( leftInfraredSensor ) == HIGH )
    {
        // The left infrared sensor detected tape.
        return LEFT_SENSOR;
    }
    else if( digitalRead( rightInfraredSensor) == HIGH )
    {
        // The right infrared sensor detected tape.
        return RIGHT_SENSOR;
    }
    else
    {
        // Both infrared sensors detectet nothing.
        return NON_SENSOR;
    }
}

/**
 * This function will use the utra sonar sensor to mesure the distance to an object in front of the battle bot.
 */
int detectObstacle()
{
    return sonar.ping_cm();
}

/**
 * This function will clear an line on the LCD screen by its line index number.
 *
 * @param the index of the line so on an 2X16 display either 0 for line 1 or 1 for line 2.
 */
void clearLcdLine( int lineIndex )
{
    // Clear the whole of line 1
    lcd.setCursor (0, lineIndex);
    for (int i = 0; i < 16; ++i)
    {
      lcd.write(' ');
    }
    // Now write a message to line 1
    lcd.setCursor (0, 1);
}

/**
 * This method will print an message to the first line of the lcd screen if it is the same message it will not update it
 * so that the screen wont flikker when the same message is printed every milli second.
 *
 * @param currentCommand The last printed message to the lcd screen.
 */
void updateLCDCommand( String currentCommand )
{
    if( lcdDisplayText != currentCommand )
    {
        clearLcdLine( 0 );
        lcd.setCursor( 0, 0 );
        lcd.print( currentCommand );
        lcdDisplayText = currentCommand;
        Serial.println( currentCommand );
    }
}

/**
 * This method will print an message to the second line of the lcd screen if it is the same message it will not update it
 * so that the screen wont flikker when the same message is printed every milli second.
 *
 * @param currentCommand The last printed message to the lcd screen.
 */
void updateSecondLCDCommand( String secondCurrentCommand )
{
    if( secondLcdDisplayText != secondCurrentCommand )
    {
        clearLcdLine( 1 );
        lcd.setCursor( 0, 1 );
        lcd.print( secondCurrentCommand );
        secondLcdDisplayText = secondCurrentCommand;
        Serial.println( secondCurrentCommand );
    }
}

void overrideCommand( int command, int argument )
{
    incommingBluetouthCommand = command; // Varialbe that stores the last bluetouth message.
    commandInt = command;
    commandString = String(command); // Variable that stores the program command extracted from the bluetouth message.
    commandArgument = String(argument); // Variable that stores the program command argument extrated from the bluetouth message.
}

/**
 * This method will drive the battle bot on an pice of tape on the ground.
 */
void followLineProgram()
{
  TapeDetected onSensor = detectTape();

  switch( onSensor )
    {
        case RIGHT_SENSOR:
            updateSecondLCDCommand( "Tape right" );
            drive( 20, -1 );
            break;

        case LEFT_SENSOR:
            updateSecondLCDCommand( "Tape left" );
            drive( -1, 20 );
            break;

        case BOTH_SENSOR:
            updateSecondLCDCommand( "Tape both" );
            drive( 20, 20 );
            break;

        case NON_SENSOR:
            updateSecondLCDCommand( "No tape" );
            drive( 20, 20 );
            break;

        default:
            break;
    }
}

/**
 * This function will keep the battle bot driving in an area marked by tape on the ground. Meanwile it will avoid
 * obstacles placed in this area.
 */
void obstacleAvoidanceProgram()
{
    TapeDetected onSensor = detectTape();
    long int distanceToObject = sonar.ping_cm();

    if( distanceToObject < 15 && distanceToObject > 0 )
    {
        updateSecondLCDCommand( "Obstacle");
        // turn around
        drive( -10, -10);
        delay(400);
        drive( -10, 10 );
        delay( 400 );
    }


    switch(  onSensor )
    {
        case RIGHT_SENSOR:
            updateSecondLCDCommand( "Tape right" );
            drive( -10, -10 );
            delay( 800 );
            drive( -15, 10 );
            delay( 400 );
            break;

        case LEFT_SENSOR:
            updateSecondLCDCommand( "Tape left" );
            drive( -10, -10 );
            delay( 800 );
            drive( 10, -15 );
            delay( 400 );
            break;

        case BOTH_SENSOR:
            updateSecondLCDCommand( "Tape both" );
            drive( -10, -10 );
            delay( 800 );
            drive( -10, 10 );
            delay( 400 );
            break;

        case NON_SENSOR:
            updateSecondLCDCommand( "Keep roling" );
            drive( 10, 10);
            break;

        default:
            break;
    }
}

void battleProgram()
{
    TapeDetected onSensor = detectTape();
    long int distanceToObject = sonar.ping_cm();

    if( distanceToObject < 15 && distanceToObject > 0 )
    {
        updateSecondLCDCommand( "Attack!!!");
        // turn around
        drive( 200, 200);
        delay(500);
        overrideCommand( 15, 0 );
    }


    switch(  onSensor )
    {
        case RIGHT_SENSOR:
            updateSecondLCDCommand( "Tape right" );
            drive( -10, -10 );
            delay( 800 );
            drive( -15, 10 );
            delay( 400 );
            break;

        case LEFT_SENSOR:
            updateSecondLCDCommand( "Tape left" );
            drive( -10, -10 );
            delay( 800 );
            drive( 10, -15 );
            delay( 400 );
            break;

        case BOTH_SENSOR:
            updateSecondLCDCommand( "Tape both" );
            drive( -10, -10 );
            delay( 800 );
            drive( -10, 10 );
            delay( 400 );
            break;

        case NON_SENSOR:
            updateSecondLCDCommand( "Keep roling" );
            drive( 10, 10);
            break;

        default:
            break;
    }
}

/**
 * Function to listen incomming commands from the bluetouth chip.
 */
void listenForBluetouthCommands()
{
    if ( bluetouthSerial.available())
    {
        incommingBluetouthCommand = bluetouthSerial.parseInt();
    }

    commandString = String(incommingBluetouthCommand);
    commandString = commandString.substring( 0, 2 ); // Exstract the program command from bluetouth the incomming bluetouth message.
    commandInt = commandString.toInt();
    commandArgument = commandString.substring( 2 ); // Exstract the program command argument from the incomming bluetouth message.
}

//Hier worden eventuele functies, middels inkomende data, aangeroepen.
void executeCommand()
{
    Button pressed = (Button)commandInt;

    switch( pressed )
    {
        case SQUARE:// Square button.
            debugMessage = "follow line";
            followLineProgram();
            break;

        case TRIANGLE: // Triangle button.
            debugMessage = "obstacle avoidance";
            obstacleAvoidanceProgram();
            break;

        case CROSS: // Cross button.
            debugMessage = "labyrint";
            break;

        case CIRCLE: // Circle button.
            debugMessage = "sumo";
            battleProgram();
            break;

        case SELECT: // Select button.
            clearLcdLine( 1 );
            breakCart();
            debugMessage = "Break cart";
            break;

        case START: // Start button.
            clearLcdLine( 1 );
            debugMessage = "Next";
            break;

        case ARROW_UP: // Arrow up button.
            clearLcdLine( 1 );
            debugMessage = "Move forward";
            drive( 20, 20 );
            break;

        case ARROW_DOWN: // Arrow down button.
            clearLcdLine( 1 );
            debugMessage = "Move backward";
            drive( -20, -20 );
            break;

         case ARROW_RIGHT: // Arrow right button.
            clearLcdLine( 1 );
            debugMessage = "Turn right";
            drive( 20, 0);
            break;

        case ARROW_LEFT: // Arrow left button.
            clearLcdLine( 1 );
            debugMessage = "Turn left";
            drive( 0, 20 );
            break;

        default:
            // do nothing
            break;
    }

    // Write the current executing command to the LCD screen.
    updateLCDCommand( debugMessage );
}

/**
 * This is the main function of the battle bot it will listen for incomming commands and execute it.
 */
void loop()
{
    listenForBluetouthCommands();
    executeCommand();
}
