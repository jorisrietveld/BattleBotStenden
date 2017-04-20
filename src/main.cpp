#include "Arduino.h"
#include <SoftwareSerial.h>
#include <NewPing.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <BattleBotDrive.h>

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

#define bluetouthReceivePin A0 // The input pin for receiving bluetouth messages.
#define bluetouthTransmitPin A1 // The output pin for transmitting bluetouth messages.

#define lcdDisplayAddress 0x27 // The address of the LCD display
#define lcdDisplayColumns 16 // The amount of characters each row of the display has.
#define lcdDisplayRows 2 // The amount of rows the display has

#define drivingLowerLimit 45 // The under limit for the motor speed.
#define drivingUpperLimit 255 // The upper limit for the motor speed.

#define defaultDrivingSpeed 60 // The default speed of the cart.

enum{ TRUE = 1, FALSE = 0 }; // Simple enumeration for boolean states.

/**
 * An enumeration with the difrend infrared sensor states.
 */
enum TapeDetected{
  NON_SENSOR, // Sensor state when both infrared sensors don't detect any tape.
  LEFT_SENSOR, // Sensor state when the left infrared sensor detects tape.
  RIGHT_SENSOR, // sensor state when the right infrared sensor detects tape.
  BOTH_SENSOR  // Sensor state when both of the infrared sensors detect tape.
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
long currentDrivingSpeed = 0;
int labyrintSplitCounter = 0;

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
unsigned long previousMillisSendVelocity; // This variable keeps track of the previous velocity data transmission over bluetouth.
unsigned long sendVelocityInterval = 3000; // This variable sets the interval of the velocity data that gets send over bluetouth.

/**
 * The initiation of of objects that are used to communicate with the battle bot's modules.
 */
// Create an new serial communication object for bluetouth communication.
SoftwareSerial bluetouthSerial( bluetouthReceivePin, bluetouthTransmitPin );

// Create an new lcd object for displaying debugging messages on the battle bot.
LiquidCrystal_I2C lcd( lcdDisplayAddress, lcdDisplayColumns, lcdDisplayRows );

// Create an new Ping object for measuring the distance to obstacles.
NewPing sonar( ultraEchoTriggerPin, ultraEchoPin, maxPingDistance );

// Create an new MPU6050 object for using the gyroscope and accelerometer.
MPU6050 accelgyro;

// Create an new BattleBot object for controlling the battle bot.
BattleBotDrive battleBotDrive( leftMotorForwardPin, rightMotorForwardPin, leftMotorBackwardPin, rightMotorBackwardPin );
/**
 * Function to initialize the battleBot.
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
 * This function will overide any bluetouth command so you can invoke any program programmatically.
 */
void overrideCommand( int command, int argument )
{
    incommingBluetouthCommand = command;
    commandInt = command;
    commandString = String(command);
    commandArgument = String(argument);
}

/**
 * Every three seconds send an approximation of the driving speed to the cnc server.
 */
void sendVelocity()
{
    long currentMillis = millis();
    if( currentMillis - previousMillisSendVelocity == sendVelocityInterval )
    {
        String botId = "16";
        String feedback = botId + ((battleBotDrive.getLeftDrivingSpeed()+battleBotDrive.getRightDrivingSpeed()/2)/10);
        previousMillisSendVelocity = currentDrivingSpeed;
    }
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
        // Both infrared sensors detected nothing.
        return NON_SENSOR;
    }
}

/**
 * This function will use the ultra sonar sensor to measure the distance to an object in front of the battle bot.
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
    lcd.setCursor (0, lineIndex);
    for (int i = 0; i < 16; ++i)
    {
        lcd.write(' ');
    }
    lcd.setCursor (0, lineIndex);
}

/**
 * This method will print an message to the first line of the lcd screen if it is the same message it will not update it
 * so that the screen wont flicker when the same message is printed every milli second.
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

/**
 * This function will make the battle bot drive backward and turn in another direction when it finds tape.
 */
void avoidTape()
{
    TapeDetected onSensor = detectTape();

    switch(  onSensor )
    {
        case RIGHT_SENSOR: // When the right sensor detects any tape backup and turnaround.
            updateSecondLCDCommand( "Tape right" );
            battleBotDrive.drive( -defaultDrivingSpeed, -defaultDrivingSpeed );
            delay( 800 );
            battleBotDrive.drive( -defaultDrivingSpeed, defaultDrivingSpeed );
            delay( 800 );
            break;

        case LEFT_SENSOR: // When the left sensor detects any tape backup and turnaround.
            updateSecondLCDCommand( "Tape left" );
            battleBotDrive.drive( -defaultDrivingSpeed, -defaultDrivingSpeed );
            delay( 800 );
            battleBotDrive.drive( defaultDrivingSpeed, -defaultDrivingSpeed );
            delay( 800 );
            break;

        case BOTH_SENSOR:
            updateSecondLCDCommand( "Tape both" ); // When both sensors are detecting tape backup and turnaround.
            battleBotDrive.drive( -defaultDrivingSpeed, -defaultDrivingSpeed );
            delay( 800 );
            battleBotDrive.drive( -defaultDrivingSpeed, defaultDrivingSpeed );
            delay( 400 );
            break;

        case NON_SENSOR: // When no tape is detected just keep moving forward.
            updateSecondLCDCommand( "Keep rolling" );
            battleBotDrive.drive( defaultDrivingSpeed, defaultDrivingSpeed );
            break;

        default:
            break;
    }
}

/**
 * This method will drive the battle bot on an piece of tape on the ground.
 */
void followLineProgram()
{
  TapeDetected onSensor = detectTape();
  long int distanceToObject = sonar.ping_cm();

  if( distanceToObject < 15 && distanceToObject > 0 )
  {
      updateSecondLCDCommand( "End");
      overrideCommand( 15, 0 );
  }

  switch( onSensor )
    {
        case RIGHT_SENSOR:
            updateSecondLCDCommand( "Tape right" );
            battleBotDrive.drive( defaultDrivingSpeed, -defaultDrivingSpeed );
            break;

        case LEFT_SENSOR:
            updateSecondLCDCommand( "Tape left" );
            battleBotDrive.drive( -defaultDrivingSpeed, defaultDrivingSpeed );
            break;

        case BOTH_SENSOR:
            updateSecondLCDCommand( "Tape both" );
            battleBotDrive.drive( -defaultDrivingSpeed, defaultDrivingSpeed );
            break;

        case NON_SENSOR:
            updateSecondLCDCommand( "No tape" );
            battleBotDrive.drive( defaultDrivingSpeed, defaultDrivingSpeed );
            break;

        default:
            break;
    }
}

/**
 * This function will keep the battle bot driving in an area marked by tape on the ground. Meanwhile it will avoid
 * obstacles placed in this area.
 */
void obstacleAvoidanceProgram()
{
    long int distanceToObject = sonar.ping_cm();

    if( distanceToObject < 15 && distanceToObject > 0 )
    {
        updateSecondLCDCommand( "Obstacle");
        // turn around
        battleBotDrive.drive( -defaultDrivingSpeed, -defaultDrivingSpeed);
        delay(600);
        battleBotDrive.drive( -defaultDrivingSpeed, defaultDrivingSpeed );
        delay( 600 );
    }

    avoidTape();
}

/**
 * This function will make the battle bot avoid tape on the ground and smash into every object it finds
 * within 15 centimeters.
 */
void battleProgram()
{
    long int distanceToObject = sonar.ping_cm();

    if( distanceToObject < 15 && distanceToObject > 0 )
    {
        updateSecondLCDCommand( "Attack!!!");
        battleBotDrive.drive( 200, 200);
        delay(5000);
        overrideCommand( 15, 0 );
    }

    avoidTape();
}

void labyrintTurnLeft()
{
    battleBotDrive.drive(-defaultDrivingSpeed, -defaultDrivingSpeed);
    delay(500);
    battleBotDrive.drive(defaultDrivingSpeed, -defaultDrivingSpeed);
    delay(400);
}

void labyrintTurnRight()
{
  battleBotDrive.drive(-defaultDrivingSpeed, -defaultDrivingSpeed);
  delay(500);
  battleBotDrive.drive( -defaultDrivingSpeed, defaultDrivingSpeed );
  delay(400);
}

void labyrintProgram()
{
    TapeDetected onSensor = detectTape();

    switch( onSensor )
      {
          case RIGHT_SENSOR:
              updateSecondLCDCommand( "Tape right" );
              battleBotDrive.drive( -defaultDrivingSpeed, defaultDrivingSpeed );
              break;

          case LEFT_SENSOR:
              updateSecondLCDCommand( "Tape left" );
              battleBotDrive.drive( defaultDrivingSpeed, -defaultDrivingSpeed );
              break;

          case BOTH_SENSOR:
              updateSecondLCDCommand( "Tape both" );
              if( labyrintSplitCounter <= 2)
              {
                  labyrintSplitCounter++;
                  labyrintTurnLeft();
              }
              else if( labyrintSplitCounter > 2)
              {
                  labyrintSplitCounter++;
                  labyrintTurnRight();
              }
              else
              {
                  // error so try to finish it annyway
                  labyrintTurnLeft();
              }
              break;

          case NON_SENSOR:
              updateSecondLCDCommand( "No tape" );
              battleBotDrive.drive( defaultDrivingSpeed-30, defaultDrivingSpeed-30 );
              break;

          default:
              break;
      }
}

/**
 * This function will get called when the start button is pressed.
 */
void nextProgram()
{
    overrideCommand(15, 0);
    // TODO: implement something?
}

/**
 * Function to listen incoming commands from the bluetouth chip.
 */
void listenForBluetouthCommands()
{
    if ( bluetouthSerial.available())
    {
        incommingBluetouthCommand = bluetouthSerial.parseInt();
    }

    commandString = String(incommingBluetouthCommand);
    commandString = commandString.substring( 0, 2 ); // Extract the program command from bluetouth the incoming bluetouth message.
    commandInt = commandString.toInt();
    commandArgument = commandString.substring( 2 ); // Extract the program command argument from the incoming bluetouth message.
}

/**
 * This function will execute the program it has to run. It can be set by bluetooth or by the ovverideCommand() function.
 */
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
            labyrintProgram();
            break;

        case CIRCLE: // Circle button.
            debugMessage = "sumo";
            battleProgram();
            break;

        case SELECT: // Select button.
            clearLcdLine( 1 );
            battleBotDrive.breakCart();
            debugMessage = "Break cart";
            break;

        case START: // Start button.
            clearLcdLine( 1 );
            debugMessage = "Next";
            nextProgram();
            break;

        case ARROW_UP: // Arrow up button.
            clearLcdLine( 1 );
            debugMessage = "Move forward";
            battleBotDrive.drive( defaultDrivingSpeed, defaultDrivingSpeed );
            break;

        case ARROW_DOWN: // Arrow down button.
            clearLcdLine( 1 );
            debugMessage = "Move backward";
            battleBotDrive.drive( -defaultDrivingSpeed, -defaultDrivingSpeed );
            break;

         case ARROW_RIGHT: // Arrow right button.
            clearLcdLine( 1 );
            debugMessage = "Turn right";
            battleBotDrive.drive( defaultDrivingSpeed, -defaultDrivingSpeed);
            break;

        case ARROW_LEFT: // Arrow left button.
            clearLcdLine( 1 );
            debugMessage = "Turn left";
            battleBotDrive.drive( -defaultDrivingSpeed, defaultDrivingSpeed );
            break;

        default:
            // do nothing
            break;
    }

    // Write the current executing command to the LCD screen.
    updateLCDCommand( debugMessage );
}

/**
 * This is the main function of the battle bot it will listen for incoming commands and execute it.
 */
void loop()
{
    sendVelocity();
    listenForBluetouthCommands();
    executeCommand();
}
