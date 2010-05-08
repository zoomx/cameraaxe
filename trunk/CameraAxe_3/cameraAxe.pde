#include <SoftwareSerial.h>
#include <EEPROM.h>

// Maurice Ribble 
// 4-1-2010
// http://www.glacialwanderer.com/hobbyrobotics
// Open Source, licensed under a Creative Commons Attribution 3.0 License (http://creativecommons.org/licenses/by-sa/3.0/)
// Compiled with Arduino Software 0018 (http://arduino.cc)

// This code is the firmware for the CameraAxe3 which is a flash/camera trigger tool.  This hardware contains the following:
// A) 6 buttons to control a menu
// B) 1 Serial LCD to display a menu
// C) 2 3.5 mm jack for driving flashes or cameras (called devices in code)
// D) 2 3.5 mm jack to analog sensors (power to these ports can be enabled/disable via a transistor - over 100mA supported)

// REVISIONS:
// 3002 (12-02-2009) - Remove timelapse mode and add projectile mode
// 3003 (4-01-2010)  - Added new valve sensor mode
//                   - Added timelapse mode back from version 3000
//                   - Fixed bug with EEPROM_DEVICE1_TYPE and EEPROM_DEVICE2_TYPE values from eeprom (Andy Modla)
// 3004 (4-13-2010)  - Added control for low/high trigger signal on projectile sensor

// Define digital/analog pins
#define BUTTON_MENU_PIN      2
#define BUTTON_SET_PIN       3
#define BUTTON_UP_PIN        4
#define BUTTON_DOWN_PIN      5
#define BUTTON_LEFT_PIN      6
#define BUTTON_RIGHT_PIN     7
#define LCD_TX_PIN           8
//#define NOT_USED           9
#define DEVICE1_SHUTTER_PIN  10
#define DEVICE1_FOCUS_PIN    11
#define DEVICE2_SHUTTER_PIN  12
#define DEVICE2_FOCUS_PIN    13
//#define NOT_USED           0
//#define NOT_USED           1
#define SENSOR1_APIN         2
#define SENSOR1_PIN          16
#define SENSOR1_POWER_APIN   17
#define SENSOR2_APIN         4
#define SENSOR2_PIN          18
#define SENSOR2_POWER_APIN   19

// LCD defines (Assumes you're using LCD NHD-0216K3Z-FL-GBW)
#define LCD_OP_CODE              0xFE
#define LCD_SET_CURSOR_POSITION  0x45
#define LCD_CLEAR_SCREEN         0x51
#define LCD_CURSOR_UNDERLINE     0x47
#define LCD_CURSOR_UNDERLINE_OFF 0x48
#define LCD_BRIGHTNESS           0x53

// Memory offsets into EEPROM
#define EEPROM_NONE                          -1
#define EEPROM_DEVICE1_SENSOR                 0
#define EEPROM_DEVICE1_TYPE                   1
#define EEPROM_DEVICE1_DELAY_MS               2
#define EEPROM_DEVICE1_CYCLE_SEC              3
#define EEPROM_DEVICE2_SENSOR                 4
#define EEPROM_DEVICE2_TYPE                   5
#define EEPROM_DEVICE2_DELAY_MS               6
#define EEPROM_DEVICE2_CYCLE_SEC              7
#define EEPROM_SENSOR1_TRIGGER_LOW_HIGH       8
#define EEPROM_SENSOR1_TRIGGER_VAL            9
#define EEPROM_SENSOR1_POWER                  10
#define EEPROM_SENSOR2_TRIGGER_LOW_HIGH       11
#define EEPROM_SENSOR2_TRIGGER_VAL            12
#define EEPROM_SENSOR2_POWER                  13
#define EEPROM_PROJECTILE_SENSOR_DISTANCE     14
#define EEPROM_PROJECTILE_SENSOR_LOW_HIGH     15
#define EEPROM_VALVE_DROP1_SIZE               16
#define EEPROM_VALVE_DROP2_DELAY              17
#define EEPROM_VALVE_DROP2_SIZE               18
#define EEPROM_VALVE_PHOTO_DELAY              19
#define EEPROM_TIMELAPSE_SECONDS              20
#define EEPROM_TIMELAPSE_MINUTES              21
#define EEPROM_TIMELAPSE_HOURS                22

#define EEPROM_SIZE                           23 

// Special EEPROM values that don't need to be managage like those above
//  Make sure these happen after EEPROM_SIZE
#define EEPROM_LCD_BRIGHTNESS                 28
#define EEPROM_INCH_CM                        29

enum { START_POS_BEGINNING, START_POS_MIDDLE, START_POS_END };                                   // lcdSetNumber start position
enum { SENSOR_NONE, SENSOR_MIN, SENSOR_MAX };                                                    // Sensor Averaging Method
enum { BUTTON_S_MENU, BUTTON_S_SET, BUTTON_S_UP, BUTTON_S_DOWN, BUTTON_S_LEFT, BUTTON_S_RIGHT }; // Button state keys
enum { BUTTON_PRESSED=0, BUTTON_NOT_PRESSED=1 };                                                 // Valuse of pressed vs not pressed buttons
enum { MENUMODE_MENU, MENUMODE_TRIGGER };                                                        // Menu mode (either in menus or running triggers)
enum { MENU_DEVICE1, MENU_DEVICE2, MENU_SENSOR1, MENU_SENSOR2, MENU_PROJECTILE,
       MENU_VALVE, MENU_TIMELAPSE, MENU_SIZE };                                                  // g_menu values for tracking current menu
enum { DEVICE_MAP_SENSOR1, DEVICE_MAP_SENSOR2, DEVICE_MAP_NONE };                                // Remaps sensors to trigger devices
enum { DEVICE_TYPE_FLASH, DEVICE_TYPE_CAMERA_FAST, DERVICE_TYPE_CAMERA_LOW_POW };                // Type of device (camera or flash)
enum { SENSOR_POWER_ON, SENSOR_POWER_OFF_DEVICE1, SENSOR_POWER_OFF_DEVICE2 };                    // Tells when to send power to sensors
enum { UNIT_INCH, UNIT_CM };                                                                     // Using inches or cm for distance units

// GLOBALS
SoftwareSerial g_lcdSerial   = SoftwareSerial(LCD_TX_PIN, LCD_TX_PIN);  // Software serial port used to control the LCD
volatile int   g_menuMode    = MENUMODE_MENU;                           // Either menu or trigger mode
int            g_menu        = MENU_DEVICE1;                            // Menu
int            g_subMenu     = 0;                                       // Submenu
int            g_eepromShadow[EEPROM_SIZE];                             // Shadow copy of values stored in flash memory
int            g_inchCm      = UNIT_INCH;                               // Tracks using inches or cm

void setup()
{
  // Setup input/output on all pins being used as digital in/outs
  pinMode(BUTTON_MENU_PIN,     INPUT);
  pinMode(BUTTON_SET_PIN,      INPUT);
  pinMode(BUTTON_UP_PIN,       INPUT);
  pinMode(BUTTON_DOWN_PIN,     INPUT);
  pinMode(BUTTON_LEFT_PIN,     INPUT);
  pinMode(BUTTON_RIGHT_PIN,    INPUT);
  pinMode(LCD_TX_PIN,          OUTPUT);
  pinMode(DEVICE1_SHUTTER_PIN, OUTPUT);
  pinMode(DEVICE1_FOCUS_PIN,   OUTPUT);
  pinMode(DEVICE2_SHUTTER_PIN, OUTPUT);
  pinMode(DEVICE2_FOCUS_PIN,   OUTPUT);
  pinMode(SENSOR1_POWER_APIN,  OUTPUT);
  pinMode(SENSOR2_POWER_APIN,  OUTPUT);

  Serial.begin(9600); // open hw serial for debugging
  g_lcdSerial.begin(9600);
  delay(100);                  // Let LCD screen boot
  lcdOpCode(LCD_CLEAR_SCREEN);
  lcdSetPosition(0, 0);
  lcdOpCode(LCD_CURSOR_UNDERLINE_OFF);
  int brightness = 1;
  
  g_inchCm = UNIT_INCH;

  // If pressing menu while booting then reset to factory defaults
  if (digitalRead(BUTTON_MENU_PIN) == BUTTON_PRESSED && 
      digitalRead(BUTTON_SET_PIN) == BUTTON_PRESSED && 
      digitalRead(BUTTON_RIGHT_PIN) == BUTTON_PRESSED)
  {
    eepromWriteInt(EEPROM_DEVICE1_SENSOR,               0);
    eepromWriteInt(EEPROM_DEVICE1_TYPE,                 0);
    eepromWriteInt(EEPROM_DEVICE1_DELAY_MS,             0);
    eepromWriteInt(EEPROM_DEVICE1_CYCLE_SEC,            5);
    eepromWriteInt(EEPROM_DEVICE2_SENSOR,               2);
    eepromWriteInt(EEPROM_DEVICE2_TYPE,                 0);
    eepromWriteInt(EEPROM_DEVICE2_DELAY_MS,             0);
    eepromWriteInt(EEPROM_DEVICE2_CYCLE_SEC,            5);
    eepromWriteInt(EEPROM_SENSOR1_TRIGGER_LOW_HIGH,     1);
    eepromWriteInt(EEPROM_SENSOR1_TRIGGER_VAL,          500);
    eepromWriteInt(EEPROM_SENSOR1_POWER,                0);
    eepromWriteInt(EEPROM_SENSOR2_TRIGGER_LOW_HIGH,     1);
    eepromWriteInt(EEPROM_SENSOR2_TRIGGER_VAL,          500);
    eepromWriteInt(EEPROM_SENSOR2_POWER,                0);
    eepromWriteInt(EEPROM_PROJECTILE_SENSOR_DISTANCE,   20);
    eepromWriteInt(EEPROM_PROJECTILE_SENSOR_LOW_HIGH,   1);
    eepromWriteInt(EEPROM_VALVE_DROP1_SIZE,             80);
    eepromWriteInt(EEPROM_VALVE_DROP2_DELAY,            0);
    eepromWriteInt(EEPROM_VALVE_DROP2_SIZE,             0);
    eepromWriteInt(EEPROM_VALVE_PHOTO_DELAY,            200);
    eepromWriteInt(EEPROM_TIMELAPSE_SECONDS,            30);
    eepromWriteInt(EEPROM_TIMELAPSE_MINUTES,            0);
    eepromWriteInt(EEPROM_TIMELAPSE_HOURS,              0);
    eepromWriteInt(EEPROM_LCD_BRIGHTNESS,               1);
    eepromWriteInt(EEPROM_INCH_CM,                      UNIT_INCH);
    
    lcdOpCode(LCD_CLEAR_SCREEN);
    g_lcdSerial.print("  U/D Brightness");
    lcdSetPosition(0,1);
    g_lcdSerial.print("3004    L/R Inch");
    waitTillAllButtonsReleased();

    lcdSetBrightness(brightness);

    do
    {
      if (digitalRead(BUTTON_UP_PIN)  == BUTTON_PRESSED)
      {
        brightness = min(brightness+1, 8);
        lcdSetBrightness(brightness);
        eepromWriteInt(EEPROM_LCD_BRIGHTNESS, brightness);
        waitTillAllButtonsReleased();
      }
      if (digitalRead(BUTTON_DOWN_PIN)  == BUTTON_PRESSED)
      {
        brightness = max(brightness-1, 1);
        lcdSetBrightness(brightness);
        eepromWriteInt(EEPROM_LCD_BRIGHTNESS, brightness);
        waitTillAllButtonsReleased();
      }
      if ((digitalRead(BUTTON_LEFT_PIN)  == BUTTON_PRESSED) || (digitalRead(BUTTON_RIGHT_PIN) == BUTTON_PRESSED))
      {
        if (g_inchCm == UNIT_INCH)
        {
          g_inchCm = UNIT_CM;
          eepromWriteInt(EEPROM_INCH_CM, g_inchCm);
          lcdSetPosition(12,1);
          g_lcdSerial.print("Cm  ");
          waitTillAllButtonsReleased();
        }
        else
        {
          g_inchCm = UNIT_INCH;
          eepromWriteInt(EEPROM_INCH_CM, g_inchCm);
          lcdSetPosition(12,1);
          g_lcdSerial.print("Inch");
          waitTillAllButtonsReleased();
        }
      }
    }
    while((digitalRead(BUTTON_MENU_PIN)  == BUTTON_NOT_PRESSED) &&
          (digitalRead(BUTTON_SET_PIN)   == BUTTON_NOT_PRESSED));
  }

  // Load all the setting saved in eeprom
  g_eepromShadow[EEPROM_DEVICE1_SENSOR]               = eepromReadInt(EEPROM_DEVICE1_SENSOR, 0, 2);
  g_eepromShadow[EEPROM_DEVICE1_TYPE]                 = eepromReadInt(EEPROM_DEVICE1_TYPE, 0, 2);
  g_eepromShadow[EEPROM_DEVICE1_DELAY_MS]             = eepromReadInt(EEPROM_DEVICE1_DELAY_MS, 0, 999);
  g_eepromShadow[EEPROM_DEVICE1_CYCLE_SEC]            = eepromReadInt(EEPROM_DEVICE1_CYCLE_SEC, 0, 999);
  g_eepromShadow[EEPROM_DEVICE2_SENSOR]               = eepromReadInt(EEPROM_DEVICE2_SENSOR, 0, 2);
  g_eepromShadow[EEPROM_DEVICE2_TYPE]                 = eepromReadInt(EEPROM_DEVICE2_TYPE, 0, 2);
  g_eepromShadow[EEPROM_DEVICE2_DELAY_MS]             = eepromReadInt(EEPROM_DEVICE2_DELAY_MS, 0, 999);
  g_eepromShadow[EEPROM_DEVICE2_CYCLE_SEC]            = eepromReadInt(EEPROM_DEVICE2_CYCLE_SEC, 0, 999);
  g_eepromShadow[EEPROM_SENSOR1_TRIGGER_LOW_HIGH]     = eepromReadInt(EEPROM_SENSOR1_TRIGGER_LOW_HIGH, 0, 1);
  g_eepromShadow[EEPROM_SENSOR1_TRIGGER_VAL]          = eepromReadInt(EEPROM_SENSOR1_TRIGGER_VAL, 0, 999);
  g_eepromShadow[EEPROM_SENSOR1_POWER]                = eepromReadInt(EEPROM_SENSOR1_POWER, 0, 2);
  g_eepromShadow[EEPROM_SENSOR2_TRIGGER_LOW_HIGH]     = eepromReadInt(EEPROM_SENSOR2_TRIGGER_LOW_HIGH, 0, 1);
  g_eepromShadow[EEPROM_SENSOR2_TRIGGER_VAL]          = eepromReadInt(EEPROM_SENSOR2_TRIGGER_VAL, 0, 999);
  g_eepromShadow[EEPROM_SENSOR2_POWER]                = eepromReadInt(EEPROM_SENSOR2_POWER, 0, 2);
  g_eepromShadow[EEPROM_PROJECTILE_SENSOR_DISTANCE]   = eepromReadInt(EEPROM_PROJECTILE_SENSOR_DISTANCE, 0, 999);
  g_eepromShadow[EEPROM_PROJECTILE_SENSOR_LOW_HIGH]   = eepromReadInt(EEPROM_PROJECTILE_SENSOR_LOW_HIGH, 0, 1);
  g_eepromShadow[EEPROM_VALVE_DROP1_SIZE]             = eepromReadInt(EEPROM_VALVE_DROP1_SIZE, 0, 999);
  g_eepromShadow[EEPROM_VALVE_DROP2_DELAY]            = eepromReadInt(EEPROM_VALVE_DROP2_DELAY, 0, 999);
  g_eepromShadow[EEPROM_VALVE_DROP2_SIZE]             = eepromReadInt(EEPROM_VALVE_DROP2_SIZE, 0, 999);
  g_eepromShadow[EEPROM_VALVE_PHOTO_DELAY]            = eepromReadInt(EEPROM_VALVE_PHOTO_DELAY, 0, 999);
  g_eepromShadow[EEPROM_TIMELAPSE_SECONDS]            = eepromReadInt(EEPROM_TIMELAPSE_SECONDS, 0, 59);
  g_eepromShadow[EEPROM_TIMELAPSE_MINUTES]            = eepromReadInt(EEPROM_TIMELAPSE_MINUTES, 0, 59);
  g_eepromShadow[EEPROM_TIMELAPSE_HOURS]              = eepromReadInt(EEPROM_TIMELAPSE_HOURS, 0, 999);
  brightness                                          = eepromReadInt(EEPROM_LCD_BRIGHTNESS, 1, 8);
  g_inchCm                                            = eepromReadInt(EEPROM_INCH_CM, UNIT_INCH, UNIT_CM);

  lcdSetBrightness(brightness);

  // Default values  
  digitalWrite(DEVICE1_SHUTTER_PIN, LOW);
  digitalWrite(DEVICE1_FOCUS_PIN, LOW);
  digitalWrite(DEVICE2_SHUTTER_PIN, LOW);
  digitalWrite(DEVICE2_FOCUS_PIN, LOW);
  digitalWrite(SENSOR1_POWER_APIN, HIGH);
  digitalWrite(SENSOR2_POWER_APIN, HIGH);
}

void loop()
{
  if (g_menuMode == MENUMODE_MENU)
  {
    int button;
    
    // This is where the main menu action happens
    if (g_menu == MENU_DEVICE1)
      button = deviceSetupMenu("Camera/Flash1", EEPROM_DEVICE1_TYPE, EEPROM_DEVICE1_SENSOR, EEPROM_DEVICE1_DELAY_MS, EEPROM_DEVICE1_CYCLE_SEC);
    else if (g_menu == MENU_DEVICE2)
      button = deviceSetupMenu("Camera/Flash2", EEPROM_DEVICE2_TYPE, EEPROM_DEVICE2_SENSOR, EEPROM_DEVICE2_DELAY_MS, EEPROM_DEVICE2_CYCLE_SEC);
    else if (g_menu == MENU_SENSOR1)
      button = sensorSetupMenu("Sensor1", SENSOR1_APIN, EEPROM_SENSOR1_TRIGGER_LOW_HIGH, EEPROM_SENSOR1_TRIGGER_VAL, EEPROM_SENSOR1_POWER);
    else if (g_menu == MENU_SENSOR2)
      button = sensorSetupMenu("Sensor2", SENSOR2_APIN, EEPROM_SENSOR2_TRIGGER_LOW_HIGH, EEPROM_SENSOR2_TRIGGER_VAL, EEPROM_SENSOR2_POWER);
    else if (g_menu == MENU_PROJECTILE)
      button = projectileSetupMenu();
    else if (g_menu == MENU_VALVE)
      button = valveSetupMenu();
    else if (g_menu == MENU_TIMELAPSE)
      button = timelapseSetupMenu();
    
    if (button == BUTTON_S_SET)
    {
      g_menuMode = MENUMODE_TRIGGER;    // This takes us from menu mode to trigger mode
    }
    else if (button == BUTTON_S_MENU)
    {
      g_subMenu = 0;                    // Reset the submenu to first screen to avoid confusion
      g_menu = (g_menu + 1)%MENU_SIZE;  // If we do past the end of the menus then loop back to the first menu
    }
  }
  else  // MENUMODE_TRIGGER
  {
    if (g_menu == MENU_PROJECTILE)
    {
      projectileFunc();
    }
    else if (g_menu == MENU_VALVE)
    {
      valveFunc();
    }
    else if (g_menu == MENU_TIMELAPSE)
    {
      timelapseFunc();
    }
    else  // MENU_DEVICE1, MENU_DEVICE2, MENU_SENSOR1, MENU_SENSOR2
    {
      sensorFunc();
    }
    waitTillAllButtonsReleased();
  }
}

// Handles the projectile triggering
// This mode uses a special sensor that has two gate triggers that go high when triggered.  The gates are
// seperated by SENSOR_DISTANCE units.  This mode uses digital signals instead of analog to improve
// speed (analog reads on ATMEL 168 take 100 micro seconds).  The time taken to trigger these sensors lets
// me calculate the velocity of the projectile.  Then assuming constant velocity (ie this doesn't work for
// objects being acclerated by gravity) it can caculate how long to wait until impact with the target (user
// provided distance in inches or cm).
void projectileFunc()
{
  int i;
  int targetDistance = g_eepromShadow[EEPROM_PROJECTILE_SENSOR_DISTANCE];
  int lowHigh = g_eepromShadow[EEPROM_PROJECTILE_SENSOR_LOW_HIGH];
  unsigned long int distanceBetweenSensors;
  
  if (lowHigh == 0)
  {
    lowHigh = LOW;
  }
  else
  {
    lowHigh = HIGH;
  }
  
  if (g_inchCm == UNIT_INCH)
  {
    distanceBetweenSensors = 200;  // Distance between my sensors is 2.00 inches
  }
  else
  {
    distanceBetweenSensors = 508;  // Distance between my sensors is 2.00 inches (or 5.08 cm)
  }

  lcdOpCode(LCD_CLEAR_SCREEN);
  g_lcdSerial.print("Projectile Press");
  lcdSetPosition(0, 1);
  g_lcdSerial.print("    Menu to exit");
  
  while(g_menuMode == MENUMODE_TRIGGER)
  {
    unsigned long startTime;
    unsigned long endTime;
    unsigned long impactTime;

    if(digitalRead(BUTTON_MENU_PIN) == BUTTON_PRESSED)  // Exit if Menu button is pressed
    {
       g_menuMode = MENUMODE_MENU;
       break;
    }
      
    if (digitalRead(SENSOR1_PIN) == lowHigh)  // If sensor1 detects projectile
    {
      startTime =  micros();
      endTime = micros();
      while(digitalRead(SENSOR2_PIN) != lowHigh)  // Look for sensor2 to detect projectile
      {
        endTime = micros();
        if (endTime - startTime > 1000000)  // If we have waited 1 second and there has not been a detection projectile must have missed second sensor
        {
          lcdSetPosition(0, 1);
          g_lcdSerial.print("Second trig fail");
          for(i=0; i<2000; ++i)  // Display message for awhile
          {
            delay(1);
            if (digitalRead(BUTTON_MENU_PIN) == BUTTON_PRESSED)
              break;
          }
          lcdSetPosition(0, 1);
          g_lcdSerial.print("    Menu to exit");
          endTime = 0;
          break;
        }
      }
      
      if (endTime)
      {
        unsigned long int elapsedTime             = (endTime - startTime) ? (endTime - startTime) : 1;
        unsigned long int hundredthInchCmPerSec   = distanceBetweenSensors*1000000/elapsedTime;
        unsigned long int impactTime              = 100*1000000/hundredthInchCmPerSec*targetDistance + endTime;  // If changing be careful about overflowing int32
        unsigned long int curTime;
        
        while (micros() < impactTime)  // Wait for impact
        {}
                
        //trigger flash
        digitalWrite(DEVICE1_FOCUS_PIN, HIGH);
        digitalWrite(DEVICE1_SHUTTER_PIN, HIGH);
        digitalWrite(DEVICE2_FOCUS_PIN, HIGH);
        digitalWrite(DEVICE2_SHUTTER_PIN, HIGH);
        
        // Display how fast the projectile was moving
        lcdSetPosition(0, 1);
        if((hundredthInchCmPerSec < 5*12*100) || g_inchCm != UNIT_INCH)
        {
          if (g_inchCm == UNIT_INCH)
          {
            g_lcdSerial.print("Inch/sec:       ");
          }
          else
          {
            g_lcdSerial.print("Cm/sec:         ");
          }
          lcdSetPosition(10, 1);
          g_lcdSerial.print(hundredthInchCmPerSec/100);
        }
        else
        {
          g_lcdSerial.print("Feet/Sec:       ");
          lcdSetPosition(10, 1);
          g_lcdSerial.print(hundredthInchCmPerSec/100/12);
        }
        for(i=0; i<2000; ++i)  // Display message for awhile
        {
          delay(1);
          if (digitalRead(BUTTON_MENU_PIN) == BUTTON_PRESSED)
            break;
        }
        lcdSetPosition(0, 1);
        g_lcdSerial.print("    Menu to exit");
        // Turn off flash
        digitalWrite(DEVICE1_FOCUS_PIN, LOW);
        digitalWrite(DEVICE1_SHUTTER_PIN, LOW);
        digitalWrite(DEVICE2_FOCUS_PIN, LOW);
        digitalWrite(DEVICE2_SHUTTER_PIN, LOW);
      }
    }
  }
  digitalWrite(DEVICE1_FOCUS_PIN, LOW);
  digitalWrite(DEVICE1_SHUTTER_PIN, LOW);
  digitalWrite(DEVICE2_FOCUS_PIN, LOW);
  digitalWrite(DEVICE2_SHUTTER_PIN, LOW);
}

// Handles the Timelapse countdown and triggering
// This sensor is times a solenoid valve opening to create one or two drops and take a picture.
// Device 1 is optionally attached to your camera (alternately you can trigger the camera manually with a long exposure)
// Here is the normal sequence 
//   Drop 1 is created
//   Wait and then create Drop 2
//   Wait and then trigger device 2 (flash)
void valveFunc()
{
  lcdOpCode(LCD_CLEAR_SCREEN);
  g_lcdSerial.print("Valve Sensor");

  pinMode(SENSOR1_PIN,  OUTPUT);
  pinMode(SENSOR2_PIN,  OUTPUT);
  digitalWrite(SENSOR1_PIN, LOW);
  digitalWrite(SENSOR2_PIN, LOW);

  // Trigger camera
  digitalWrite(DEVICE1_FOCUS_PIN, HIGH);
  digitalWrite(DEVICE1_SHUTTER_PIN, HIGH);
  delay(200);
  digitalWrite(DEVICE1_FOCUS_PIN, LOW);
  digitalWrite(DEVICE1_SHUTTER_PIN, LOW);

  // Create drop 1
  digitalWrite(SENSOR1_PIN, HIGH);
  digitalWrite(SENSOR2_PIN, HIGH);
  delay(g_eepromShadow[EEPROM_VALVE_DROP1_SIZE]);
  digitalWrite(SENSOR1_PIN, LOW);
  digitalWrite(SENSOR2_PIN, LOW);

  // Create drop 2
  delay(g_eepromShadow[EEPROM_VALVE_DROP2_DELAY]);
  digitalWrite(SENSOR1_PIN, HIGH);
  digitalWrite(SENSOR2_PIN, HIGH);
  delay(g_eepromShadow[EEPROM_VALVE_DROP2_SIZE]);
  digitalWrite(SENSOR1_PIN, LOW);
  digitalWrite(SENSOR2_PIN, LOW);
  delay(g_eepromShadow[EEPROM_VALVE_PHOTO_DELAY]);

  // Trigger flash
  digitalWrite(DEVICE2_FOCUS_PIN, HIGH);
  digitalWrite(DEVICE2_SHUTTER_PIN, HIGH);
  delay(200);
  digitalWrite(DEVICE2_FOCUS_PIN, LOW);
  digitalWrite(DEVICE2_SHUTTER_PIN, LOW);

  pinMode(SENSOR1_PIN,  INPUT);
  pinMode(SENSOR2_PIN,  INPUT);
  g_menuMode = MENUMODE_MENU;  // Always return to menu in this mode
}

// Handles timelapse triggering
void timelapseFunc()
{
  int i;
  int hours = g_eepromShadow[EEPROM_TIMELAPSE_HOURS];
  int minutes = g_eepromShadow[EEPROM_TIMELAPSE_MINUTES];
  int seconds = g_eepromShadow[EEPROM_TIMELAPSE_SECONDS];
  int forceHalfSec = 0;
  
  lcdOpCode(LCD_CLEAR_SCREEN);
  g_lcdSerial.print("Timelapse   Menu");
  lcdSetPosition(0, 1);
  g_lcdSerial.print("           Exits");
  
  while(g_menuMode == MENUMODE_TRIGGER)
  {
    lcdPrintWithZeros(0, 1, 3, hours);
    g_lcdSerial.print(":");
    lcdPrintWithZeros(4, 1, 2, minutes);
    g_lcdSerial.print(":");
    lcdPrintWithZeros(7, 1, 2, seconds);
  
    if (forceHalfSec)
    {
      for(i=0; i<50; ++i)
      {
        delay(10);
        if(digitalRead(BUTTON_MENU_PIN) == BUTTON_PRESSED)
        {
          g_menuMode = MENUMODE_MENU;
          break;
        }
      }

      forceHalfSec = 0;
    }
    else
    {
      for(i=0; i<100; ++i)
      {
        delay(10);
        if(digitalRead(BUTTON_MENU_PIN) == BUTTON_PRESSED)
        {
          g_menuMode = MENUMODE_MENU;
          break;
        }
      }
    }
    --seconds;
    
    if ((seconds == 0) && (minutes == 0) && (hours == 0) && (g_menu != MENUMODE_MENU))
    {
      seconds = g_eepromShadow[EEPROM_TIMELAPSE_SECONDS];
      minutes = g_eepromShadow[EEPROM_TIMELAPSE_MINUTES];
      hours = g_eepromShadow[EEPROM_TIMELAPSE_HOURS];
  
      //trigger camera
      digitalWrite(DEVICE1_FOCUS_PIN, HIGH);
      digitalWrite(DEVICE1_SHUTTER_PIN, HIGH);
      digitalWrite(DEVICE2_FOCUS_PIN, HIGH);
      digitalWrite(DEVICE2_SHUTTER_PIN, HIGH);
      for(i=0; i<100; ++i)
      {
        delay(10);
        if(digitalRead(BUTTON_MENU_PIN) == BUTTON_PRESSED)
        {
          g_menuMode = MENUMODE_MENU;
          break;
        }
      }
      digitalWrite(DEVICE1_SHUTTER_PIN, LOW);
      digitalWrite(DEVICE1_FOCUS_PIN, LOW);
      digitalWrite(DEVICE2_SHUTTER_PIN, LOW);
      digitalWrite(DEVICE2_FOCUS_PIN, LOW);
      forceHalfSec = 1;
    }
    else if ((seconds == -1) && (minutes == 0))
    {
      seconds = 59;
      minutes = 59;
      --hours;
    }
    else if (seconds == -1)
    {
      seconds = 59;
      -- minutes;
    }
  
    if (digitalRead(BUTTON_MENU_PIN) == BUTTON_PRESSED)
      g_menuMode = MENUMODE_MENU;
  }
  
  digitalWrite(DEVICE1_FOCUS_PIN, LOW);
  digitalWrite(DEVICE1_SHUTTER_PIN, LOW);
  digitalWrite(DEVICE2_FOCUS_PIN, LOW);
  digitalWrite(DEVICE2_SHUTTER_PIN, LOW);
}

// Handles all the sensor triggering
// This handles 2 sensors (can be any sensor you make) and 2 trigger devices (camera or flash).
// Since each device can be triggered by any sensor this function acts as a crossbar
// between sensors and devices.
void sensorFunc()
{
  int i;

  // These arrays are for device info
  int deviceDelayMs[2]     = {g_eepromShadow[EEPROM_DEVICE1_DELAY_MS], g_eepromShadow[EEPROM_DEVICE2_DELAY_MS]};
  int deviceCycleSec[2]    = {g_eepromShadow[EEPROM_DEVICE1_CYCLE_SEC], g_eepromShadow[EEPROM_DEVICE2_CYCLE_SEC]};
  int deviceToSensorMap[2] = {g_eepromShadow[EEPROM_DEVICE1_SENSOR], g_eepromShadow[EEPROM_DEVICE2_SENSOR]};
  int deviceType[2]        = {g_eepromShadow[EEPROM_DEVICE1_TYPE], g_eepromShadow[EEPROM_DEVICE2_TYPE]};
  int deviceShutterPins[2] = {DEVICE1_SHUTTER_PIN, DEVICE2_SHUTTER_PIN};
  int deviceFocusPins[2]   = {DEVICE1_FOCUS_PIN, DEVICE2_FOCUS_PIN};

  unsigned long deviceStartTrigTimeMs[2]   = {0,0};
  unsigned long deviceEndTrigTimeMs[2]     = {0,0};
  
  // These arrays are for sensor info
  int sensorTriggerLowHigh[2]    = {g_eepromShadow[EEPROM_SENSOR1_TRIGGER_LOW_HIGH], g_eepromShadow[EEPROM_SENSOR2_TRIGGER_LOW_HIGH]};
  int sensorTriggerVals[2]       = {g_eepromShadow[EEPROM_SENSOR1_TRIGGER_VAL], g_eepromShadow[EEPROM_SENSOR2_TRIGGER_VAL]};
  int sensorPower[2]             = {g_eepromShadow[EEPROM_SENSOR1_POWER], g_eepromShadow[EEPROM_SENSOR2_POWER]};
  int sensorPins[2]              = {SENSOR1_APIN, SENSOR2_APIN};
  int sensorPowerPins[2]         = {SENSOR1_POWER_APIN, SENSOR2_POWER_APIN};

  lcdOpCode(LCD_CLEAR_SCREEN);
  
  if (deviceType[0] == DEVICE_TYPE_FLASH)
    g_lcdSerial.print("Flash1  ");
  else
    g_lcdSerial.print("Camera1 ");
  if (deviceType[1] == DEVICE_TYPE_FLASH)
    g_lcdSerial.print(" Flash2 ");
  else
    g_lcdSerial.print(" Camera2");

  lcdSetPosition(0, 1);
  
  for(i=0; i<2; ++i)
  {
    if (i == 0)
    {
      if(deviceToSensorMap[i] == DEVICE_MAP_SENSOR1)
        g_lcdSerial.print("Sensor1 ");
      else if(deviceToSensorMap[i] == DEVICE_MAP_SENSOR2)
        g_lcdSerial.print("Sensor2 ");
      else //DEVICE_MAP_NONE
        g_lcdSerial.print("None    ");
    }
    else
    {
      if(deviceToSensorMap[i] == DEVICE_MAP_SENSOR1)
        g_lcdSerial.print(" Sensor1");
      else if(deviceToSensorMap[i] == DEVICE_MAP_SENSOR2)
        g_lcdSerial.print(" Sensor2");
      else //DEVICE_MAP_NONE
        g_lcdSerial.print(" None   ");
    }
      
    if (deviceType[i] == DEVICE_TYPE_CAMERA_FAST)
      digitalWrite(deviceFocusPins[i], HIGH);
  }

  // Now that everything is setup this does the actual triggering
  while(g_menuMode == MENUMODE_TRIGGER)
  {
    for(i=0; i<2; ++i)  // Loop through the 2 devices
    {
      int remap = deviceToSensorMap[i];  // Remaps sensor 1 or 2 to the camera/flash device
      int val;
      
      // All sensor arrays should use remap index.  All device arrays should us i index.
      
      if(remap != DEVICE_MAP_NONE)
      {
        unsigned long curTime = millis();
        val = analogRead(sensorPins[remap]);
  
        if(deviceEndTrigTimeMs[i] != 0)
        {
          // This does the actual triggering
          if (deviceStartTrigTimeMs[i] && (curTime > deviceStartTrigTimeMs[i])) // Starts trigger
          {
            deviceStartTrigTimeMs[i] = 0;
            digitalWrite(deviceShutterPins[i], HIGH);      // Trigger device
            if (deviceType[i] != DEVICE_TYPE_CAMERA_FAST)
              digitalWrite(deviceFocusPins[i], HIGH);      // Trigger device focus if we didn't do early focus
            if (sensorPower[0]-1 == remap)
              digitalWrite(sensorPowerPins[0], LOW);      // Turn off power to sensor
            if (sensorPower[1]-1 == remap)
              digitalWrite(sensorPowerPins[1], LOW);      // Turn off power to sensor

            if ((deviceToSensorMap[0] == DEVICE_MAP_NONE) || (deviceToSensorMap[1] == DEVICE_MAP_NONE))
            {
              lcdSetPosition(15, 1);
              g_lcdSerial.print("*");
            }
            
            // Track each device seperately (nice, but can serial port can cause multiple millisecond lag between triggers
            /*if (i == 0)
            {
              lcdSetPosition(0, 0);
              g_lcdSerial.print("*");
            }
            if (i == 1)
            {
              lcdSetPosition(9, 0);
              g_lcdSerial.print("*");
            }*/
            

          }
          if (curTime > deviceEndTrigTimeMs[i]) // Ends trigger
          {
            deviceEndTrigTimeMs[i] = 0;
            digitalWrite(deviceShutterPins[i], LOW);
            if (deviceType[i] != DEVICE_TYPE_CAMERA_FAST)
              digitalWrite(deviceFocusPins[i], LOW);
            if (sensorPower[0]-1 == remap)
              digitalWrite(sensorPowerPins[0], HIGH);
            if (sensorPower[1]-1 == remap)
              digitalWrite(sensorPowerPins[1], HIGH);

            if ((deviceToSensorMap[0] == DEVICE_MAP_NONE) || (deviceToSensorMap[1] == DEVICE_MAP_NONE))
            {
              lcdSetPosition(15, 1);
              g_lcdSerial.print(" ");
            }
            
            // Track each device seperately (nice, but can serial port can cause multiple millisecond lag between triggers
            /*if (i == 0)
            {
              lcdSetPosition(0, 0);
              if (deviceType[0] == DEVICE_TYPE_FLASH)
                g_lcdSerial.print("F");
              else
                g_lcdSerial.print("C");
            }
            if (i == 1)
            {
              lcdSetPosition(9, 0);
              if (deviceType[1] == DEVICE_TYPE_FLASH)
                g_lcdSerial.print("F");
              else
                g_lcdSerial.print("C");
            }
            */
          }
        }
        else
        {
          // This sets up the trigger times
          if(sensorTriggerLowHigh[remap] == 0)
          {
            if(val <= sensorTriggerVals[remap])
            {
              deviceStartTrigTimeMs[i] = curTime + deviceDelayMs[i];
              deviceEndTrigTimeMs[i]   = deviceStartTrigTimeMs[i] + deviceCycleSec[i]*1000;
            }
          }
          else
          {
            if(val >= sensorTriggerVals[remap])
            {
              deviceStartTrigTimeMs[i] = curTime + deviceDelayMs[i];
              deviceEndTrigTimeMs[i]   = deviceStartTrigTimeMs[i] + deviceCycleSec[i]*1000;
            }
          }
        }
      }
    }
    
    if(digitalRead(BUTTON_MENU_PIN) == BUTTON_PRESSED)
      g_menuMode = MENUMODE_MENU;
  }

  // Default values  
  digitalWrite(DEVICE1_SHUTTER_PIN, LOW);
  digitalWrite(DEVICE1_FOCUS_PIN, LOW);
  digitalWrite(DEVICE2_SHUTTER_PIN, LOW);
  digitalWrite(DEVICE2_FOCUS_PIN, LOW);
  digitalWrite(SENSOR1_POWER_APIN, HIGH);
  digitalWrite(SENSOR2_POWER_APIN, HIGH);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// MENUS
////////////////////////////////////////////////////////////////////////////////////////////////////

// This is the menu that lets you link a device trigger to a sensor for triggering
int deviceSetupMenu(char *deviceName, int eepromDeviceType, int eepromDeviceSensor, int eepromSensorDelay, int eepromSensorCycle)
{
  byte delayMs[3];
  byte cycleSec[3];
  char *deviceTypeStr[3]  = { "Flash     ", "Fast_Cam  ", "LowPow_Cam"};
  char *deviceSensorStr[4] = { "Sensor1", "Sensor2", "None   " };
  int subMenuSize = 4;
  int button;

  intToNumArray(g_eepromShadow[eepromSensorDelay], delayMs, 3);
  intToNumArray(g_eepromShadow[eepromSensorCycle], cycleSec, 3);

  lcdOpCode(LCD_CLEAR_SCREEN);
  g_lcdSerial.print(deviceName);
  lcdSetPosition(0, 1);

  if (g_subMenu == 0)
  {
    g_lcdSerial.print("Type:");
    button = lcdSetString(6, 1, 3, &g_eepromShadow[eepromDeviceType], deviceTypeStr);
    eepromWriteInt(eepromDeviceType, g_eepromShadow[eepromDeviceType]);
  }
  else if (g_subMenu == 1)
  {
    g_lcdSerial.print("Trig by:");
    button = lcdSetString(9, 1, 3, &g_eepromShadow[eepromDeviceSensor], deviceSensorStr);
    eepromWriteInt(eepromDeviceSensor, g_eepromShadow[eepromDeviceSensor]);
  }
  else if (g_subMenu == 2)
  {
    g_lcdSerial.print("Delay (ms):");
    button = lcdSetNumber(13, 1, START_POS_MIDDLE, 3, delayMs, -1, SENSOR_NONE, 9);
    g_eepromShadow[eepromSensorDelay] = numArrayToInt(delayMs, 3);
    eepromWriteInt(eepromSensorDelay, g_eepromShadow[eepromSensorDelay]);
  }
  else if (g_subMenu == 3)
  {
    g_lcdSerial.print("Cycle (sec):");
    button = lcdSetNumber(13, 1, START_POS_MIDDLE, 3, cycleSec, -1, SENSOR_NONE, 9);
    g_eepromShadow[eepromSensorCycle] = numArrayToInt(cycleSec, 3);
    eepromWriteInt(eepromSensorCycle, g_eepromShadow[eepromSensorCycle]);
  }

  if (button == BUTTON_S_RIGHT)
    g_subMenu = (g_subMenu + 1) % subMenuSize;
  else if (button == BUTTON_S_LEFT)
    g_subMenu = (g_subMenu > 0) ? (g_subMenu-1) : (subMenuSize-1);

  waitTillAllButtonsReleased();
  return button;
}  

// This menu is used for setting up all the different sensors
int sensorSetupMenu(char *sensorName, int sensorPin, int eepromSensorTriggerLowHigh, int eepromSensorTriggerVal, int eepromSensorPower)
{
  byte triggerVal[3];
  char *triggerLowHighStr[2]     = { "Low ", "High"  };
  char *triggerPowerStr[3]       = { "On         ", "Off_Sensor1", "Off_Sensor2" };
  int  subMenuSize               = 3;
  int button;
 
  intToNumArray(g_eepromShadow[eepromSensorTriggerVal], triggerVal, 3);

  lcdOpCode(LCD_CLEAR_SCREEN);
  g_lcdSerial.print(sensorName);
  lcdSetPosition(0, 1);
  

  if (g_subMenu == 0)
  {
    g_lcdSerial.print("Trigger on:");
    button = lcdSetString(12, 1, 2, &g_eepromShadow[eepromSensorTriggerLowHigh], triggerLowHighStr);
    eepromWriteInt(eepromSensorTriggerLowHigh, g_eepromShadow[eepromSensorTriggerLowHigh]);
  }
  else if (g_subMenu == 1)
  {
    g_lcdSerial.print("Value:");
    int sensorAvgMethod;
    if(g_eepromShadow[eepromSensorTriggerLowHigh] == 0)
      sensorAvgMethod = SENSOR_MIN;
    else
     sensorAvgMethod = SENSOR_MAX;

    button = lcdSetNumber(13, 1, START_POS_MIDDLE, 3, triggerVal, sensorPin, sensorAvgMethod, 9);
    g_eepromShadow[eepromSensorTriggerVal] = numArrayToInt(triggerVal, 3);
    eepromWriteInt(eepromSensorTriggerVal, g_eepromShadow[eepromSensorTriggerVal]);
  }
  else if (g_subMenu == 2)
  {
    g_lcdSerial.print("Pow:");
    button = lcdSetString(5, 1, 3, &g_eepromShadow[eepromSensorPower], triggerPowerStr);
    eepromWriteInt(eepromSensorPower, g_eepromShadow[eepromSensorPower]);
  }

  if (button == BUTTON_S_RIGHT)
    g_subMenu = (g_subMenu + 1) % subMenuSize;
  else if (button == BUTTON_S_LEFT)
    g_subMenu = (g_subMenu > 0) ? (g_subMenu-1) : (subMenuSize-1);

  waitTillAllButtonsReleased();
  return button;
}

// This is the menu that sets up projectile
int projectileSetupMenu()
{
  char *triggerLowHighStr[2]     = { "Low ", "High"  };
  byte targetDistance[3];
  int  subMenuSize               = 2;
  int button;

  intToNumArray(g_eepromShadow[EEPROM_PROJECTILE_SENSOR_DISTANCE], targetDistance, 3);
  
  lcdOpCode(LCD_CLEAR_SCREEN);
  lcdSetPosition(0, 0);
  g_lcdSerial.print("Projectile");
  lcdSetPosition(0, 1);

  if (g_subMenu == 0)
  {
    if (g_inchCm == UNIT_INCH)
      g_lcdSerial.print("Dist (inch)  ");
    else
      g_lcdSerial.print("Dist (cm)    ");
      
    lcdPrintWithZeros(13, 1, 3, g_eepromShadow[EEPROM_PROJECTILE_SENSOR_DISTANCE]);

    button = lcdSetNumber(13, 1, START_POS_MIDDLE, 3, targetDistance, -1, SENSOR_MIN, 9);
    g_eepromShadow[EEPROM_PROJECTILE_SENSOR_DISTANCE] = numArrayToInt(targetDistance, 3);
    eepromWriteInt(EEPROM_PROJECTILE_SENSOR_DISTANCE, g_eepromShadow[EEPROM_PROJECTILE_SENSOR_DISTANCE]);
  }
  else if (g_subMenu == 1)
  {
    g_lcdSerial.print("Trigger on:");
    button = lcdSetString(12, 1, 2, &g_eepromShadow[EEPROM_PROJECTILE_SENSOR_LOW_HIGH], triggerLowHighStr);
    eepromWriteInt(EEPROM_PROJECTILE_SENSOR_LOW_HIGH, g_eepromShadow[EEPROM_PROJECTILE_SENSOR_LOW_HIGH]);
  }

  if (button == BUTTON_S_RIGHT)
    g_subMenu = (g_subMenu + 1) % subMenuSize;
  else if (button == BUTTON_S_LEFT)
    g_subMenu = (g_subMenu > 0) ? (g_subMenu-1) : (subMenuSize-1);

  waitTillAllButtonsReleased();
  return button;
}

// This menu is used for setting up the valve sensor
int valveSetupMenu()
{
  byte drop1Size[3];
  byte drop2Delay[3];
  byte drop2Size[3];
  byte flashDelay[3];
  int  subMenuSize = 4;
  int button;
 
  intToNumArray(g_eepromShadow[EEPROM_VALVE_DROP1_SIZE], drop1Size, 3);
  intToNumArray(g_eepromShadow[EEPROM_VALVE_DROP2_DELAY], drop2Delay, 3);
  intToNumArray(g_eepromShadow[EEPROM_VALVE_DROP2_SIZE], drop2Size, 3);
  intToNumArray(g_eepromShadow[EEPROM_VALVE_PHOTO_DELAY], flashDelay, 3);

  lcdOpCode(LCD_CLEAR_SCREEN);
  g_lcdSerial.print("Valve Sensor");
  lcdSetPosition(0, 1);

  if (g_subMenu == 0)
  {
    g_lcdSerial.print("Drop1 Size:");
    button = lcdSetNumber(13, 1, START_POS_MIDDLE, 3, drop1Size, -1, SENSOR_MIN, 9);
    g_eepromShadow[EEPROM_VALVE_DROP1_SIZE] = numArrayToInt(drop1Size, 3);
    eepromWriteInt(EEPROM_VALVE_DROP1_SIZE, g_eepromShadow[EEPROM_VALVE_DROP1_SIZE]);
    waitTillAllButtonsReleased();
  }
  else if (g_subMenu == 1)
  {
    g_lcdSerial.print("Drop2 Delay:");
    button = lcdSetNumber(13, 1, START_POS_MIDDLE, 3, drop2Delay, -1, SENSOR_MIN, 9);
    g_eepromShadow[EEPROM_VALVE_DROP2_DELAY] = numArrayToInt(drop2Delay, 3);
    eepromWriteInt(EEPROM_VALVE_DROP2_DELAY, g_eepromShadow[EEPROM_VALVE_DROP2_DELAY]);
    waitTillAllButtonsReleased();
  }
  else if (g_subMenu == 2)
  {
    g_lcdSerial.print("Drop2 Size:");
    button = lcdSetNumber(13, 1, START_POS_MIDDLE, 3, drop2Size, -1, SENSOR_MIN, 9);
    g_eepromShadow[EEPROM_VALVE_DROP2_SIZE] = numArrayToInt(drop2Size, 3);
    eepromWriteInt(EEPROM_VALVE_DROP2_SIZE, g_eepromShadow[EEPROM_VALVE_DROP2_SIZE]);
    waitTillAllButtonsReleased();
  }
  else if (g_subMenu == 3)
  {
    g_lcdSerial.print("Flash Delay:");
    button = lcdSetNumber(13, 1, START_POS_MIDDLE, 3, flashDelay, -1, SENSOR_MIN, 9);
    g_eepromShadow[EEPROM_VALVE_PHOTO_DELAY] = numArrayToInt(flashDelay, 3);
    eepromWriteInt(EEPROM_VALVE_PHOTO_DELAY, g_eepromShadow[EEPROM_VALVE_PHOTO_DELAY]);
    waitTillAllButtonsReleased();
  }  

  if (button == BUTTON_S_RIGHT)
    g_subMenu = (g_subMenu + 1) % subMenuSize;
  else if (button == BUTTON_S_LEFT)
    g_subMenu = (g_subMenu > 0) ? (g_subMenu-1) : (subMenuSize-1);

  return button;
}


#define TL_START_HOUR 0
#define TL_END_HOUR   1
#define TL_START_MIN  2
#define TL_END_MIN    3
#define TL_START_SEC  4
#define TL_END_SEC    5

// This is the menu that sets up timelapse
int timelapseSetupMenu()
{
  byte seconds[2];
  byte minutes[2];
  byte hours[3];
  int button = BUTTON_S_UP; // Just can't be menu or set
  int cursorPos = TL_START_HOUR;
  
  intToNumArray(g_eepromShadow[EEPROM_TIMELAPSE_HOURS], hours, 3);
  intToNumArray(g_eepromShadow[EEPROM_TIMELAPSE_MINUTES], minutes, 2);
  intToNumArray(g_eepromShadow[EEPROM_TIMELAPSE_SECONDS], seconds, 2);
  
  lcdOpCode(LCD_CLEAR_SCREEN);
  lcdSetPosition(0, 0);
  g_lcdSerial.print("Timelapse");
  lcdPrintWithZeros(0, 1, 3, g_eepromShadow[EEPROM_TIMELAPSE_HOURS]);
  g_lcdSerial.print(":");
  lcdPrintWithZeros(4, 1, 2, g_eepromShadow[EEPROM_TIMELAPSE_MINUTES]);
  g_lcdSerial.print(":");
  lcdPrintWithZeros(7, 1, 2, g_eepromShadow[EEPROM_TIMELAPSE_SECONDS]);

  while( (button != BUTTON_S_MENU) && (button != BUTTON_S_SET) )
  {
    if (cursorPos == TL_START_HOUR)
    {
      button = lcdSetNumber(0, 1, START_POS_BEGINNING, 3, hours, -1, SENSOR_MIN, 9);
      g_eepromShadow[EEPROM_TIMELAPSE_HOURS] = numArrayToInt(hours, 3);
      if (button == BUTTON_S_LEFT)
        cursorPos = TL_START_HOUR;
      else if (button == BUTTON_S_RIGHT)
        cursorPos = TL_START_MIN;
    }
    else if (cursorPos == TL_END_HOUR)
    {
      button = lcdSetNumber(0, 1, START_POS_END, 3, hours, -1, SENSOR_MIN, 9);
      g_eepromShadow[EEPROM_TIMELAPSE_HOURS] = numArrayToInt(hours, 3);
      if (button == BUTTON_S_LEFT)
        cursorPos = TL_START_HOUR;
      else if (button == BUTTON_S_RIGHT)
        cursorPos = TL_START_MIN;
    }
    else if (cursorPos == TL_START_MIN)
    {
      button = lcdSetNumber(4, 1, START_POS_BEGINNING, 2, minutes, -1, SENSOR_MIN, 5);
      g_eepromShadow[EEPROM_TIMELAPSE_MINUTES] = numArrayToInt(minutes, 2);
      if (button == BUTTON_S_LEFT)
        cursorPos = TL_END_HOUR;
      else if (button == BUTTON_S_RIGHT)
        cursorPos = TL_START_SEC;
    }
    else if (cursorPos == TL_END_MIN)
    {
      button = lcdSetNumber(4, 1, START_POS_END, 2, minutes, -1, SENSOR_MIN, 5);
      g_eepromShadow[EEPROM_TIMELAPSE_MINUTES] = numArrayToInt(minutes, 2);
      if (button == BUTTON_S_LEFT)
        cursorPos = TL_END_HOUR;
      else if (button == BUTTON_S_RIGHT)
        cursorPos = TL_START_SEC;
      
    }
    else if (cursorPos == TL_START_SEC)
    {
      button = lcdSetNumber(7, 1, START_POS_BEGINNING, 2, seconds, -1, SENSOR_MIN, 5);
      g_eepromShadow[EEPROM_TIMELAPSE_SECONDS] = numArrayToInt(seconds, 2);
      if (button == BUTTON_S_LEFT)
        cursorPos = TL_END_MIN;
      else if (button == BUTTON_S_RIGHT)
        cursorPos = TL_END_SEC;
      
    }
    else if (cursorPos == TL_END_SEC)
    {
      button = lcdSetNumber(7, 1, START_POS_END, 2, seconds, -1, SENSOR_MIN, 5);
      g_eepromShadow[EEPROM_TIMELAPSE_SECONDS] = numArrayToInt(seconds, 2);
      if (button == BUTTON_S_LEFT)
        cursorPos = TL_END_MIN;
      else if (button == BUTTON_S_RIGHT)
        cursorPos = TL_END_SEC;
    }
    eepromWriteInt(EEPROM_TIMELAPSE_HOURS, g_eepromShadow[EEPROM_TIMELAPSE_HOURS]);
    eepromWriteInt(EEPROM_TIMELAPSE_MINUTES, g_eepromShadow[EEPROM_TIMELAPSE_MINUTES]);
    eepromWriteInt(EEPROM_TIMELAPSE_SECONDS, g_eepromShadow[EEPROM_TIMELAPSE_SECONDS]);
    waitTillAllButtonsReleased();
  }

  waitTillAllButtonsReleased();
  return button;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// General Helper Functions
////////////////////////////////////////////////////////////////////////////////////////////////////

// Converts a number array to an integer
int numArrayToInt(byte *array, int size)
{
  int i;
  int val = 0;
  int mult = 1;
  for (i=size-1; i>=0; --i)
  {
    val += ((int)array[i]) * mult;
    mult *= 10;
  }
  return val;
}

// Converts an integer to a number array
void intToNumArray(int num, byte *array, int size)
{
  int i;
  for (i=size-1; i>=0; --i)
  {
    array[i] = num%10;
    num /= 10;
  }
}

// Writes an integer to eeprom
void eepromWriteInt(int addr, int val)
{
  addr *= 2;  // int is 2 bytes
  EEPROM.write(addr+1, val&0xFF);
  val /= 256;
  EEPROM.write(addr+0, val&0xFF);
}

// Reads an integer from eeprom
int eepromReadInt(int addr, int minVal, int maxVal)
{
  int val;

  addr *= 2;  // int is 2 bytes
  val = EEPROM.read(addr+0);
  val *= 256;
  val |= EEPROM.read(addr+1);
  val = constrain(val, minVal, maxVal);
  return val;
}

// Wait for all the current button presses to end (handles debouncing buttons)
void waitTillAllButtonsReleased()
{
  while(1)
  {
    int i;
    int buttonMenu=1, buttonSet=1, buttonUp=1, buttonDown=1, buttonLeft=1, buttonRight=1;  // BUTTON_NOT_PRESSED is 1

    // Need to sample many times to makes sure the button isn't currently bouncing
    for(i=0; i<100; ++i)
    {
      buttonMenu  &= digitalRead(BUTTON_MENU_PIN);
      buttonSet   &= digitalRead(BUTTON_SET_PIN);
      buttonUp    &= digitalRead(BUTTON_UP_PIN);
      buttonDown  &= digitalRead(BUTTON_DOWN_PIN);
      buttonLeft  &= digitalRead(BUTTON_LEFT_PIN);
      buttonRight &= digitalRead(BUTTON_RIGHT_PIN);
      delayMicroseconds(10);
    }

    if ((buttonMenu == BUTTON_NOT_PRESSED) && (buttonSet == BUTTON_NOT_PRESSED) && 
      (buttonUp == BUTTON_NOT_PRESSED)   && (buttonDown == BUTTON_NOT_PRESSED) && 
      (buttonLeft == BUTTON_NOT_PRESSED) && (buttonRight == BUTTON_NOT_PRESSED))
    {
      break;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// LCD Helper Functions
////////////////////////////////////////////////////////////////////////////////////////////////////
// Assumes you're using LCD NHD-0216K3Z-FL-GBW 

// Sends an LCD op code to the LCD
void lcdOpCode(byte code)
{
  g_lcdSerial.print(LCD_OP_CODE, BYTE); // Tells LCD the next byte is a special op code
  g_lcdSerial.print(code, BYTE);
}

// LCD I'm using is 16x2 characters.  Line 0's viewable cursor positions is 0-0xF and
// line 1's viewable curso position is 0x40-0x4f.  X is expected to be 0-0xF and Y is
// expected to be 0-1.
void lcdSetPosition(byte x, byte y)
{
  byte positionCode = (y*0x40 + x);
  lcdOpCode(LCD_SET_CURSOR_POSITION);
  g_lcdSerial.print(positionCode, BYTE);
}

// Set the backlight brightness for LCD.  1 is a dim backlight and 8 is a bright blacklight.
void lcdSetBrightness(byte brightness)
{
  lcdOpCode(LCD_BRIGHTNESS);
  g_lcdSerial.print(brightness, BYTE);
}

// Prints an array of numbers (store one digit per array element) to the LCD
void lcdPrintNumArray(byte x, byte y, byte length, byte *val)
{
  int i;
  lcdSetPosition(x ,y);
  for (i=0; i<length; ++i)
  {
    g_lcdSerial.print((int)(val[i]));
  }
}

// Prints an integer with leading zeros
void lcdPrintWithZeros(byte x, byte y, byte length, int val)
{
  int tempVal = 1;

  // pow() returns wrong value (broken in 0014)
  --length;
  while (length)
  {
    tempVal *= 10;
    --length;
  }

  lcdSetPosition(x ,y);
  while (tempVal && ((val/tempVal) == 0))
  {
    g_lcdSerial.print("0");
    tempVal /= 10;
  }
  if (tempVal)
  {
    g_lcdSerial.print(val, DEC);
  }
}

// Sets a number using buttons and the LCD (up to increase number, down decrease number, left to move to left digit, right to move to right digit) (numbers wrap from 9->0 and 0->9)
int lcdSetNumber(int cursorOrigX, int cursorOrigY, int startPos, int length, byte *val, int sensorPin, int sensorAvgMethod, int maxUpperDigit)
{
  int cursorPos;
  int cursorPosMax     = length - 1;
  int prevAnyPressed   = BUTTON_NOT_PRESSED;
  int displaySensor    = 1;

  if (startPos == START_POS_MIDDLE)
    cursorPos = length/2;
  else if (startPos == START_POS_BEGINNING)
    cursorPos = 0;
  else if (startPos == START_POS_END)
    cursorPos = cursorPosMax;

  lcdPrintNumArray(cursorOrigX, cursorOrigY, length, val);
  lcdSetPosition(cursorOrigX+cursorPos, cursorOrigY);
  lcdOpCode(LCD_CURSOR_UNDERLINE);

  while(1)
  {
    int buttonMenu, buttonSet, buttonUp, buttonDown, buttonLeft, buttonRight;

    if (sensorPin >= 0)
    {
      unsigned long curTime, endTime;
      int sensorVal = analogRead(sensorPin);

      sensorVal = min(999, sensorVal);
      curTime = endTime = millis();

      // Sample analag values for 1000 ms.  Each sample takes about 100 us so we should get about 10000 samples
      // The reason we want so many samples is so the lcd screen doesn't change so fast it's unreadable.
      // We will exit out early if any of the buttons are pressed.
      endTime += 1000;
      while (curTime <= endTime)
      {
        int i;
        for (i=0; i<10; ++i) // This loop keeps button checking from influencing results too much
        {
          int curVal = analogRead(sensorPin);
          curVal = min(999, curVal);

          if (sensorAvgMethod == SENSOR_MIN)
            sensorVal = min(sensorVal, curVal);
          else if (sensorAvgMethod == SENSOR_MAX)
            sensorVal = max(sensorVal, curVal);
        }
        buttonMenu  = digitalRead(BUTTON_MENU_PIN);
        buttonSet   = digitalRead(BUTTON_SET_PIN);
        buttonUp    = digitalRead(BUTTON_UP_PIN);
        buttonDown  = digitalRead(BUTTON_DOWN_PIN);
        buttonLeft  = digitalRead(BUTTON_LEFT_PIN);
        buttonRight = digitalRead(BUTTON_RIGHT_PIN);
        
        if ((buttonMenu == BUTTON_PRESSED) || (buttonSet == BUTTON_PRESSED) || (buttonUp == BUTTON_PRESSED) ||
            (buttonDown == BUTTON_PRESSED) || (buttonLeft == BUTTON_PRESSED) || (buttonRight == BUTTON_PRESSED))
        {
          displaySensor = 0;
          break;
        }

        curTime = millis();
      }
      
      if (displaySensor)
      { 
        lcdOpCode(LCD_CURSOR_UNDERLINE_OFF);
        lcdSetPosition(12, 0);
        g_lcdSerial.print(" ");
        lcdPrintWithZeros(13, 0, 3, sensorVal);
      }
      else
      {
        lcdOpCode(LCD_CURSOR_UNDERLINE_OFF);
        lcdSetPosition(12, 0);
        g_lcdSerial.print("    ");
      }
      displaySensor = 1;
      lcdSetPosition(cursorOrigX+cursorPos, cursorOrigY);
      lcdOpCode(LCD_CURSOR_UNDERLINE);
    }

    buttonMenu  = digitalRead(BUTTON_MENU_PIN);
    buttonSet   = digitalRead(BUTTON_SET_PIN);
    buttonUp    = digitalRead(BUTTON_UP_PIN);
    buttonDown  = digitalRead(BUTTON_DOWN_PIN);
    buttonLeft  = digitalRead(BUTTON_LEFT_PIN);
    buttonRight = digitalRead(BUTTON_RIGHT_PIN);

    if (buttonMenu == BUTTON_PRESSED)
    {
      lcdOpCode(LCD_CURSOR_UNDERLINE_OFF);
      return BUTTON_S_MENU;
    }
    if (buttonSet == BUTTON_PRESSED)
    {
      lcdOpCode(LCD_CURSOR_UNDERLINE_OFF);
      return BUTTON_S_SET;
    }
    if (prevAnyPressed == BUTTON_NOT_PRESSED)
    {
      if (buttonUp == BUTTON_PRESSED)
      {
        int maxDigit;
  
        if (cursorPos == 0)
          maxDigit = maxUpperDigit+1;
        else
          maxDigit = 10;
  
        val[cursorPos]++;
        if (val[cursorPos] >= maxDigit)
        {
          val[cursorPos] = 0;
        }
        lcdPrintNumArray(cursorOrigX, cursorOrigY, length, val);
        lcdSetPosition(cursorOrigX+cursorPos, cursorOrigY);
      }
      if (buttonDown == BUTTON_PRESSED)
      {
        int maxDigit;
  
        if (cursorPos == 0)
          maxDigit = maxUpperDigit;
        else
          maxDigit = 9;
  
        val[cursorPos]--;
        if (val[cursorPos] > 250 ) // Unsigned byte wraps back to 255 instead of going negative
        {
          val[cursorPos] = maxDigit;
        }
        lcdPrintNumArray(cursorOrigX, cursorOrigY, length, val);
        lcdSetPosition(cursorOrigX+cursorPos, cursorOrigY);
      }
      if (buttonLeft == BUTTON_PRESSED)
      {
        cursorPos--;
        if (cursorPos < 0)
        {
          lcdOpCode(LCD_CURSOR_UNDERLINE_OFF);
          return BUTTON_S_LEFT;
        }
        lcdSetPosition(cursorOrigX+cursorPos, cursorOrigY);
      }
      if (buttonRight == BUTTON_PRESSED)
      {
        cursorPos++;
        if (cursorPos > cursorPosMax)
        {
          lcdOpCode(LCD_CURSOR_UNDERLINE_OFF);
          return BUTTON_S_RIGHT;
        }
        lcdSetPosition(cursorOrigX+cursorPos, cursorOrigY);
      }
    }
    if ((buttonUp == BUTTON_NOT_PRESSED) && (buttonDown == BUTTON_NOT_PRESSED) && 
        (buttonLeft == BUTTON_NOT_PRESSED) && (buttonRight == BUTTON_NOT_PRESSED))
    {
      if (prevAnyPressed == BUTTON_PRESSED)
      {
        prevAnyPressed = BUTTON_NOT_PRESSED;
        delay(10); //Prevent button bounce
      }
    }
    else
    {
      prevAnyPressed = BUTTON_PRESSED;
    }
  }
}

// Sets a string using buttons and the LCD (up/down to select string and right/left to move to previous/next submenu)
int lcdSetString(int cursorOrigX, int cursorOrigY, int maxStrings, int *val, char **str)
{
  int prevAnyPressed    = BUTTON_NOT_PRESSED;

  lcdSetPosition(cursorOrigX, cursorOrigY);
  g_lcdSerial.print(str[*val]);

  lcdSetPosition(cursorOrigX, cursorOrigY);
  lcdOpCode(LCD_CURSOR_UNDERLINE);

  while(1)
  {
    int buttonMenu, buttonSet, buttonUp, buttonDown, buttonLeft, buttonRight;

    buttonMenu  = digitalRead(BUTTON_MENU_PIN);
    buttonSet   = digitalRead(BUTTON_SET_PIN);
    buttonUp    = digitalRead(BUTTON_UP_PIN);
    buttonDown  = digitalRead(BUTTON_DOWN_PIN);
    buttonLeft  = digitalRead(BUTTON_LEFT_PIN);
    buttonRight = digitalRead(BUTTON_RIGHT_PIN);

    if (buttonMenu == BUTTON_PRESSED)
    {
      lcdOpCode(LCD_CURSOR_UNDERLINE_OFF);
      return BUTTON_S_MENU;
    }
    if (buttonSet == BUTTON_PRESSED)
    {
      lcdOpCode(LCD_CURSOR_UNDERLINE_OFF);
      return BUTTON_S_SET;
    }
    if (buttonLeft == BUTTON_PRESSED)
    {
      lcdOpCode(LCD_CURSOR_UNDERLINE_OFF);
      return BUTTON_S_LEFT;
    }
    if (buttonRight == BUTTON_PRESSED)
    {
      lcdOpCode(LCD_CURSOR_UNDERLINE_OFF);
      return BUTTON_S_RIGHT;
    }

    if (prevAnyPressed == BUTTON_NOT_PRESSED)
    {
      if (buttonUp == BUTTON_PRESSED)
      {
        (*val) = (*val > 0) ? ((*val)-1) : (maxStrings-1);
        g_lcdSerial.print(str[*val]);
        lcdSetPosition(cursorOrigX, cursorOrigY);
      }
      if (buttonDown == BUTTON_PRESSED)
      {
        (*val) = ((*val)+1) % maxStrings;
        g_lcdSerial.print(str[*val]);
        lcdSetPosition(cursorOrigX, cursorOrigY);
      }
    }

    if ((buttonUp == BUTTON_NOT_PRESSED) && (buttonDown == BUTTON_NOT_PRESSED))
    {
      if (prevAnyPressed == BUTTON_PRESSED)
      {
        prevAnyPressed = BUTTON_NOT_PRESSED;
        delay(10); //Prevent button bounce
      }
    }
    else
    {
      prevAnyPressed = BUTTON_PRESSED;
    }
  }
}



