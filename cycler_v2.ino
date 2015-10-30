/*
 * ARDUINO UNO THERMAL CYCLER WITH LCD DISPLAY
 * For use with the Polymerase Chain Reaction (PCR) process
 * 
 * A Global Ties - Engineering World Health Team Project
 *
 */

#include <avr/sleep.h>
#include <math.h>
#include "LCDScreen.h"
#include "Adafruit_RGBLCDShield.h"
#include <Wire.h>

#define pin_thermistor   0
#define pin_heater_PWR   6
#define pin_heater_DIR   7
#define pin_fan_PWR      11
#define pin_fan_DIR      12

#define led_red          3
#define led_blue         4
#define led_green        5

LCDScreen UI = LCDScreen(); 	// LCD display

double TEMP_CEILING;		// Upper and
double TEMP_FLOOR;		// lower temperature limits to cycle between
double FINAL_FLOOR = 50; 	// Temperature to drop to on wrap()

double CRITICAL_MAX;

double WARNING_LIMIT  = 50;

int MAX_CYCLES;                 // Cycles to perform
int MAINTAIN_DURATION =  10000; // Hold at temperature for 10 seconds
int PRINT_DELAY       =  1000;  // Print to serial monitor once every second (1000 ms)
boolean DEBUG =  false; 	// Change to true to test fan and heater before running
boolean ERRORCHECK = false;
int current_cycle;		// No. of cycles passed
double PCR_time_start;		// Start time of process

void setup() 
{

  Serial.println( "Setting up ... " );

  Serial.begin( 9600 );			// Set the data rate (baud) with which uno comms with comp

  /* Heater and Fan are both outputs */
  pinMode( pin_heater_PWR, OUTPUT );   
  pinMode( pin_heater_DIR, OUTPUT );
  pinMode( pin_fan_PWR,    OUTPUT );
  pinMode( pin_fan_DIR,    OUTPUT ); 

  pinMode( led_red,       OUTPUT );   
  pinMode( led_blue,      OUTPUT );
  pinMode( led_green,     OUTPUT );
  
  setLight( 0 ); // green

  /* initial status of Heater and Fan */
  digitalWrite( pin_heater_DIR, HIGH ); // HIGH = 5V, LOW = 0V (Ground) 
  analogWrite ( pin_heater_PWR, 0    ); // Heater is on but not running
  digitalWrite( pin_fan_DIR,    LOW  ); // Fan polarity is LOW, not running
  analogWrite ( pin_fan_PWR,    0    ); // Fan is off

  current_cycle = 0;			// No cycles complete yet.

  if ( DEBUG ) 				// Test if fan and heater are running
  { 

    double room_temp = Thermistor( analogRead( pin_thermistor ) );

    Serial.println( "Testing heater ... " );
    RampUp( room_temp + 10 );

    Serial.println( "Testing fan ... " );
    RampDown( room_temp - 5 );

    DEBUG = false;

    setLight( 0 );
  }

  heaterOff();				// In case of reset
  fanOff();

  UI.init();				// Boot up LCD screen
  UI.setup();  				// Allow user to select preferences

  /* Collect temperature preferences from UI */
  TEMP_FLOOR = UI.retrieveTF();
  TEMP_CEILING = UI.retrieveTC();
  CRITICAL_MAX = TEMP_CEILING + 20;
  MAX_CYCLES = UI.retrieveMC();

  Serial.println( "Setup complete.\n" );

  PCR_time_start = millis();		// Start the clock

}

void loop() 
{

  if ( current_cycle < MAX_CYCLES )	// More cycles to go
  {
    UI.displayInitialStatus( Thermistor( analogRead( pin_thermistor ) ), current_cycle + 1 );

    Serial.print( "Current Cycle: " );
    Serial.print( ++current_cycle );
    Serial.print( "/" );
    Serial.println( MAX_CYCLES );
    Serial.print( "( Cycling between " );
    Serial.print( TEMP_FLOOR );
    Serial.print( " and " );
    Serial.print( TEMP_CEILING );
    Serial.println( " )" );    

    RampUp( TEMP_CEILING ); 		// State 1: Heat up sample to TEMP_CEILING
    Maintain( TEMP_CEILING );		// State 2: Keep sample heated at TEMP_CEILING for MAINTAIN_DURATION
    RampDown( TEMP_FLOOR );		// State 3: Cool down sample to TEMP_FLOOR
    delay( MAINTAIN_DURATION );    

  }
  else if ( current_cycle == MAX_CYCLES ) // All cycles completed
  {
    Serial.print( current_cycle++ );
    Serial.println(" cycles completed." );

    wrapUp();				// State 4: Cool sample close to room temperature, turn off fan and heater

    double total_running_time = ( millis() - PCR_time_start )/1000; // Get final running time in seconds

    Serial.print( "PCR process completed in " ); 
    Serial.print( total_running_time );
    Serial.println( " seconds." );

    UI.finalMess();			// Print goodbye message to LCD

    sleep();				// State 5: Sleep, do nothing until reset

  }
  else 					
  {
    // do nothing 
  }

}

/* 
 * STATE 1
 * Heat up the sample to target ceiling temperature 
 *
 */
void RampUp( int target )
{
  Serial.println( "Ramping up ... " ); 
  double startTime = millis();  
  /* Get current temperature */
  double ActualTemp = Thermistor( analogRead( pin_thermistor ) );

  setLight( 1 ); // red
  analogWrite( pin_heater_PWR, 255 );
  
  int warning = 0;
  double timeToPrint = 0;
  while( ActualTemp < target ) 
  {
    /* Refresh current temperature */
    double newTemp = ( Thermistor( analogRead( 0 ) ) );

    UI.updateArrow( newTemp );
    /* Compare to previous temperature, should increase */
    if ( newTemp <= ActualTemp )
    {
      if ( ERRORCHECK ) { 
        if ( warning > WARNING_LIMIT )
        {
          Serial.println( "Error: Temperature not increasing on RampUp()." );
          Serial.println( "Please make sure device is correctly wired." );
          heaterOff();
          UI.printError( 0 );
          setLight( 3 ); // yellow
          while ( true ) { 
          }
        }
        else 
        { 
          warning++; 
        }
      }
    }
    else if ( newTemp > CRITICAL_MAX )
    {
      Serial.println( "Error: Temperature surpassing critical maximum" );
      Serial.println( "Please make sure heater is functioning correctly" );
      heaterOff();
      UI.printError( 1 );
      setLight( 3 ); // yellow
      while ( true ) { 
      }
    }
    else { 
      warning = 0; 
    }

    ActualTemp = newTemp;

    timeToPrint = checkPrint( timeToPrint, ActualTemp );

  }

  heaterOff();
  double totalTime = ( millis() - startTime )/1000;
  Serial.print( "RampUp finished in " );
  Serial.print( totalTime );
  Serial.println( " seconds.\n" );

}


/* 
 * STATE 2
 * Keep the sample heated at target ceiling temperature for a specified duration
 *
 */
void Maintain( int target )
{
  Serial.println( "Maintaining temperature ..." );  
  double time = millis();
  double stopTime = time + MAINTAIN_DURATION;

  double nextPrint = time + PRINT_DELAY;
  setLight( 1 ); // red
  while ( time < stopTime )
  {
    time = millis();
    nextPrint = PID( Thermistor( analogRead( 0 ) ), target, true, time, nextPrint );
  }
  setLight( 0 ); // green
  Serial.println( "\n" );
}

/* 
 * STATE 3
 * Cool down the sample to target floor temperature 
 *
 */
void RampDown( int target )
{

  double startTime = millis();
  double ActualTemp = Thermistor( analogRead( pin_thermistor ) );

  analogWrite( pin_fan_PWR, 255 );
  Serial.println( "Ramping down ... ");

  double timeToPrint = 0;
  int warning = 0;
  while( ActualTemp >= TEMP_FLOOR )
  {
    int newTemp = ( Thermistor( analogRead( 0 ) ) );
    UI.updateArrow( newTemp );
    if ( ActualTemp < newTemp )
    {
      if ( ERRORCHECK ) {
        
        if ( warning > WARNING_LIMIT )
        {
          Serial.println( "Error: Temperature not decreasing on RampDown()." );
          Serial.println( "Please make sure that device is correctly wired." );
          fanOff();
          UI.printError( 0 );
          while ( true ) { 
          }
        }
        else { 
          warning++; 
        }
      }
    }
    else { 
      warning = 0; 
    }

    ActualTemp = newTemp;

    timeToPrint = checkPrint( timeToPrint, ActualTemp );

  }

  fanOff();

  double totalTime = ( millis() - startTime )/1000;
  Serial.print( "RampDown finished in " );
  Serial.print( totalTime );
  Serial.println( " seconds.\n" );

}

/* 
 * STATE 4
 * Cool the sample to close to room temperature, turn off all outputs 
 *
 */
void wrapUp()
{ 

  analogWrite( pin_heater_PWR, 0 );

  Serial.println( "Wrapping up..." );
  double ActualTemp = Thermistor( analogRead( pin_thermistor ) );

  setLight( 2 );
  analogWrite( pin_fan_PWR, 255 );

  double timeToPrint = 0;
  int warning = 0;
  while( ActualTemp > FINAL_FLOOR )
  {
    int newTemp=( Thermistor( analogRead( 0 ) ) );

    if ( ActualTemp < newTemp )
    {
      if ( ERRORCHECK ) {
        if ( warning > WARNING_LIMIT )
        {
          Serial.println( "Error: Temperature not decreasing on WrapUp()." );
          Serial.println( "Check log for details." );
          fanOff();
          UI.printError( 2 );
          setLight( 3 );
          while ( true ) { }
        }
        else { 
          warning++; 
        }
      }
    }
    else { 
      warning = 0; 
    }

    ActualTemp = newTemp;

    timeToPrint = checkPrint( timeToPrint, ActualTemp );

  }

  analogWrite( pin_fan_PWR, 0 );
  setLight( 0 );
  Serial.print( "Wrap finished. Fan is turning off. Current temperature: " );
  Serial.print( ActualTemp );
  Serial.println( "\n" );
}

/* 
 * STATE 5
 * Sleep until external reset
 *
 */
void sleep()
{
  Serial.println( "Turning off heater and fan ... \n" ); 
  analogWrite( pin_heater_PWR, 0 );         // Heater is on but not running
  analogWrite( pin_fan_PWR, 0);

  Serial.println( "Putting device to sleep. Please restart the device." );
  
  setLight( 0 );

  set_sleep_mode( SLEEP_MODE_PWR_DOWN );
  cli();
  sei();
  sleep_cpu();

}

double PID( double Actual, double SetPt, boolean reset, double currentTime, double nextPrint ) 
{
  double kP=100, kI=0, kD=10;

  static double Previous_Error = 0, Integral = 0, time_one = 0;

  if( reset == true )
  {
    Previous_Error = 0;
    Integral = 0;
    time_one = 0;
  }

  double time_two = millis();
  double dt = ( time_two - time_one )/1000;

  double Error = SetPt-Actual;

  double P = kP*Error;
  double I = kI*(Integral + Error*dt);
  double D = kD*((Error - Previous_Error)/dt);

  double heat = (P+I+D);
  heat = saturation(heat);

  analogWrite( pin_heater_PWR,heat );
  Previous_Error=Error;
  Integral += Error*dt;
  time_one = time_two;


  nextPrint = checkPrint( nextPrint, Actual ); 

  return nextPrint;

}

double saturation(double heat)
{
  if( heat < 0 ) return 0;
  else if( heat > 255 ) return 255;
  else return heat;
}

void heaterOff()
{
  analogWrite( pin_heater_PWR, 0 );
  setLight( 0 ); // green

}

void fanOff()
{
  analogWrite( pin_fan_PWR, 0 );
  setLight( 0 ); // green
}

double Thermistor( int RawADC )			// Return temperature reading from sensor
{

  double temp = log( 2000.0 * ( 1024.0 / RawADC-1 ) );
  temp = 1 / ( 0.00103514 + 0.000233825 * temp + ( 0.0000000792467 * temp * temp * temp ) ); // Kelvin			

  temp = temp - 273.15; // Kelvin to Celsius Conversion 

  return temp;

}

double checkPrint( double nextPrint, double ActualTemp )
{
  double clock = millis();
  if( clock >= nextPrint )
  {
    nextPrint = clock + PRINT_DELAY;
    int inSeconds = clock/1000;
    Serial.print( inSeconds );
    Serial.print( "s");
    Serial.print( "\t" );
    Serial.print( ActualTemp );
    Serial.println( " C" );
  }

  return nextPrint;
}

/* color
   0 = green
   1 = red
   2 = blue
   3 = yellow
 */
double setLight( int color ) {
  
  int redVal, greenVal, blueVal;
  if ( color < 1 ) {
    redVal = 0;
    greenVal = 255;
    blueVal = 0;
  } else if ( color == 1 ) {
    redVal = 255;
    greenVal = 0;
    blueVal = 0;
  } else if ( color == 2 ) {
    redVal = 0;
    greenVal = 0;
    blueVal = 255; 
  } else if ( color == 3 ) {
    redVal = 255;
    greenVal = 255;
    blueVal = 0;
  }
  
  analogWrite( led_blue, blueVal );
  analogWrite( led_red, redVal );
  analogWrite( led_green, greenVal );  
}
