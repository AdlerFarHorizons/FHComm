
/*
 * XTendConfgTeensy - Configure the 1W Digi 9XTend (or other Digi modules) with AT commands
 * 
 * Hardware: Teensy 3.x attached to 9XTend module through Serial Port 2 (RX2/TX2 pins 9/10) with
 *           linear regulator providing 5V +/- 5% at 1A with level translators on serial interface.
 *           Of course other Digi modules are designed for 3.3V and probably can be directly powered by
 *           the Teensy 3.x 3.3V output for this program as well as other modes of operation.
 *           
 *           NOTE: Since Teensy 3.x is tolerant of 5V input logic levels, a level translator isn't
 *                 necessary with the 9XTend for bench conditions.
 *           NOTE: Teensy 3.x should be able to power the 9XTend from its 100mA 3.3V output when using this
 *                 program (haven't tried it).
 *           NOTE: Nothing special about the Teensy except for a real hardware serial port. Other Arduino-class
 *                 boards can be easily adapted using the AltSoftwareSerial library.
 *           
 * Assumptions: 9XTend parameters BD, AT, BT and CC at default values.
 *              Serial terminal interface provides both NL & CR line ending.
 * 
 * The Timer 2 interrupt routine is executed every 15 seconds to avoid the command mode timeout that occurs
 * after 20 sec of inactivity. This parameter doesn't seem to be programmable as indicated in the spec, so
 * this is the workaround. You can take the XTend out of command mode and end the program gracefully by entering 
 * "@" prior to powering off.
 */

#include <MsTimer2.h>
const int xtCmdPin = 16;
const int xtSleepPin = 15;
char c;

void setup() {
  MsTimer2::set( 15000, keepAlive );
  Serial2.begin(9600);
  Serial.begin(9600);
  while( !Serial );
  while( !Serial2 );
  Serial.print( "Entering command mode... " );
  Serial2.print("+++");
  delay( 1100 );
  while( Serial2.available() ) Serial.write( Serial2.read() );
  Serial.println( "" );
  Serial.println( "Enter Digi AT command sequence WITH \"AT\" prefix (upper or lower case accepted)." );
  Serial.println( "To exit command mode and end the session, enter the single character \"@\" (no quotes):" );
  MsTimer2::start();
}

void loop() {
  // put your main code here, to run repeatedly:
  while ( c != '@' ) { // Do until exit character, '@' appears
    if ( Serial.available() ) {
      toUpperCase( c = Serial.read() );
      Serial.write( c );
      if ( c != '@' ) {
        Serial2.write( c );
      } else {
        Serial2.println( "ATCN" );
        MsTimer2::stop();
      }
    }
    if ( Serial2.available() ) {
      c = Serial2.read();
      Serial.write( c );
      if ( c == 13 ) Serial.write( 10 );
    }
  }
  Serial.println( "\nAT command session ended." );
  while( true );
}

void toUpperCase( char ch ) {
  if ( (byte)ch > 96 && (byte)ch < 123 ) ch -= 32;
}

void keepAlive() {
  Serial2.println( "AT" );
  while( Serial2.read() != 13 );
  while( Serial2.available() ) Serial2.read();
}

