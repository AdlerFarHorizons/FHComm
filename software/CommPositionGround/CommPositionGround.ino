/*
 * Arduino IDE Board Selection: Teensy 3.1 or 3.2
 * Device: MK20DX256VLH7 (NXP K20P64M72SF1 Spec)
 * CPU Speed: 96MHz (optimized) Overclock
 * 
 * RTC: Vbat 1.71V min @ 1uA
 */

/*
 * COMMANDS;
 * 
 * Commands are entered via the terminal as (with CR and/or LF
 * terminator required):
 * 
 * @[R|L] <arg1> <arg2> ... <argN>
 * 
 * Where R or L signify a Remote or Local command.
 * 
 * Local commands are for interaction with the local unit, while
 * Remote commands are sent in the next TX packet.
 * 
 * Remote Commands:
 * 
 * A field that can accomodate multiple remote command strings is
 * appended to the telemetry data string with a ":" prefix separator.
 * The individual commands + arguments are simply appended
 * with the '@' serving as a delimiter. If there are no commands,
 * the field is blank, but the ':' delimiter is still present.
 * 
 * NOTE:
 *    Currently only a single command per packet is allowed.
 *    Any new commands are ignored until the packet is sent.
 * 
 */

/*
 * Serial:  External Serial Monitor through USB
 * Serial1: GPS
 * Serial2: RF data IN/OUT
 * 
 */

#include <TimeLib.h>
#include "Maint.h"
#include <EEPROM.h>

// System configuration constants
const boolean DEBUG = false;
const String PKTPREFIX = "$FH";
const String SOURCEID = "GS0";
const String PKTSUFFIX = "*";
const int TXINTERVAL = 10; // Seconds. Must match payload setting

/*
 * Digi 9XTend configured for Pin Sleep mode (SM1), 9600 Baud RF rate (BR0),
 * 2 retries (RR2), RX LED on valid addresses only (CD4), minimum "after" and "before"
 * command guard times (AT2, BT0), PAN ID 0x3426 (ID3426), and transmit power setting
 * according to: 1,2,3,4 -> 1,10,500,1000 mW
 */
const String xtPwr = "4";
const String xtConfStr = "ATSM1,BR0,RR2,CD4,AT2,BT0,ID3426,PL" + xtPwr;

// Array Sizes
const int GPSLEN = 100; //max length of GPS sentence
const int TERMLEN = 100; // Terminal input line length
const int CMDLEN = TERMLEN;
const int PKTLEN = 350;
const int SPECTLEN = 50;

// Hardware pin assignements
const int xtRssiFiltPin = A9;
const int battMonPin = A8;
const int vinMonPin = A7;
const int tempMonPin = A6;
const int xtRxLedPin = 17;
const int xtCmdPin = 16;
const int xtSleepPin = 15;
const int xtNTxPin = 6;
const int sdCsPin = 14;
const int sdCdPin = 5;
const int gpsPpsPin = 2;
const int gpsXstbyPin = 3;
const int xtRssiPwmPin = 4;

// Time sync constants
const float xtalTol = 20e-6;
const float maxTimeTol = 0.5; // seconds
const long maxTimeSyncAge = (long)( 0.5 + maxTimeTol / xtalTol );

// Measurement constants
const float rssiSlope = 10 / 0.15; // db per Duty Cycle
const float rssiIntercept = -100.0 - 0.2 * rssiSlope;
const float refVoltage = 3.3;
const int tempSensorType = 0; //LM60
//    NOTE: First terms below are empirically determined scale factors.
const float battMonToVolts = ( 8.9 / 8.7 ) * refVoltage * ( 10.0 + 47.0 ) / 10.0 / 1024;
const float vinMonToVolts = ( 5.0 / 4.9 ) * refVoltage * ( 10.0 + 10.0 ) / 10.0 / 1024;

/*
 * Set addresses of EEPROM parameters
 */
const int eeaddrStart = 256;
const int eeaddrLastGPSTimeSync = eeaddrStart; // time_t
//const int eeaddrXxxx = eeaddrLastGPSTimeSync + sizeof( time_t );


/*
 * Copernicus II power up configuration is assumed to be set to defaults except
 * the Port B rate is set at 9600 Baud and Dynamics Mode is set to "Air".
 * 
 * A configuration string is sent to set up automatic sentence output on Port B for
 * one ZDA sentence every three seconds to keep the Teensy RTC synchronized:
 * 
 *  ZDA:Provides nothing until first fix, then provides full date
 *      based on internal clock until reset. Clock synced to true
 *      UTC at prior PPS output sometime after a fully valid 2D fix.
 *      
 * Based on a PPS signal count, a "TF" position fix sentence is requested one second
 * prior to the last pps ("TXINTERVAL'th") in the PPS count cycle.
 *      
 *  TF: Provides a complete 3D position and velocity fix but no date/time.
 *      Also indicates whether time at last PPS is true UTC or GPS time.
 *      
 * After the first received payload packet, pseudo-sync will occur and the TF
 * request will be between 1 and 2 sec prior to transmitting the ground station
 * response to the payload packet. If both payload and ground have GPS
 * synchronization, The request will be 1 sec prior to transmission.
 */
const String gpsConfStr = "$PTNLSNM,0020,03*57\r\n";
const String gpsTFReqStr = "$PTNLQTF*45\r\n";

// GPS Variables
char gps[GPSLEN], gpsZDA[GPSLEN], gpsTF[GPSLEN];
int gpsIndex = 0;
boolean gpsRdy, gpsChkFlg, gpsErrFlg, gpsMsgFlg, gpsDataRdyFlg;
volatile boolean gpsPosValid, gpsZDAFlg, gpsTFFlg, gpsTimeFlg, gpsTimeValid, gpsFixReqdFlg;
byte gpsChk;
float gpsLat, gpsLon, gpsAlt, gpsVe, gpsVn, gpsVu;
int gpsYr, gpsMon, gpsDay, gpsHr, gpsMin, gpsSec, gpsFixQual, gpsNumSats;
volatile int ppsCnt;

// Receive variables
char rx[PKTLEN], rxBuf[PKTLEN];

int rxIndex;
boolean rxRdy, rxPktFlg, rxChkFlg, rxErrFlg, rxBufCurrent, rxValid;
boolean rxFlg, rxNewFlg;
volatile boolean rssiFlg;
volatile int rssiState;
elapsedMicros rssiTime;
volatile long rssiMeasTime, rssiEndTime;
float rssi;
int numb; // Received payload transmit (sync) packet sequence mumber

// Transmit variables
char tx[PKTLEN];
int txIndex = 0;
boolean txFlg, txPktFlg;

boolean procDataFlg, pseudoSyncFlg;

// File system variables
File logFile;
char logFileName[13];
boolean isLogging;
boolean outputFlg; // False suppresses debug & status output when
                   // interactive local command is active.

char cmd[CMDLEN]; // Command buffer includes arguments
int cmdIndex; // Index for command string array, cmdStrings
String cmdStr; // Current full command string (command code + args)
boolean cmdInFlg; // A command is being entered;
boolean cmdFlg; // A command has been entered and is pending
boolean remCmdFlg; // Pending command is remote if true, local if false
char cmdChar = '@';
// Command code format: L or R for Local or remote, followed by 2-char command
const int NUMCMDCODES = 1; // Number of command codes
const int CMDCODELEN = 4; // Includes extra char for null terminator

char cmdStrings[NUMCMDCODES][CMDCODELEN] =
{ "RVS" }; // Switch platform video feed


// Sensor variables
float temperature, vBatt, vIn;

// Spectrum check variables
volatile boolean xtSpectFlg, xtSpectMSFlg, xtSpectLSFlg;
boolean xtSpectRdyFlg;
volatile int xtSpectIndex;
int xtSpectData[SPECTLEN];

// Debugging variables
elapsedMillis msElapsed;
elapsedMicros usElapsed;
long pktDelivered, pktReceived;

void setup() {
  
  pinMode( xtRxLedPin, INPUT );
  pinMode( xtCmdPin, OUTPUT );
  pinMode( xtSleepPin, OUTPUT );
  pinMode( sdCsPin, OUTPUT );
  pinMode( sdCdPin, INPUT_PULLUP );
  pinMode( gpsPpsPin, INPUT );
  pinMode( gpsXstbyPin, OUTPUT );
  pinMode( xtRssiPwmPin, INPUT );
  pinMode( xtNTxPin, INPUT );
  
  digitalWrite( xtCmdPin, LOW );
  digitalWrite( xtSleepPin, LOW );
  digitalWrite( gpsXstbyPin, HIGH );
  
  gpsMsgFlg = false;
  gpsRdy = true;
  txPktFlg = false;
  rxPktFlg = false;
  rxIndex = 0;
  rssiState = 0;
  outputFlg = true;
  cmdIndex = 0;
  cmdFlg = false;
  procDataFlg = false;
  
  delay( 5000 ); //Wait for GPS to power up.
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  
  delay( 1000 ); // Give time for GPS serial TX line to bias up
  
  // Wait for internal serial ports to activate.
  //    NOTE: "Serial", the external USB port, is NOT tested as it is not a required
  //    connection and will hang when not there.
  while ( !Serial1 );
  while ( !Serial2 );
  
  
  Serial.println( "Ground Station Unit\n" );
  Serial.println( "TX Power Setting " + xtPwr + "\n" );
  
  // Configure XTend
  xtConfig( xtConfStr );
  
  // Clear out serial receive buffers
  while ( Serial.available() ) Serial.read();
  while ( Serial1.available() ) Serial1.read();
  while ( Serial2.available() ) Serial2.read();
  
  
  // Send GPS Configuration messages
  gpsConfig();
  // Get GPS response
  delay( 200 );
  Serial.println( "waiting for GPS response..." );
  char tmpChar = 0;
  while ( tmpChar != 10 ) {
    if (Serial1.available() ) {
      tmpChar = Serial1.read();
      Serial.write( tmpChar );
    }
  }
  Serial.println( "got it." );
  
  /*
   * Sync library time to RTC. Default is 5 minutes.
   * This is too long.
   */
  setSyncProvider( getTeensy3Time );
  setSyncInterval( 10 );
  
  delay( 2000 );
  if ( !MaintInit( Serial, sdCsPin, outputFlg ) ) {
    isLogging = false;
  } else {
    String logFileNameStr = timeToFilename( now() );
    logFileNameStr.toCharArray( logFileName, 13 );
    logFile = SD.open( logFileName, FILE_WRITE );
    if ( logFile ) {
      Serial.println( "Log file " + logFileNameStr + " opened. Logging enabled" );
      isLogging = true;
    } else {
      Serial.println( "Failed to open logfile. Logging disabled" );
      isLogging = false;
    }
  }
  
  if (isLogging ) {
    logFile.println( "\n====== Comm started ======\n" );
    logFile.flush();
  }
  xtSpectFlg = false;
  xtSpectIndex = -1;
  xtSpectMSFlg = false;
  xtSpectLSFlg = false;
  
  attachInterrupt( gpsPpsPin, ppsSvc, RISING );
  
  // Sample the filtered RSSI PWM signal at trailing edge of
  // RX LED indication for use in the RSSI calculation.
  //attachInterrupt( xtRxLedPin, rssiFiltMeas, FALLING );
}

void loop() {
  if ( cmdFlg ) procCmd();
  if ( gpsZDAFlg ) procZDAMsg();
  if ( gpsTFFlg ) procTFMsg();
  if ( gpsMsgFlg ) updateGPSMsg();
  if ( gpsFixReqdFlg ) requestGPSFix();
  if ( procDataFlg ) procData();
  if ( txFlg ) sendTxPkt();
  if ( txPktFlg ) makeTxPkt();
  if ( rxPktFlg ) procRxPkt();
  if ( rssiFlg ) procRssi();
  if ( Serial1.available() ) getGPSByte();
  if ( Serial2.available() ) getRXByte();
  if ( ( cmdStr = maintCheck() ) != "" ) parseCmd();
}

void parseCmd() {
  // NOTE: getField ignores 1st (command) char
  String tmpCmd = getField( cmdStr, 0, ' ' );

  
  
  
  
  
  
  
    if ( tmpCmd.length() == 3 ) {
    // Extract 3-char command string without arguments
    int tmpIndex = -1;
    // Get command index
    for ( int i = 0 ; i < NUMCMDCODES ; i++ ) {
      if ( (String)cmdStrings[i] == tmpCmd ) tmpIndex = i;
    }

    if ( tmpIndex < 0 ) {
      Serial.println( "\n'" + tmpCmd + "' is not a valid command\n" );
      cmdStr = "";
    } else {
      // Valid command received.
      Serial.println( "\n'" + tmpCmd + "' Command entered\n" );
      cmdFlg = true;
    }
  } else {
    Serial.println( "\n'" + tmpCmd + "' is not a valid command\n" );
    cmdStr = "";
  }
}

void procCmd() {

  if ( cmdStr.charAt( 1 ) == 'L' ) {
    Serial.println( "\nProcessing local command '" + cmdStr + "'\n");
    remCmdFlg = false;
    cmdFlg = false;
    cmdStr = "";
  }
  if ( cmdStr.charAt( 1 ) == 'R' ) {
    remCmdFlg = true;
  }
}

void getRXByte() {
  char c = Serial2.read();
  if ( xtSpectFlg ) {
    // Spectrum data as db below 0dBm as two hex chars converted
    // to dB above -110dBm which results in fewer bytes to transmit
    if ( xtSpectMSFlg ) {
      xtSpectData[xtSpectIndex] = 110 - ( hexDigit2Int( c ) << 4 );
      xtSpectMSFlg = false;
      xtSpectLSFlg = true;
    } else if ( xtSpectLSFlg ) {
      //Serial.print( xtSpectIndex % 10 );
      xtSpectData[xtSpectIndex] -= hexDigit2Int( c );
      if ( DEBUG && outputFlg ) {
        Serial.print( xtSpectData[xtSpectIndex] );
        Serial.write( ',' );
      }
      xtSpectLSFlg = false;
    } else if ( c == 13 ) {
      xtSpectIndex++;
      xtSpectMSFlg = true;
    }
    if ( xtSpectIndex == 50 ) {
      xtSpectFlg = false;
      xtSpectMSFlg = false;
      xtSpectLSFlg = false;
      xtSpectRdyFlg = true;
      xtSpectIndex = -1;
    }
  }
  
  if ( c == '$' ) {
    if ( outputFlg && DEBUG ) Serial.println( "\nRX msg started..." );
    rxRdy = false;
    rxIndex = 0;
    xtSpectFlg = false; // Abort spectrum data processing if it didn't finish
  } else {
    if ( c == '*' && !rxRdy ) { // End of in-process RX msg
      rxRdy = true;
      rxPktFlg = true;
      if ( outputFlg && DEBUG ) Serial.println( "\nRX msg ended." );
    }
  }

  if ( !rxRdy ) {
    rx[rxIndex] = c;
    if ( rxIndex < PKTLEN ) rxIndex++;
    rx[rxIndex] = 0;
  }
}

void getGPSByte() {
  char c = Serial1.read();
  if ( c == '$' ) { // '$', start of GPS statement - KN
    gpsIndex = 0;
    gpsRdy = false;
    gpsChk = 0;
    if ( outputFlg && DEBUG ) Serial.println( "\nGPS msg started..." );
  }
  if ( c == '*' ) { // '*' indicates end of GPS sentence
    gpsChkFlg = true;
    if ( outputFlg && DEBUG ) Serial.println( gps );
  }
  if ( !gpsRdy ) {
    if ( gpsChkFlg ) {
      gpsErrFlg = gpsChk != c;
      gpsChkFlg = false;
      gpsRdy = true;
      gpsMsgFlg = true;
    } else {
      gps[gpsIndex] = c;
      gpsChk ^= (byte)c;
    }
    gpsIndex++;
    gps[gpsIndex] = 0;
  }
}

void updateGPSMsg() {
  if ( outputFlg && DEBUG ) Serial.println( "GPS msg rcvd" );
  String str = String( gps );
  int i;
  
  if ( str.substring( 3, 6 ) == "ZDA" ) {
    for ( i = 0; i < GPSLEN ; i++ ) {
      gpsZDA[i] = gps[i];
    }
    gpsZDAFlg = true;
  }

  if ( str.substring( 5, 8 ) == "RTF" ) {
    for ( i = 0; i < GPSLEN ; i++ ) {
      gpsTF[i] = gps[i];
    }
    gpsTFFlg = true;
  }

  if ( outputFlg && DEBUG ) Serial.println( "GPS Buffer updated." );
  gpsMsgFlg = false;
  if ( outputFlg && DEBUG ) {
    Serial.print( "GPS Flags:" );Serial.print( gpsTFFlg );
    Serial.println( gpsZDAFlg );
  }
}

void procZDAMsg() {
  if ( outputFlg && DEBUG ) Serial.println( gpsZDA );
  String utc = getField( gpsZDA, 1, ',' );
  if ( utc.length() > 0 ) {
    gpsHr = utc.substring( 0, 2 ).toInt();
    gpsMin = utc.substring( 2, 4 ).toInt();
    gpsSec = utc.substring( 4, 6 ).toInt();
    gpsDay = getField( gpsZDA, 2, ',' ).toInt();
    gpsMon = getField( gpsZDA, 3, ',' ).toInt();
    gpsYr = getField( gpsZDA, 4, ',' ).toInt();
    gpsTimeFlg = true;
  }
  gpsZDAFlg = false;
}

void procTFMsg() {
  // Need UTC offset with at least 2D fix for valid time sync
  gpsTimeValid = (boolean)getField( gpsTF, 15, ',' ).toInt() &&
                 getField( gpsTF, 5, ',' ).toInt() >= 2;
  // Need 3D position fix for proper pointing.
  gpsPosValid = getField( gpsTF, 5, ',' ) == '3';
  if ( outputFlg && DEBUG ) {
    Serial.print( "gpsPosValid:" );Serial.println( gpsPosValid );
    Serial.print( "gpsTimeValid:" );Serial.println( gpsTimeValid );
  }
  // Get GPS info
  gpsFixQual = getField( gpsTF, 5, ',' ).toInt();
  gpsNumSats = getField( gpsTF, 4, ',' ).toInt();
  // Get position, velocity data
  if ( gpsPosValid ) {
    String latStr = getField( gpsTF, 6, ',' );
    gpsLat = latStr.substring( 0, 2 ).toFloat();
    gpsLat = gpsLat + latStr.substring( 2 ).toFloat() / 60.0;
    if ( getField( gpsTF, 7, ',' ) == 'S' ) gpsLat *= -1.0;
    String lonStr = getField( gpsTF, 8, ',' );
    gpsLon = lonStr.substring( 0, 3 ).toFloat();
    gpsLon = gpsLon + lonStr.substring( 3 ).toFloat() / 60.0;
    if ( getField( gpsTF, 9, ',' ) == 'W' ) gpsLon *= -1.0;
    gpsAlt = getField( gpsTF, 10, ',' ).toFloat();
    gpsVe = getField( gpsTF, 11, ',' ).toFloat();
    gpsVn = getField( gpsTF, 12, ',' ).toFloat();
    gpsVu = getField( gpsTF, 13, ',' ).toFloat();
  } else {
    gpsLat = 0.0;
    gpsLat = 0.0;
    gpsLon = 0.0;
    gpsAlt = 0.0;
    gpsVe = 0.0;
    gpsVn = 0.0;
    gpsVu = 0.0;
  }
  gpsTFFlg = false;
  gpsDataRdyFlg = true;
}

void requestGPSFix() {
  gpsSendCmd( gpsTFReqStr );
  gpsFixReqdFlg = false;
}

void makeTxPkt() {
  temperature = readTemp( tempMonPin, tempSensorType, refVoltage );
  String txPktStr =
    SOURCEID + ',' +
    String( numb ) + ',' +
    String( numb ) + ',' + // corresponds to PL last received seq number. For packet parameter symmetry only.
    String( rssi, 1 ) + ',' +
    String( now() - 1 ) + ',' +  // Position fix is 1 PPS old.
    String( temperature, 1 ) + ',' +
    String( vBatt, 1 ) + ',' +
    String( vIn, 1 ) + ',' +
    String( gpsNumSats ) + ',' +
    String( gpsFixQual ) + ',' +
    String( gpsLon, 5 ) + ',' +
    String( gpsLat, 5 ) + ',' +
    String( gpsAlt ) + ',' +
    String( gpsVe, 3 ).trim() + ',' +
    String( gpsVn, 3 ).trim() + ',' +
    String( gpsVu, 3 ).trim();

  if ( xtSpectRdyFlg ) {
    for ( int i = 0 ; i < SPECTLEN ; i++ ) {
      txPktStr += ( ',' + String( xtSpectData[i] ) );
    }
  }
  txPktStr += ',';
  txPktStr += ':';

  if ( cmdFlg && remCmdFlg ) {
    txPktStr += cmdStr;
    cmdFlg = false;

    cmdStr = "";
  }
  
  txPktStr = PKTPREFIX + txPktStr + PKTSUFFIX; // + chkStr;
  txPktStr.toCharArray( tx, PKTLEN );
  txPktFlg = false;
  txFlg = true;
  gpsDataRdyFlg = false;
  xtSpectRdyFlg = false;
}

void sendTxPkt() {
  sendData();
  rssi = 0;
  numb++;
  txFlg = false;
  procDataFlg = true;
}

void procRxPkt() {

  // Pseudo-synchronize PPS timing with Payload:
  ppsCnt = 0;
  // If no GPS fix for either payload or ground, will be no more than 1 sec early.
  // If GPS acquired in both, it will be in precise sync with payload.
  pseudoSyncFlg = true;
  if ( outputFlg ) {
    digitalClockDisplay( now() );
    Serial.println( " Received..." );
    Serial.println( rx );
  }
  if ( isLogging ) {
    logFile.print( " RX:" );
    logFile.println( rx );
    logFile.flush();
  }
  // ??? Replace with call to rssiStart with rssiState = 0 ???
  rxPktFlg = false;
  rssiState = 1; // Arm RSSI measurement
  attachInterrupt( xtRssiPwmPin, rssiStart, RISING );

  numb = getField( rx, 1, ',' ).toInt(); // Get payload packet index
  txPktFlg = true; // Trigger Ground Station TX process on Payload RX packet receipt.
}









void procRssi() {
  rssiState = 0;
  rssiFlg = false;
  rssi = calcRssi();
}

void rssiStart() {
  detachInterrupt( xtRssiPwmPin );
  if ( rssiState == 1 ) {
    rssiTime = 0; //rssiStartTime = micros();
    rssiState = 2;
    attachInterrupt( xtRssiPwmPin, rssiMeas, FALLING );
  } else {
    rssiState = 1;
    attachInterrupt( xtRssiPwmPin, rssiStart, RISING );
  }
}

void rssiMeas() {
  detachInterrupt( xtRssiPwmPin );
  if ( rssiState == 2 ) {
    rssiMeasTime = rssiTime; //micros();
    if ( rssiMeasTime > 8100 ) {
      // Missed the measure falling edge, into next pulse cycle
      rssiMeasTime = 0;
      rssiEndTime = 8000;
      rssiState = 4;
      rssiFlg = true;
    } else {
      rssiState = 3;
      attachInterrupt( xtRssiPwmPin, rssiStop, RISING );
    }
  } else {
    rssiState = 1;
    attachInterrupt( xtRssiPwmPin, rssiStart, RISING );
  }
}

void rssiStop() {
  detachInterrupt( xtRssiPwmPin );
  if ( rssiState == 3 ) {
    rssiEndTime = rssiTime;
    rssiState = 4;
    rssiFlg = true;
  } else {
    rssiState = 1;
    attachInterrupt( xtRssiPwmPin, rssiStart, RISING );
  }
}

float calcRssi() {
  float dutyCycle = (float)( rssiMeasTime ) / (float)( rssiEndTime );
  return rssiIntercept + rssiSlope * dutyCycle;
}

String getField( String str, int fieldIndex, char delim ) {
  int startIndex = -1;
  int endIndex = 0;
  for ( int i = 0 ; i <= fieldIndex ; i++ ) {
    startIndex = endIndex + 1;
    endIndex = str.indexOf( delim, startIndex );
  }
  if ( endIndex == -1 ) endIndex = str.length();
  return str.substring( startIndex, endIndex );
}

void sendData() {
  attachInterrupt( xtNTxPin, measV, FALLING );
  int i = 0;
  while ( tx[i] != 0 ) {
    Serial2.write( tx[i] );
    i++;
  }
  Serial2.write( 10 );
  if ( outputFlg ) {
    digitalClockDisplay( now() );
    Serial.println( " Sending..." );
    Serial.println( tx );
  }
  if ( isLogging ) {
    logFile.print( " TX:" );
    logFile.println( tx );
    logFile.flush();
  }
}

void measV() {
  detachInterrupt( xtNTxPin );
  vBatt = analogRead( battMonPin ) * battMonToVolts;
  vIn = analogRead( vinMonPin ) * vinMonToVolts;
}

time_t getTeensy3Time() {
  return Teensy3Clock.get();
}

void digitalClockDisplay(time_t t) {
  // digital clock display of a time_t value
  int hr = hour(t);
  if ( hr < 10 ) Serial.print( '0' );
  Serial.print(hr);
  printDigits(minute(t));
  printDigits(second(t));
  Serial.print(" ");
  Serial.print(day(t));
  Serial.print(" ");
  Serial.print(month(t));
  Serial.print(" ");
  Serial.print(year(t));
}

void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10) Serial.print('0');
  Serial.print(digits);
}

String timeToFilename( time_t t ) {
  char timeStr[15];
  sprintf( timeStr, "%04i%02i%02i%02i%02i%02i",
           year(t), month(t), day(t), hour(t), minute(t), second(t) );
  String tempStr = (String)timeStr;
  String tempStr2 = tempStr.substring( 2, 10 ) + "." + tempStr.substring( 10, 13 );
  return String( tempStr2 );
}

void gpsConfig() {
  Serial1.print( gpsConfStr );
}

void gpsSendCmd( String cmd ) {
  Serial1.print( cmd );
}

void ppsSvc() {
  long tmpMicros;
  if ( outputFlg && DEBUG ) {
    tmpMicros = micros();
  }
  
  // Set clock(s) to UTC time if a valid fix came in since the last PPS
  if ( outputFlg && DEBUG ) digitalClockDisplay( now() );
  if ( gpsTimeValid && gpsTimeFlg ) {
    time_t oldTime = getTeensy3Time();
    setTime( gpsHr, gpsMin, gpsSec, gpsDay, gpsMon, gpsYr ); // "Library" time
    time_t newTime = now() + 1;
    setTime( newTime );
    Teensy3Clock.set( newTime ); // Teensy RTC
    time_t temp;
    EEPROM.get(  eeaddrLastGPSTimeSync, temp );
    if ( ( abs( oldTime - newTime ) >= 1 || 
         ( newTime - temp ) >= maxTimeSyncAge ) ) {
      EEPROM.put( eeaddrLastGPSTimeSync, newTime );
      if ( outputFlg && DEBUG ) {
        Serial.print( "Time Corrected:" );
      }
    }
  }

  // Reset GPS flags
  gpsPosValid = false;
  gpsTimeValid = false;
  gpsTimeFlg = false;
  gpsZDAFlg = false;
  gpsTFFlg = false;







  if ( ppsCnt == TXINTERVAL - 2 ) {


    gpsFixReqdFlg = true;
    if ( outputFlg && DEBUG ) Serial.println( "Getting GPS fix..." );

    xtCommand( "ATRM" );
    xtSpectFlg = true;
    xtSpectIndex = -1;
    xtSpectMSFlg = false;
    xtSpectLSFlg = false;
  }




  ppsCnt++;
  if ( ppsCnt == TXINTERVAL ) ppsCnt = 0;
  if ( outputFlg && DEBUG ) {
    Serial.print( " ISR duration: " );
    Serial.println( micros() - tmpMicros );
  }
}

String checkStr( String str ) {
  
  char buf[80]; // Max length of NMEA excl. '$',CR,LF = 79, + null
  str.toCharArray(buf, 80);
  byte check = 0x00;
  for ( unsigned int i = 0 ; i < str.length() ; i++ ) {
    check ^= (byte)buf[i];
  }
  String chkStr = String( check, HEX );
  if ( check < 0x10 ) {
    chkStr = '0' + chkStr;
  }
  chkStr.toUpperCase();
  return chkStr;
}

void xtConfig( String configStr ) {
  Serial.print( "Configuring XTend... " );
  Serial2.print("+++");
  delay( 1100 );
  Serial2.println( configStr );
  Serial2.println( "ATCN" );
  delay( 1000 );
  while ( Serial2.available() ) Serial.write( Serial2.read() );
  Serial.println( "" );
}

/*
 * This is a function you can add to read the temperature
 * from one of several devices:
 * 
 * Arguments:
 * pin (int) Analog pin number attached to sensor.
 * sensType (int) Selects one of three sensor types.
 * vRef (float) The arduino's supply voltage (5.0 for UNO)
 */

float readTemp( int pin, int sensType, float vRef ) {
  // sensType parameter vs. Temperature Sensor type:
  //   0  LM60
  //   1  MAX6605
  //   2  TMP36
  int mVoltsAtRefTemp[] = { 424, 744, 750 };
  int refTempC[] = { 0, 0, 25 };
  float mVperDegC[] = { 6.25, 11.9, 10.0 };

  int reading = analogRead( pin );
  //Serial.print( "Vtemp:" );Serial.println( reading );
  float mVolts = reading * vRef / 1.024;

  return ( ( mVolts - mVoltsAtRefTemp[sensType] ) / 
           ( mVperDegC[sensType] ) + 
             refTempC[sensType]);
  
}

void xtCommand( String xtCmd ) {
  Serial2.print( "+++" );
  delay( 210 );
  Serial2.println( xtCmd );
  Serial2.println( "ATCN" );
}

int hexDigit2Int( char digit ) {
  if ( digit >= 48 && digit <= 57 ) {
    return digit - 48;
  } else if ( digit >= 65 && digit <= 70 ) {
    return digit - 65 + 10; 
  } else if ( digit >= 97 && digit <= 102 ) {
    return digit - 97 + 10;
  } else {
    return 0;
  }
}

/*
 * If maxVal = 0, autoscale using next multiple of 10 higher than max value.
 */
void displaySpectrum( int spect[], int maxVal, int increment, int len ) {
  int scale = maxVal;
  if ( scale == 0 ) {
    for ( int i = 0 ; i < len ; i++ ) if ( spect[i] > scale ) {
      scale = 10 * floor( spect[i] / 10 ) + 10;
    }
  }
  for ( int row = scale ; row >= 0 ; row -= increment ) {
    Serial.write( '\n' );
    if ( row < 10 ) Serial.write( ' ' );
    if ( row < 100 ) Serial.write( ' ' );
    Serial.print( row );
    Serial.write( '|');
    for ( int col = 0 ; col < len ; col++ ) {
      if ( spect[col] > row - increment / 2 ) {
        Serial.write( '*' );
      } else {
        Serial.write( ' ' );
      }
    }
  }
  Serial.write( '\n' );
}

void procData() {
  float deSense = 0;
  for ( int i = 0 ; i < SPECTLEN ; i++ ) {
    xtSpectData[i] = getField( rx, 16 + i, ',' ).toInt();
    deSense += pow( 10, xtSpectData[i] / 10.0 ) - 1;
  }
  deSense = 10 * log10( deSense );
  if ( outputFlg ) {
    displaySpectrum( xtSpectData, 0, 2, 50 );
    Serial.print( "De-Sense = " ); Serial.print( deSense, 1 );
    Serial.println( " dB" );
  }
  procDataFlg = false;
}

