/*
 * Arduino IDE Board Selection: Teensy 3.1 or 3.2
 * Device: MK20DX256VLH7 (NXP K20P64M72SF1 Spec)
 * CPU Speed: 96MHz (optimized) Overclock
 * 
 * RTC: Vbat 1.71V min @ 1uA
 */

/*
 * Serial:  External Serial Monitor through USB
 * Serial1: GPS
 * Serial2: RF data IN/OUT
 */

#include <TimeLib.h>
#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>

// System configuration constants
const boolean DEBUG = false;
const String PKTPREFIX = "$FH";
const String SOURCEID = "GS0";
const String PKTSUFFIX = "*";
const int TXINTERVAL = 10; // Seconds. Must match payload setting

// Array Sizes
const int GPSLEN = 100; //max length of GPS sentence
const int TERMLEN = 100;
const int CMDLEN = TERMLEN;
const int PKTLEN = 200;

// Hardware pin assignements
const int teensyLedPin = 13;
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
 * one ZDA sentence every two seconds for time synchronization:
 * 
 *  ZDA:Provides nothing until first f ix, then provides full date
 *      based on internal clock until reset. Clock synced to true
 *      UTC at prior PPS output sometime after a fully valid 2D fix.
 *      
 * Based on a PPS signal count, a "TF" position fix sentence is requested 1 sec
 * prior to the last pps ("TXINTERVAL'th") in the PPS count cycle.
 *      : 
 *  TF: Provides a complete 3D position and velocity fix but no date/time. 
 *      Also indicates whether time at last PPS is true UTC or GPS time.    
 *      
 * After the first received payload packet, pseudo-sync will occur and the TF
 * request will be between 1 and 2 sec prior to transmitting the ground station
 * response to the payload packet. If both payload and ground have GPS
 * synchronization, The request will be 1 sec prior to transmission.*/
const String gpsConfStr = "$PTNLSNM,0020,03*57\r\n";
const String gpsTFReqStr = "$PTNLQTF*45\r\n";

/*
 * Digi 9XTend configured for Pin Sleep mode, 9600 Baud RF rate, 2 retries, RX LED on
 * valid addresses only.
 */

const String xtConfStr = "ATSM1,BR0,RR2,CD4,PL0";

// GPS variables
char gps[GPSLEN], gpsBuf[GPSLEN], gpsZDA[GPSLEN], gpsTF[GPSLEN]; 
int gpsIndex = 0;
boolean gpsRdy, gpsZDAFlg, gpsTFFlg, gpsChkFlg, gpsErrFlg, gpsFixFlg;
boolean gpsPosValid, gpsMsgFlg, gpsTimeFlg, gpsTimeValid;
byte gpsChk;
float gpsLat, gpsLon, gpsAlt, gpsVe, gpsVn, gpsVu;
int gpsYr, gpsMon, gpsDay, gpsHr, gpsMin, gpsSec, gpsFixQual;
int ppsCnt;

// Receive variables
char rx[PKTLEN], rxBuf[PKTLEN];
int rxIndex;
boolean rxRdy, rxPktFlg, rxChkFlg, rxErrFlg, rxBufCurrent, rxValid;
boolean rxFlg, rxNewFlg;
volatile int rssiState;
elapsedMicros rssiTime;
volatile long rssiMeasTime, rssiEndTime;
float rssi;
int numb;

// Transmit variables
char tx[PKTLEN];
int txIndex = 0;
boolean txFlg, txPktFlg;

// File system variables
File logFile;
char logFileName[13];
boolean isLogging;

// Sensor variables
float temperature, vBatt, vIn;

void setup(){

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
  
  delay( 5000 ); //Wait for GPS to power up.
  Serial.begin(9600);
  Serial1.begin(9600);  
  Serial2.begin(9600); 
  delay( 1000 ); // Give time for GPS serial TX line to bias up

  // Wait for internal serial ports to activate.
  //    NOTE: "Serial", the external USB port, is NOT tested as it is not a required
  //    connection and will hang when not there.
  while( !Serial1 );
  while( !Serial2 );

  Serial.println( "Ground Station Unit\n" );

  // Configure XTend
  xtConfig( xtConfStr );

  // Clear out serial receive buffers
  while( Serial.available() ) Serial.read();
  while( Serial1.available() ) Serial1.read();
  while( Serial2.available() ) Serial2.read();

  
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
  if (!SD.begin( sdCsPin )) {
    Serial.println("Card failed, or not present");
    Serial.println( "SD Card failed or not present. Logging disabled" );
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
  IntervalTimer rssiTimer;
  attachInterrupt( gpsPpsPin, ppsSvc, RISING );
}

void loop(){
  if ( gpsMsgFlg ) updateGPSMsg();
  if ( gpsZDAFlg ) procZDAMsg();
  if ( gpsTFFlg ) procTFMsg();
  if ( gpsFixFlg ) {
    gpsSendCmd( gpsTFReqStr );
    gpsFixFlg = false;
  }
  if ( Serial1.available() ) getGPSByte();
  if ( Serial2.available() ) getRXByte();
  if ( rxPktFlg ) procRxPkt();
  if ( rssiState == 1 ) digitalWrite( teensyLedPin, HIGH );
  if ( rssiState == 2 ) digitalWrite( teensyLedPin, LOW );
  if ( rssiState == 3 ) digitalWrite( teensyLedPin, HIGH );
  if ( rssiState == 4 ) {
    digitalWrite( teensyLedPin, LOW );
    rssi = calcRssi();
    rssiState = 0;
  }
  if ( txPktFlg ) makeTxPkt();
  if ( txFlg ) {
    sendData();
    txFlg = false;
    rssi = 0;
  }
}

void getRXByte() {
  // "First Draft" packet acquisition based on simple start and end chars.
  //    No checksum check
  char c = Serial2.read();
  if ( c == '$' || rxRdy ) {
    if ( DEBUG ) Serial.println( "\nRX msg started..." );
    rxRdy = false;
    rxIndex = 0;
  } else {
    if ( c == '*' && !rxRdy ) { // End of in-process RX msg
      rxRdy = true;
      rxPktFlg = true;
      if ( DEBUG ) Serial.println( "\nRX msg ended." );
    } 
  }

  if ( !rxRdy ) {
    rx[rxIndex] = c;
    rxIndex++;
    rx[rxIndex] = 0;
  }
}

void getGPSByte() {
  char c = Serial1.read();
  if ( c == '$' ) { // '$', start of GPS statement - KN
    gpsIndex = 0;
    gpsRdy = false;
    gpsChk = 0;
    if ( DEBUG ) Serial.println( "\nGPS msg started..." );
  }
  if ( c == '*' ) { // '*' indicates end of GPS sentence
    gpsChkFlg = true;
    if ( DEBUG ) Serial.println( gps );
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
  if ( DEBUG ) Serial.println( "GPS msg rcvd" );
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

  for ( int i = 0 ; i < GPSLEN ; i++ ) {
      gpsBuf[i] = gps[i];
  }
  if ( DEBUG ) Serial.println( "GPS Buffer updated." );
  gpsMsgFlg = false;
  if ( DEBUG ) {
    Serial.print( "GPS Flags:" );Serial.print( gpsTFFlg );
    Serial.println( gpsZDAFlg );
  }
}

void procZDAMsg() {
  if ( DEBUG ) Serial.println( gpsZDA );
  String utc = getField( gpsZDA, 1, ',' );
  if ( utc.length() > 0 ) {
    gpsHr = utc.substring( 0,2 ).toInt();
    gpsMin = utc.substring( 2,4 ).toInt();
    gpsSec = utc.substring( 4,6 ).toInt();
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
  if ( DEBUG ) {
    Serial.print( "gpsPosValid:" );Serial.println( gpsPosValid );
    Serial.print( "gpsTimeValid:" );Serial.println( gpsTimeValid );
  }
  if ( gpsPosValid ) {
    // Get GPS info
    gpsFixQual = getField( gpsTF, 5, ',' ).toInt();
    // Get position, velocity data
    String latStr = getField( gpsTF, 6, ',' );
    gpsLat = latStr.substring( 0,2 ).toFloat();      
    gpsLat = gpsLat + latStr.substring( 2 ).toFloat() / 60.0;
    if ( getField( gpsTF, 7, ',' ) == 'S' ) gpsLat *= -1.0;
    String lonStr = getField( gpsTF, 8, ',' );
    gpsLon = lonStr.substring( 0,3 ).toFloat();
    gpsLon = gpsLon + lonStr.substring( 3 ).toFloat() / 60.0;
    if ( getField( gpsTF, 9, ',' ) == 'W' ) gpsLon *= -1.0;
    gpsAlt = getField( gpsTF, 10, ',' ).toFloat();
    gpsVe = getField( gpsTF, 11, ',' ).toFloat();
    gpsVn = getField( gpsTF, 12, ',' ).toFloat();
    gpsVu = getField( gpsTF, 13, ',' ).toFloat();
  }
  gpsTFFlg = false;
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
    String( gpsFixQual ) + ',' +
    String( gpsLon,5 ) + ',' +
    String( gpsLat,5 ) + ',' +
    String( gpsAlt ) + ',' +
    String( gpsVe, 3 ).trim() + ',' +
    String( gpsVn, 3 ).trim() + ',' +
    String( gpsVu, 3 ).trim();
    byte chkSum = 0;
    int txLen = txPktStr.length();
    for ( int i = 0 ; i < txLen ; i++ ) {
      chkSum ^=(byte)txPktStr.charAt(i);
    }
    String chkStr = String( chkSum, HEX );
    if ( chkStr.length() == 1 ) chkStr = '0' + chkStr;
    txPktStr = PKTPREFIX + txPktStr + PKTSUFFIX + chkStr;
    txPktStr.toCharArray( tx, PKTLEN );
    txPktFlg = false;
    txFlg = true;
}

void procRxPkt() {

  // Pseudo-synchronize PPS timing with Payload.
  // If no GPS fix for either payload or ground, will be no more than 1 sec early.
  // If GPS acquired in both, it will be in precise sync with payload.
  ppsCnt = 0;
  
  digitalClockDisplay( now() );Serial.println( " Received..." );  
  Serial.println( rx );
  if ( isLogging ) {
    logFile.print( " RX:" );
    logFile.println( rx );
    logFile.flush();
  }
  numb = getField( rx, 1, ',' ).toInt();
  rxPktFlg = false;
  txPktFlg = true;
  //rssi = 0; // Erase any prior reading - doesn't work on GS unit (???)
  rssiState = 1; // Arm RSSI measurement
  attachInterrupt( xtRssiPwmPin, rssiStart, RISING );
}

void rssiStart() {
  detachInterrupt( xtRssiPwmPin );
  if ( rssiState == 1 ) {
    rssiTime = 0; //rssiStartTime = micros();
    rssiState++;
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
    } else {
      rssiState++;
      attachInterrupt( xtRssiPwmPin, rssiStop, RISING );
    }
  } else {
    rssiState = 1;
    attachInterrupt( xtRssiPwmPin, rssiStart, RISING );
  }
}

void rssiStop() {
  detachInterrupt( xtRssiPwmPin );
  if ( rssiState == 3 ){
    rssiEndTime = rssiTime;
    rssiState++;
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
  while( tx[i] != 0 ) {
    printAndSendChar( tx[i] );
    i++;
  }
  printAndSendChar( 10 );

  digitalClockDisplay( now() );Serial.println( " Sending..." );
  Serial.println( tx );
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

void printAndSendString( String whatToSend ){ //sends message to both terminal & rf module
  Serial2.print( whatToSend ); // RF module
//  Serial.print( whatToSend ); // Terminal
//  if ( isLogging ) logFile.print( whatToSend ); // Log file
}

void printAndSendChar( char whatToSend ){ //sends message to both terminal & rf module
  Serial2.write( whatToSend ); // RF module
//  Serial.write( whatToSend ); // Terminal
//  if ( isLogging ) logFile.write( whatToSend ); // Log file
}

time_t getTeensy3Time() {
  return Teensy3Clock.get();
}

void digitalClockDisplay(time_t t) {
  // digital clock display of the time
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

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10) Serial.print('0');
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
  if ( DEBUG ) {
    tmpMicros = micros();
  }
  
  // Set clock(s) to UTC time if a valid fix came in since the last PPS
  if ( DEBUG ) digitalClockDisplay( now() );
  if ( gpsTimeValid && gpsTimeFlg ) {
    time_t oldTime = getTeensy3Time();
    setTime( gpsHr,gpsMin,gpsSec,gpsDay,gpsMon,gpsYr ); // "Library" time    
    time_t newTime = now() + 1;
    setTime( newTime );
    Teensy3Clock.set( newTime ); // Teensy RTC
    time_t temp;
    EEPROM.get(  eeaddrLastGPSTimeSync, temp );
    if ( ( abs( oldTime - newTime ) >= 1 || 
         ( newTime - temp ) >= maxTimeSyncAge ) ) {
      EEPROM.put( eeaddrLastGPSTimeSync, newTime );
      if ( DEBUG ) {
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
    gpsFixFlg = true;
    if ( DEBUG ) Serial.println( "Getting GPS fix..." );
  }
  ppsCnt++;
  if ( ppsCnt == TXINTERVAL ) ppsCnt = 0;
  if ( DEBUG ) {
    Serial.print( " ISR duration: " );
    Serial.println( micros() - tmpMicros );
  }
}

String checkStr( String str ) {
  
  char buf[80]; // Max length of NMEA excl. '$',CR,LF = 79, + null
  str.toCharArray(buf, 80);
  byte check = 0x00;
  for ( int i = 0 ; i < str.length() ; i++ ) {
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
  Serial2.println( xtConfStr );
  Serial2.println( "ATCN" );
  delay( 1000 );
  while( Serial2.available() ) Serial.write( Serial2.read() );
  Serial.println( "" );
}


/*
  This is a function you can add to read the temperature
  from one of several devices:
  
  Arguments:
  pin (int) Analog pin number attached to sensor.
  sensType (int) Selects one of three sensor types.
  vRef (float) The arduino's supply voltage (5.0 for UNO)
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

  return( ( mVolts - mVoltsAtRefTemp[sensType] ) / 
            ( mVperDegC[sensType] ) + 
            refTempC[sensType]);
  
}
