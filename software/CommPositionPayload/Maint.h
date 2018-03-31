#include <SPI.h>
#include <SD.h>

File _sdRoot, _sdFile;
String _maintCmdStr;
boolean _maintCmdFlg, _maintLineEndFlg, _maintFtpModeFlg;
boolean _maintFtpGetFlg, _maintEchoFlg, _maintModeFlg;
boolean * _outputEnableFlgAddr, _outputEnableFlgBackup;

usb_serial_class _port;

/*
 * Header functions have prototypes to make them global so they can call other header functions.
 */
boolean MaintInit( usb_serial_class*, int, boolean* );
String maintCheck();
void _getMaintByte();
String _procMaintCmd();
void _getMaintByte();
void _getFileBlock();
String _getField2( String, int, char );
void _listDirectory( File, int );

/*
 * The serial port and outputEnableFlg variables are passed by reference so that
 * the header functions herein can access these global variables as they would if
 * in the main program.
 */
boolean MaintInit( usb_serial_class &port, int sdcs, boolean &outputEnableFlg ) {
  _port = port; // For global access;
  _outputEnableFlgAddr = &outputEnableFlg;
  _maintEchoFlg = true;
  boolean _sdValidFlg;
  _port.print( "Initializing SD card..." );

  if ( !SD.begin( sdcs ) ) {
    _port.println( "initialization failed" );
    _port.println( " NO SD Card or bad configuration " );
    _sdValidFlg = false;
  } else {
    _port.println( "done" );
    _sdValidFlg = true;
  }
  return _sdValidFlg;
}

String maintCheck() {
  String tmpStr = ""; 
  if ( _maintFtpGetFlg ) _getFileBlock();
  if ( _maintCmdFlg ) tmpStr = _procMaintCmd();
  if ( _port.available() ) _getMaintByte();
  return tmpStr.toUpperCase();
}

void _getMaintByte() {
  char inByte = _port.read();
  if ( !_maintCmdFlg ) {
    if ( inByte == '\r' || inByte == '\n' ) {
      if ( _maintLineEndFlg ) {
        _maintLineEndFlg = false;
      } else {
        _maintLineEndFlg = true;
        _maintCmdFlg = ( _maintCmdStr != "" );
        //_port.write( '\n' );
      }
    } else {
      _maintCmdStr += inByte;
      _maintLineEndFlg = false;
    }
  }
  if ( _maintEchoFlg && _maintModeFlg ) {
    _port.write( inByte );
  }
}

/*
 * Processes _maintCmdStr. Checks for built-in commands, processes them,
 * and returns the null String. If not built-in, the unprocessed command
 * string is returned.
 */
String _procMaintCmd() {
  String remCode = _getField2( _maintCmdStr, 0, ' ' );
  String args = _getField2( _maintCmdStr, 1, ' ' );
  char tmp[32];
  args.toCharArray( tmp, 32 );
  _maintCmdFlg = false;
  String tmpStr = _maintCmdStr;
  _maintCmdStr = "";
  remCode.toUpperCase();
  if ( _maintModeFlg ) {
    if ( _maintFtpModeFlg ) {
      tmpStr = "";
      if ( remCode == "LS" ) {
        _port.print("\n");
        _listDirectory( _sdRoot, 0 );
        _port.print( "\nFTP> " );
  //    } else if ( remCode == "CD" ) {
  //      if ( SD.open( tmp ).isDirectory() ) {
  //        _sdRoot = SD.open( tmp );
  //      } else {
  //        _port.println( "" );_port.print( tmp );_port.println( "is not a directory." );
  //      }
  //      _port.print( "\nFTP> " );
      } else if ( remCode == "BYE" ) {
        _port.print( "\nLeaving FTP Mode...\n\n>" );
        _maintFtpModeFlg = false;
      } else if ( remCode == "GET" ) {
        _sdFile = SD.open( tmp );
        if ( _sdFile ) {
          unsigned long fileSize = _sdFile.size();
          _port.write( 1 ); // Send file exists flag to client
          for ( int i = 0 ; i < 4 ; i++ ) {
            _port.write( (byte)( ( fileSize >> ( 8 * (3 - i) ) ) & 255 ) );
          }
          _maintFtpGetFlg = true;
        } else {
          _port.write( 0 ); // Send file not found flag to client
          _port.print( "\nCan't open file :" ); _port.println( tmp );
          _port.print( "\nFTP> " );
        }
      } else {      
        _port.print( "\nFTP> " );
      }
    } else {
  
    //if ( !_maintFtpModeFlg ) {
      if ( remCode == "FTP" ) {
        _port.println( "\n\nEntering FTP Mode..." );
        _maintFtpModeFlg = true;
        _sdRoot = SD.open("/");
        _port.print( "\nFTP> " );
        tmpStr = "";
      } else if ( remCode == "ECHO" ) {
        if ( strcmp( tmp, "on" ) == 0 ) _maintEchoFlg = true;
        if ( strcmp( tmp, "off" ) == 0 ) _maintEchoFlg = false;
        _port.print( "\n> " );
        tmpStr = "";
      } else if ( remCode == "EXIT" ) {
        _maintModeFlg = false;
        (*_outputEnableFlgAddr) = true;
        _maintEchoFlg = false;
        _port.println( "\nLeaving maintenance mode" );
        tmpStr = "";
      } else {
        _port.print( "\n> " );
      }
    }
  } else if ( remCode == "CMD" ) {
    _maintModeFlg = true;
    (*_outputEnableFlgAddr) = false;
    _port.println( "\nEntering maintenance mode" );
    _port.print( "\n> " );
    tmpStr = "";
  } else {
    tmpStr = "";
  }
  return tmpStr;
}

void _listDirectory(File dir, int numTabs) {
  while (true) {

    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      _port.print('\t');
    }
    _port.print(entry.name());
    if (entry.isDirectory()) {
      _port.println("/");
    } else {
      // files have sizes, directories do not
      _port.print("\t\t");
      _port.println(entry.size(), DEC);
    }
    entry.close();
  }
  dir.rewindDirectory();
}

void _getFileBlock() {
  for ( int i = 0 ; i < 512 ; i++ ) {
    if ( _sdFile.available() ) {
      _port.write( _sdFile.read() );
    } else {
      _maintFtpGetFlg = false;
      _sdFile.close();
      _port.print( "\nFTP> " );
      break;
    }
  }
}

String _getField2( String str, int fieldIndex, char delim ) {
  int startIndex = 0;
  int endIndex = -1;
  for ( int i = 0 ; i <= fieldIndex ; i++ ) {
    startIndex = endIndex + 1;
    endIndex = str.indexOf( delim, startIndex );
  }
  if ( endIndex == -1 ) endIndex = str.length();
  return str.substring( startIndex, endIndex );
}
