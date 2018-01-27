#! /usr/bin/env ruby
#
# xsftp client terminal
# Based on miniterm.rb
#
# ruby 2.0.0+ environment
# serialport required (gem install serialport)

# NOTES:
#   Get Serial Port Listings:
#      Windows: wmic path Win32_SerialPort
#      Linux: 
#      OSX:

require 'serialport'


# Main Starts here

@fileXferFlg = false
@getReqFlg = false
@outFile = nil
@outFileName = nil
@eolFlg = false
@msg = ""
@byteCnt = 0

if ARGV.size < 4
  STDERR.print <<EOF
  Usage: ruby #{$0} portpath bps nbits stopb
EOF
  exit(1)
end

sp = SerialPort.new(ARGV[0], ARGV[1].to_i, ARGV[2].to_i, ARGV[3].to_i, SerialPort::NONE)
sp.read_timeout = 0
open("/dev/tty", "r+") { |tty|
  tty.sync = true
  Thread.new {
    while true do
      inByte = sp.getbyte() 
      # Abort file transfer if first byte is zero (file doesn't exist)
      # otherwise, create the new file and proceed with transfer
      if @getReqFlg
        if inByte == 0
          @fileXferFlg = false
        else
          @outFile = File.open( @outFileName, "w" )
          inByte = sp.getbyte()
          @numBytes = 0
          @byteCnt = 0
          @fileXferFlg = true
        end
        @getReqFlg = false
      end

      if @fileXferFlg
        #tty.printf( "%i,%i\n", inByte, @byteCnt )
        if @byteCnt < 4
          @numBytes = 256 * @numBytes + inByte #First 4 bytes are file size
        else
          if @byteCnt == 4
            tty.printf( "\nTransferring %i bytes...", @numBytes)
          end
          if @numBytes > 0
            @outFile.putc( inByte )
          end
          #tty.printf("%2i %2x", @byteCnt - 4, inByte )          
          if @numBytes == 0 || @byteCnt == @numBytes + 3  #On final byte, terminate transfer
            @outFile.close()
            @fileXferFlg = false
            tty.printf( "%s", "done\n" )
          end
        end
        @byteCnt += 1
      else
        tty.printf("%c", inByte)
      end
    end
  }
  while (l = tty.gets) do
    sp.write(l.sub("\n", "\r"))
    if l.split( " " )[0] == "get" 
      @getReqFlg = true
      @outFileName = l.chomp.split( " " )[1]
    end
    if l.split( " " )[0] == "exit"
      exit(1)
    end
  end
}
