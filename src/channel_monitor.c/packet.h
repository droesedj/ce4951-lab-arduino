/*
   LAB WEEK 9: Data header + CRC
   CE4951-012
   @author Dennis Droese
   @filename packet.h
   @date 10/28/2018
*/

#ifndef packet_h
#define packet_h

#include "Arduino.h"
#include "transmitter.h"

class Packet {
  private:

    /// Sync data expected by all network devices at the start of a packet.
    byte mSynch = 0x55;
    /// Version identifier of the network packet
    byte mVer = 0x01;
    /// Address of the device where the packet originates.
    byte mSource;
    /// Address of the device where the packet must be delivered to.
    byte mDestination;
    /// Flag for whether or not a CRC is present.
    byte f_CRC = 0x01;
    /// Size of the data to be transmitted (in bytes)
    byte mLength;
    /// Data to be sent with the packet.
    byte mData[255];
    /// CRC8-CCITT frame check sequence trailer.
    byte mCRC8;

  public:

    bool CRC8_valid;

    /// Constructor.  Used to create a packet from raw data.
    Packet(byte* data, int len, byte source, byte dest);

    /// Constructor used to decode a recieved packet.
    Packet(byte* data, int len);

    /// Sends the packet over the transmitter.
    // Returns true if successful.
    bool Transmit(Transmitter trans);

    /// Gets the data component of the packet in c-string form.  NOT NULL TERMINATED!!!
    String GetDataString();
    /// Returns a string summary of the packet.
    String GetSummary();
};

#endif
