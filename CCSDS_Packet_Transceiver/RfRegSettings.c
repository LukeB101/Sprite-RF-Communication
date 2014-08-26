#ifndef RF_SETTINGS_H
#define RF_SETTINGS_H

                          /*## Choose here which frquency to transmit on depending on what local laws are. ##*/
#define MHZ434            //434MHz  
//#define MHZ915            //915MHz
//#define CCSDS_PROX1_CHANNEL1_435.6MHZ_FORWARD_LINK
//#define CCSDS_PROX1_CHANNEL1_404.4MHZ_RETURN_LINK


#include "RF1A.h"


#ifdef MHZ915
// Chipcon
// Product = CC430Fx13x
// Chip version = C   (PG 0.7)
// Crystal accuracy = 10 ppm
// X-tal frequency = 26 MHz
// RF output power = 0 dBm
// RX filterbandwidth = 101.562500 kHz
// Deviation = 19 kHz
// Datarate = 38.383484 kBaud
// Modulation = (1) GFSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 914.999969 MHz
// Channel spacing = 199.951172 kHz
// Channel number = 0
// Optimization = -
// Sync mode = (3) 30/32 sync word bits detected
// Format of RX/TX data = (0) Normal mode, use FIFOs for RX and TX
// CRC operation = (1) CRC calculation in TX and CRC check in RX enabled
// Forward Error Correction = 
// Length configuration = (0) Fixed packet length, packet length configured by PKTLEN
// Packetlength = 61
// Preamble count = (2)  4 bytes
// Append status = 1
// Address check = (0) No address check
// FIFO autoflush = 0
// Device address = 0
// GDO0 signal selection = ( 6) Asserts when sync word has been sent / received, and de-asserts at the end of the packet
// GDO2 signal selection = (41) RF_RDY
RF_SETTINGS rfSettings = {
    0x08,   // FSCTRL1   Frequency synthesizer control.
    0x00,   // FSCTRL0   Frequency synthesizer control.
    0x23,   // FREQ2     Frequency control word, high byte.
    0x31,   // FREQ1     Frequency control word, middle byte.
    0x3B,   // FREQ0     Frequency control word, low byte.
    0xCA,   // MDMCFG4   Modem configuration.
    0x83,   // MDMCFG3   Modem configuration.
    0x93,   // MDMCFG2   Modem configuration.
    0x22,   // MDMCFG1   Modem configuration.
    0xF8,   // MDMCFG0   Modem configuration.
    0x00,   // CHANNR    Channel number.
    0x34,   // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
    0x56,   // FREND1    Front end RX configuration.
    0x10,   // FREND0    Front end TX configuration.
    0x18,   // MCSM0     Main Radio Control State Machine configuration.
    0x16,   // FOCCFG    Frequency Offset Compensation Configuration.
    0x6C,   // BSCFG     Bit synchronization Configuration.
    0x43,   // AGCCTRL2  AGC control.
    0x40,   // AGCCTRL1  AGC control.
    0x91,   // AGCCTRL0  AGC control.
    0xE9,   // FSCAL3    Frequency synthesizer calibration.
    0x2A,   // FSCAL2    Frequency synthesizer calibration.
    0x00,   // FSCAL1    Frequency synthesizer calibration.
    0x1F,   // FSCAL0    Frequency synthesizer calibration.
    0x59,   // FSTEST    Frequency synthesizer calibration.
    0x81,   // TEST2     Various test settings.
    0x35,   // TEST1     Various test settings.
    0x09,   // TEST0     Various test settings.
    0x47,   // FIFOTHR   RXFIFO and TXFIFO thresholds.
    0x29,   // IOCFG2    GDO2 output pin configuration.
    0x06,   // IOCFG0    GDO0 output pin configuration. Refer to SmartRF� Studio User Manual for detailed pseudo register explanation.
    0x04,   // PKTCTRL1  Packet automation control.
    0x05,   // PKTCTRL0  Packet automation control.
    0x00,   // ADDR      Device address.
    0x78    // PKTLEN    Packet length.
};

#endif



#ifdef MHZ434
// Chipcon
// Product = CC430Fx13x
// Chip version = C   (PG 0.7)
// Crystal accuracy = 10 ppm
// X-tal frequency = 26 MHz
// RF output power = 0 dBm
// RX filterbandwidth = 101.562500 kHz
// Deviation = 19 kHz
// Datarate = 38.383484 kBaud
// Modulation = (1) GFSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 434 MHz
// Channel spacing = 199.951172 kHz
// Channel number = 0
// Optimization = -
// Sync mode = (3) 30/32 sync word bits detected
// Format of RX/TX data = (0) Normal mode, use FIFOs for RX and TX
// CRC operation = (1) CRC calculation in TX and CRC check in RX enabled
// Forward Error Correction = 
// Length configuration = (0) Fixed packet length, packet length configured by PKTLEN
// Packetlength = 61
// Preamble count = (2)  4 bytes
// Append status = 1
// Address check = (0) No address check
// FIFO autoflush = 0
// Device address = 0
// GDO0 signal selection = ( 6) Asserts when sync word has been sent / received, and de-asserts at the end of the packet
// GDO2 signal selection = (41) RF_RDY
RF_SETTINGS rfSettings = {
    0x0C,//08,                      // FSCTRL1   Frequency synthesizer control.
    0x00,//                         // FSCTRL0   Frequency synthesizer control.
    0x10,//21,                      // FREQ2     Frequency control word, high byte.
    0xB1,//62,                      // FREQ1     Frequency control word, middle byte.
    0x3B,//76,                      // FREQ0     Frequency control word, low byte.
    0xCA,  //F5,  //CA,             // MDMCFG4   Modem configuration.    Changed from F5 to CA   (F5 didn't seem to work)
    0x83,                           // MDMCFG3   Modem configuration.
    0x13,  //93,                    // MDMCFG2   Modem configuration.   Possibly changed 93 to 13
    0x22,//                         // MDMCFG1   Modem configuration.
    0xF8,//                         // MDMCFG0   Modem configuration.
    0x00,            //          ***// CHANNR    Channel number.
    0x34,  //15,  //34,             // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
    0x56,//                         // FREND1    Front end RX configuration.
    0x10,//                         // FREND0    Front end TX configuration.
    0x18,                           // MCSM0     Main Radio Control State Machine configuration.
    0x1D,  //16,                    // FOCCFG    Frequency Offset Compensation Configuration.
    0x1C,  //6C,                    // BSCFG     Bit synchronization Configuration.
    0xC7,  //43,                    // AGCCTRL2  AGC control.
    0x00,  //40,                    // AGCCTRL1  AGC control.
    0xB0,  //91,                    // AGCCTRL0  AGC control.
    0xEA,  //E9,                    // FSCAL3    Frequency synthesizer calibration.
    0x2A,//                         // FSCAL2    Frequency synthesizer calibration.
    0x00,//                         // FSCAL1    Frequency synthesizer calibration.
    0x1F,//                         // FSCAL0    Frequency synthesizer calibration.
    0x59,//                         // FSTEST    Frequency synthesizer calibration.
    0x88,  //81,                    // TEST2     Various test settings.
    0x31,  //35,                    // TEST1     Various test settings.
    0x09,//                         // TEST0     Various test settings.
    0x47,             //         ***// FIFOTHR   RXFIFO and TXFIFO thresholds.
    0x29,//                         // IOCFG2    GDO2 output pin configuration.
    0x06,//                         // IOCFG0    GDO0 output pin configuration. Refer to SmartRF� Studio User Manual for detailed pseudo register explanation.
    0x04,//                         // PKTCTRL1  Packet automation control.
    0x05,             //         ***// PKTCTRL0  Packet automation control.
    0x00,//                         // ADDR      Device address.
    0x78              //         ***// PKTLEN    Packet length.
};

#endif



#ifdef CCSDS_PROX1_CHANNEL1_435.6MHZ_FORWARD_LINK
// Chipcon
// Product = CC430Fx13x
// Chip version = C   (PG 0.7)
// Crystal accuracy = 10 ppm
// X-tal frequency = 26 MHz
// RF output power = 0 dBm
// RX filterbandwidth = 101.562500 kHz
// Deviation = 19 kHz
// Datarate = 38.383484 kBaud
// Modulation = (1) GFSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 435.6 MHz
// Channel spacing = 199.951172 kHz
// Channel number = 0
// Optimization = -
// Sync mode = (3) 30/32 sync word bits detected
// Format of RX/TX data = (0) Normal mode, use FIFOs for RX and TX
// CRC operation = (1) CRC calculation in TX and CRC check in RX enabled
// Forward Error Correction = 
// Length configuration = (0) Fixed packet length, packet length configured by PKTLEN
// Packetlength = 61
// Preamble count = (2)  4 bytes
// Append status = 1
// Address check = (0) No address check
// FIFO autoflush = 0
// Device address = 0
// GDO0 signal selection = ( 6) Asserts when sync word has been sent / received, and de-asserts at the end of the packet
// GDO2 signal selection = (41) RF_RDY
RF_SETTINGS rfSettings = {
    0x06,  //0C,//08,               // FSCTRL1   Frequency synthesizer control.
    0x00,//                         // FSCTRL0   Frequency synthesizer control.
    0x10,////21,                    // FREQ2     Frequency control word, high byte.
    0xC0,  //B1,//62,               // FREQ1     Frequency control word, middle byte.
    0xFC,////3B,//76,               // FREQ0     Frequency control word, low byte.
    0xF5,  //CA,  //F5,  //CA,      // MDMCFG4   Modem configuration.    Changed from F5 to CA   (F5 didn't seem to work)
    0x83,////                       // MDMCFG3   Modem configuration.
    0x13,  //93,                    // MDMCFG2   Modem configuration.   Possibly changed 93 to 13
    0x22,////                       // MDMCFG1   Modem configuration.
    0xF8,////                       // MDMCFG0   Modem configuration.
    0x00,//                      ***// CHANNR    Channel number.
    0x15,  //34,  //15,  //34,      // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
    0x56,////                       // FREND1    Front end RX configuration.
    0x10,////                       // FREND0    Front end TX configuration.
    0x10,  //18,                    // MCSM0     Main Radio Control State Machine configuration.
    0x16, //1D,  //16,              // FOCCFG    Frequency Offset Compensation Configuration.
    0x6C, //1C,  //6C,              // BSCFG     Bit synchronization Configuration.
    0x03, //C7,  //43,              // AGCCTRL2  AGC control.
    0x40, //00,  //40,              // AGCCTRL1  AGC control.
    0x91, //B0,  //91,              // AGCCTRL0  AGC control.
    0xE9, //EA,  //E9,              // FSCAL3    Frequency synthesizer calibration.
    0x2A,////                       // FSCAL2    Frequency synthesizer calibration.
    0x00,////                       // FSCAL1    Frequency synthesizer calibration.
    0x1F,////                       // FSCAL0    Frequency synthesizer calibration.
    0x59,////                       // FSTEST    Frequency synthesizer calibration.
    0x81, //88,//81,                // TEST2     Various test settings.
    0x35, //31,  //35,              // TEST1     Various test settings.
    0x09,////                       // TEST0     Various test settings.
    0x47,////                    ***// FIFOTHR   RXFIFO and TXFIFO thresholds.
    0x29,////                       // IOCFG2    GDO2 output pin configuration.
    0x06,//                         // IOCFG0    GDO0 output pin configuration. Refer to SmartRF� Studio User Manual for detailed pseudo register explanation.
    0x04,//                         // PKTCTRL1  Packet automation control.
    0x05,       //               ***// PKTCTRL0  Packet automation control.
    0x00,//                         // ADDR      Device address.
    0x78        //               ***// PKTLEN    Packet length.
};

#endif



#ifdef CCSDS_PROX1_CHANNEL1_404.4MHZ_RETURN_LINK
// Chipcon
// Product = CC430Fx13x
// Chip version = C   (PG 0.7)
// Crystal accuracy = 10 ppm
// X-tal frequency = 26 MHz
// RF output power = 0 dBm
// RX filterbandwidth = 101.562500 kHz
// Deviation = 19 kHz
// Datarate = 38.383484 kBaud
// Modulation = (1) GFSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 404.4 MHz
// Channel spacing = 199.951172 kHz
// Channel number = 0
// Optimization = -
// Sync mode = (3) 30/32 sync word bits detected
// Format of RX/TX data = (0) Normal mode, use FIFOs for RX and TX
// CRC operation = (1) CRC calculation in TX and CRC check in RX enabled
// Forward Error Correction = 
// Length configuration = (0) Fixed packet length, packet length configured by PKTLEN
// Packetlength = 61
// Preamble count = (2)  4 bytes
// Append status = 1
// Address check = (0) No address check
// FIFO autoflush = 0
// Device address = 0
// GDO0 signal selection = ( 6) Asserts when sync word has been sent / received, and de-asserts at the end of the packet
// GDO2 signal selection = (41) RF_RDY
RF_SETTINGS rfSettings = {
    0x0C,////08,                    // FSCTRL1   Frequency synthesizer control.
    0x00,////                       // FSCTRL0   Frequency synthesizer control.
    0x0F,  //10,//21,               // FREQ2     Frequency control word, high byte.
    0x8D,  //B1,//62,               // FREQ1     Frequency control word, middle byte.
    0xC9,  //3B,//76,               // FREQ0     Frequency control word, low byte.
    0x2D,  //CA,  //F5,  //CA,      // MDMCFG4   Modem configuration.    Changed from F5 to CA   (F5 didn't seem to work)
    0x3B,  //83,                    // MDMCFG3   Modem configuration.
    0x13,////93,                    // MDMCFG2   Modem configuration.   Possibly changed 93 to 13
    0x22,////                       // MDMCFG1   Modem configuration.
    0xF8,////                       // MDMCFG0   Modem configuration.
    0x14,  //00,                 ***// CHANNR    Channel number.
    0x62,  //34,  //15,  //34,      // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
    0xB6,  //56,//                  // FREND1    Front end RX configuration.
    0x10,////                       // FREND0    Front end TX configuration.
    0x10,  //18,                    // MCSM0     Main Radio Control State Machine configuration.
    0x1D,////16,                    // FOCCFG    Frequency Offset Compensation Configuration.
    0x1C,////6C,                    // BSCFG     Bit synchronization Configuration.
    0xC7,////43,                    // AGCCTRL2  AGC control.
    0x00,////40,                    // AGCCTRL1  AGC control.
    0xB0,////91,                    // AGCCTRL0  AGC control.
    0xEA,////E9,                    // FSCAL3    Frequency synthesizer calibration.
    0x2A,////                       // FSCAL2    Frequency synthesizer calibration.
    0x00,////                       // FSCAL1    Frequency synthesizer calibration.
    0x1F,////                       // FSCAL0    Frequency synthesizer calibration.
    0x59,////                       // FSTEST    Frequency synthesizer calibration.
    0x88,////81,                    // TEST2     Various test settings.
    0x31,////35,                    // TEST1     Various test settings.
    0x0B,  //09,  //                // TEST0     Various test settings.
    0x07,  //47,                 ***// FIFOTHR   RXFIFO and TXFIFO thresholds.
    0x29,////                       // IOCFG2    GDO2 output pin configuration.
    0x06,////                       // IOCFG0    GDO0 output pin configuration. Refer to SmartRF� Studio User Manual for detailed pseudo register explanation.
    0x04,//                         // PKTCTRL1  Packet automation control.
    0x05,         //             ***// PKTCTRL0  Packet automation control.       SMART RF STUDIO SAYS SHOULD BE 05 NOT 04
    0x00,//                         // ADDR      Device address.
    0x78          //             ***// PKTLEN    Packet length.                  SMART RF STUDIO SAYS FF
};

#endif
/*
#endif

#if !defined (MHZ_868) && !defined (MHZ_915)
#error "Please select MHZ_868 or MHZ_915 as the active project configuration" 
#endif*/


#else
/* Leave this blank so it is ignored as it means this file has already been included */
#endif
