/******************************************************************************
* CC430 RF Code Example - TX and RX (variable packet length =< FIFO size)
*
* This code example can be loaded to 2 CC430 devices. Each device will be 
* constantly receiving and every so often will transmit a CCSDS compatible 
* packet. The second device will receive this and if the CRC check indicates
* the packet is not corrupted, it will flash its LED and print the packet's
* data field to the serial monitor.
* 
* Please refer to the appropriate legal sources before performing tests with 
* this code example. 
* 
* 
* (The RF packet engine settings specify variable-length-mode with CRC check 
* enabled. The RX packet also appends 2 status bytes regarding CRC check, RSSI 
* and LQI info. For specific register settings please refer to the comments for 
* each register in RfRegSettings.c, the CC430x513x User's Guide, and/or 
* SmartRF Studio.)
* 
*
* This code is modified source code supplied by Texas Instruments Inc. 
* 
* Luke Bussell
* PocketSpacecraft.com
******************************************************************************/

#include "Function_Definitions.h"
#include "Energia.h"
#include "HAL_PMM.h"
#include "RF1A.h"
#include "RF1A.c"
#include "HAL_PMM.c"
//#include "RfRegSettings.c"


#define __even_in_range(a,b) (a)


#define  PACKET_LEN         (0x0B)	    // PACKET_LEN <= 61                    0x0B = 11
#define  RSSI_IDX           (PACKET_LEN+1)  // Index of appended RSSI 
#define  CRC_LQI_IDX        (PACKET_LEN+2)  // Index of appended LQI, checksum
#define  CRC_OK             (BIT7)          // CRC_OK bit 
#define  PATABLE_VAL        (0x51)          // 26=-12 dBm output              2D=-6 dBm output              50=0 dBm output              C6=10 dBm output


unsigned char POWER_SETTING = 5;

extern RF_SETTINGS rfSettings;

unsigned char packetReceived;
unsigned char packetTransmit; 

unsigned char RxBuffer[64];
unsigned char RxBufferLength = 0;



//                                                  /  ///              /
const unsigned char CCSDS_Version_Identifier[2]= {0x17, 0xF5,};        //###   CCSDS packet version number and packet identification  ###  0x17=0b00010111     0xF5=0b11110101
//                                               / /                                 
const unsigned char CCSDS_SequenceControl[2]= {0xC0, 0x01,};           //###   CCSDS packet version number and packet identification  ###   0xC0=0b11000000     0x=0b00000001



unsigned char TxBuffer[11]= {CCSDS_Version_Identifier[0], CCSDS_Version_Identifier[1], CCSDS_SequenceControl[0], CCSDS_SequenceControl[1], 'z', PACKET_LEN, POWER_SETTING/*'M'*/, '6', 'Y', 'L', 'B'};        //###   Transmission   ###


const unsigned int CCSDS_PacketDataLength = sizeof TxBuffer;                       //###   CCSDS packet version number and packet identification  ###

unsigned char buttonPressed = 0;
unsigned int i = 0; 
int x = 1;
int randomDivisor;
unsigned int printedRxBufferCount = 0;
unsigned int receivedPacketCount = 0;
unsigned int transmittedPacketCount = 0;
unsigned long durationTime = 0;
unsigned long transmissionDuration = 0;

unsigned char transmitting = 0; 
unsigned char receiving = 0; 
unsigned char filledRxBufferStatus = 0;
unsigned int powerSetting = 0;



void setup()
{
  TxBuffer[4] = CCSDS_PacketDataLength;
  Serial.begin(9600);
  pinMode(5, OUTPUT);
  randomSeed(10);
  powerSetting = 7;
  randomDivisor = 75;
}



void loop()
{  
  
  
  Serial.print("IN LOOP\n\n");
  
  if(filledRxBufferStatus == 1)
  {
    printReceivedData();
    clearRxBuffer();
  }
  
  // Stop watchdog timer to prevent time out reset 
//  WDTCTL = WDTPW + WDTHOLD;


  // Increase PMMCOREV level to 2 for proper radio operation
  SetVCore(2);                            
  
  ResetRadioCore();     
  InitRadio();
  InitButtonLeds();
    
  ReceiveOn(); 
  receiving = 1; 
    
  //randomDivisor = random(10, 100);
  //randomDivisor = 10;  
  while (1)
  { 
    __bis_SR_register( LPM3_bits + GIE );   
    __no_operation(); 
    
    if (x == randomDivisor)  
    {
      TxBuffer[6] = 3;
      for(int theCounter = 0; theCounter < 1; theCounter++)
      {
        printTransmissionMessage();
        
        digitalWrite(5, HIGH);
        P3OUT |= BIT6;                        // Pulse LED during Transmit                          
        buttonPressed = 0; 
        P1IFG = 0; 
      
        Serial.print("TRANSMITTING\n\n");
        ReceiveOff();
        receiving = 0;
        Transmit( (unsigned char*)TxBuffer, sizeof TxBuffer );         
        transmitting = 1;
       
        P1IE |= BIT7;                         // Re-enable button press  
        x=1;
        digitalWrite(5, LOW);
        receiving = 0;
        transmittedPacketCount++;
      }
      randomDivisor = random(10, 100);
      break;
    }
    else //if(!transmitting)
    {
      //Serial.print("Receiving\n\n");
      Serial.print("Transmitted ");
      Serial.print(transmittedPacketCount);
      Serial.print(" packets and received ");
      Serial.print(receivedPacketCount);
      Serial.print(" packets and printed RxBuffer ");
      Serial.print(printedRxBufferCount);
      Serial.print(" times.     Time since startup is ");
      durationTime = millis();
      durationTime = durationTime/1000;
      Serial.print(durationTime);
      Serial.print(" seconds.     The random divisor is ");
      Serial.print(randomDivisor);
      Serial.print(".     x = ");
      Serial.print(x);
      Serial.print(".\n\n");
      ReceiveOn();      
      receiving = 1;       
      x++;
      if(x > randomDivisor)
      {
        x=1;
      }
    }
    if(filledRxBufferStatus == 1)
    {
      break;
    }
  }
  checkRxBuffer();
}

void InitButtonLeds(void)
{
  // Set up the button as interruptible 
  P1DIR &= ~BIT7;
  P1REN |= BIT7;
  P1IES &= BIT7;
  P1IFG = 0;
  P1OUT |= BIT7;
  P1IE  |= BIT7; 

  // Initialize Port J
  PJOUT = 0x00;
  PJDIR = 0xFF; 

  // Set up LEDs 
  P1OUT &= ~BIT0;
  P1DIR |= BIT0;
  P3OUT &= ~BIT6;
  P3DIR |= BIT6;
}

void InitRadio(void)
{
  // Set the High-Power Mode Request Enable bit so LPM3 can be entered
  // with active radio enabled 
  PMMCTL0_H = 0xA5;
  PMMCTL0_L |= PMMHPMRE_L; 
  PMMCTL0_H = 0x00; 
  
  WriteRfSettings(&rfSettings);
  
  WriteSinglePATable(PATABLE_VAL);
}

void Transmit(unsigned char *buffer, unsigned char length)
{
  RF1AIES |= BIT9;                          
  RF1AIFG &= ~BIT9;                         // Clear pending interrupts
  RF1AIE |= BIT9;                           // Enable TX end-of-packet interrupt
  
  WriteBurstReg(RF_TXFIFOWR, buffer, length);
  
  Strobe( RF_STX );                         // Strobe STX   
}

void ReceiveOn(void)
{  
  RF1AIES |= BIT9;                          // Falling edge of RFIFG9
  RF1AIFG &= ~BIT9;                         // Clear a pending interrupt
  RF1AIE  |= BIT9;                          // Enable the interrupt 
  
  // Radio is in IDLE following a TX, so strobe SRX to enter Receive Mode
  Strobe( RF_SRX );                      
}

void ReceiveOff(void)
{
  RF1AIE &= ~BIT9;                          // Disable RX interrupts
  RF1AIFG &= ~BIT9;                         // Clear pending IFG

  // It is possible that ReceiveOff is called while radio is receiving a packet.
  // Therefore, it is necessary to flush the RX FIFO after issuing IDLE strobe 
  // such that the RXFIFO is empty prior to receiving a packet.
  Strobe( RF_SIDLE );
  Strobe( RF_SFRX  );                       
}

#pragma vector=CC1101_VECTOR
__interrupt void CC1101_ISR(void)
{
  switch(__even_in_range(RF1AIV,32))        // Prioritizing Radio Core Interrupt 
  {
    case  0: break;                         // No RF core interrupt pending                                            
    case  2: break;                         // RFIFG0 
    case  4: break;                         // RFIFG1
    case  6: break;                         // RFIFG2
    case  8: break;                         // RFIFG3
    case 10: break;                         // RFIFG4
    case 12: break;                         // RFIFG5
    case 14: break;                         // RFIFG6          
    case 16: break;                         // RFIFG7
    case 18: break;                         // RFIFG8
    case 20:                                // RFIFG9
      if(receiving)			    // RX end of packet
      {
        // Read the length byte from the FIFO       
        RxBufferLength = ReadSingleReg( RXBYTES );               
        ReadBurstReg(RF_RXFIFORD, RxBuffer, RxBufferLength); 
        
        // Stop here to see contents of RxBuffer
        __no_operation(); 		   
        
        // Check the CRC results
        if(RxBuffer[CRC_LQI_IDX] & CRC_OK) 
        { 
          P1OUT ^= BIT0;          // Toggle LED1 
          powerSetting = RxBuffer[7];                        //### Sets powerSetting determining number of LED flashes
          receivedPacketCount++;
          blinkLEDs();
          filledRxBufferStatus=1;
        }
      }
      else if(transmitting)		    // TX end of packet
      {
        RF1AIE &= ~BIT9;                    // Disable TX end-of-packet interrupt
        P3OUT &= ~BIT6;                     // Turn off LED after Transmit               
        transmitting = 0; 
      }
      else while(1); 			    // trap 
      break;
    case 22: break;                         // RFIFG10
    case 24: break;                         // RFIFG11
    case 26: break;                         // RFIFG12
    case 28: break;                         // RFIFG13
    case 30: break;                         // RFIFG14
    case 32: break;                         // RFIFG15
  }  
  __bic_SR_register_on_exit(LPM3_bits);     
}

#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
  switch(__even_in_range(P1IV, 16))
  {
    case  0: break;
    case  2: break;                         // P1.0 IFG
    case  4: break;                         // P1.1 IFG
    case  6: break;                         // P1.2 IFG
    case  8: break;                         // P1.3 IFG
    case 10: break;                         // P1.4 IFG
    case 12: break;                         // P1.5 IFG
    case 14: break;                         // P1.6 IFG
    case 16:                                // P1.7 IFG
      P1IE = 0;                             // Debounce by disabling buttons
      buttonPressed = 1;
      __bic_SR_register_on_exit(LPM3_bits); // Exit active    
      break;
  }
}



void checkRxBuffer()
{
  if(RxBuffer[0] != 0)
  {
    Serial.print("\n\nRxBuffer contents:\n");
    for(int RxBufferPointer=0; RxBufferPointer<sizeof RxBuffer; RxBufferPointer++)
    {
      Serial.print("Position ");
      Serial.print(RxBufferPointer+1);
      Serial.print(":     Contents:     ");
      Serial.print(RxBuffer[RxBufferPointer]);
      Serial.print("     Ascii:     ");
      Serial.write(RxBuffer[RxBufferPointer]);
      Serial.print("     Binary:     ");
      Serial.print(RxBuffer[RxBufferPointer], BIN);
      Serial.print("\n");
    }
    Serial.print("\n");
    printedRxBufferCount++;
    filledRxBufferStatus = 1;
  }
}



void printTransmissionMessage()
{
  Serial.print("\nTransmission:\n");
  Serial.print("\n- RAW:\n");
  for(int TxBufferPointer=0; TxBufferPointer<sizeof TxBuffer; TxBufferPointer++)
  {
    Serial.print(TxBuffer[TxBufferPointer], BIN);
    Serial.print("\n");
  }
  
  Serial.print("\n- Comprehensible:\n");
  Serial.println(TxBuffer[0], BIN);
  Serial.println(TxBuffer[1], BIN);
  Serial.println(TxBuffer[2], BIN);
  Serial.println(TxBuffer[3], BIN);
  Serial.println(TxBuffer[4], BIN);
  Serial.println(TxBuffer[5]);
  Serial.write(TxBuffer[6]);
  Serial.print("\n");
  Serial.write(TxBuffer[7]);
  Serial.print("\n");
  Serial.write(TxBuffer[8]);
  Serial.print("\n");
  Serial.write(TxBuffer[9]);
  Serial.print("\n");
  Serial.write(TxBuffer[10]);
  Serial.print("\n");   
}



void clearRxBuffer()
{
  Serial.print("Clearing RxBuffer\n\n");
  for(int RxBufferPointer=0; RxBufferPointer<12; RxBufferPointer++)
  {
    RxBuffer[RxBufferPointer] = 0;
  } 
}



void printReceivedData()
{
  Serial.print("\n\nReceived Message:\n");
  int messageLimiter = RxBuffer[6] + 1;
  for(int RxBufferPointer=7; RxBufferPointer<messageLimiter; RxBufferPointer++)
  {
    Serial.write(RxBuffer[RxBufferPointer]);
    Serial.print(" ");
  }
  Serial.print("\n");
  filledRxBufferStatus=0;
}



void blinkLEDs()
{
  for(int blinkingCount=0; blinkingCount<powerSetting; blinkingCount++)
  {
    digitalWrite(5, HIGH);
    delay(1000);
    digitalWrite(5, LOW);
    delay(500);
  }
}
