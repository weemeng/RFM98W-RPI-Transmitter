//RPI for RFM98W
//by Ng Wee Meng

#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>


#define REG_FIFO                    0x00 	//okay
#define REG_OPMODE                  0x01	//ever changing
#define REG_BITRATEMSB				0x02	//0x00 fixed
#define REG_BITRATELSB				0x03	//0x6B fixed
#define REG_FDEVMSB					0x04	//DontCare
#define REG_FDEVLSB					0x05	//DontCare 5kHz pg 87
#define REG_FRFMSB					0x06
#define REG_FRFMID					0x07
#define REG_FRFLSB					0x08
//#define REG_PACONFIG				0x09 	//09, 0A, 0B are for transmitter

// for Receiver
#define REG_LNA						0x0C	//1100-0000
#define REG_RXCONFIG				0x0D	//00011-110
#define REG_RSSICONFIG				0x0E	//00000011
#define REG_RSSICOLLISION			0x0F	//0x0A
#define REG_RSSITHRESH				0x10	//dont care for now
#define REG_RSSIVALUE				0x11	//read
#define REG_RXBW					0x12	//00001001 0x09 InitialConfig as AfcAutoOn=1
#define REG_AFCBW					0x13	//read
#define REG_AFCFEI					0x1A	//00000000 trigger
#define REG_AFCMSB					0x1B	//auto
#define REG_AFCLSB					0x1C	//auto
#define REG_FEIMSB					0x1D	//read
#define REG_FEILSB					0x1E	//read
#define REG_PREAMBLEDETECT			0x1F	//101-01010
#define REG_RXTIMEOUT1				0X20
#define REG_RXTIMEOUT2				0X21
#define REG_RXTIMEOUT3				0X22
#define REG_OSC						0x24	//0x05 clockout at 1Mhz

// Packet Handling
#define REG_PREAMBLEMSB				0x25 	//0x5A
#define REG_PREAMBLELSB				0x26 	//0xA5
#define REG_SYNCCONFIG				0x27	//0x98 10001000 10011000
#define REG_SYNCVALUE1				0x28	//0x48
#define REG_SYNCVALUE2				0x29
#define REG_SYNCVALUE3				0x2A
#define REG_SYNCVALUE4				0x2B
#define REG_SYNCVALUE5				0x2C
#define REG_SYNCVALUE6				0x2D
#define REG_SYNCVALUE7				0x2E
#define REG_SYNCVALUE8				0x2F
#define REG_PACKETCONFIG1			0x30	//0x08 00001000
#define REG_PACKETCONFIG2			0x31	//0x40 01000-000//111
#define REG_PAYLOADLENGTH			0x32	//change to 7FF - 2047 bytes
#define REG_FIFOTHRESH				0x35

//	Sequencer
#define REG_SEQCONFIG1				0x36
#define REG_SEQCONFIG2				0x37

#define REG_IMAGECAL				0x3B	//0100-0001
#define REG_IRQFLAGS1				0x3E	//trigger
#define REG_IRQFLAGS2				0x3F	//trigger
#define REG_DIOMAPPING1				0x40	//00-00-01-00
#define REG_DIOMAPPING2				0x41	//11-11-000-1
#define REG_PLLHOP					0x44
#define REG_BITRATEFRAC				0x5D	//super minute accuracy which can be accounted for by the doppler shift
#define REG_AGCREFLF				0x61
#define REG_AGCTHRESHLF1			0x62
#define REG_AGCTHRESHLF2			0x63
#define REG_AGCTHRESHLF3			0x64
	

//MODES
#define RF98M_MODE_SLEEP			0x08	
#define RFM98_MODE_STANDBY			0x09	//00001001
#define RFM98_MODE_FSTX				0x0A	//00001010
#define RFM98_MODE_TX				0x0B	//00001011
#define RFM98_MODE_FSRX				0x0C	//00001100
#define RFM98_MODE_RX				0x0D	//00001101

const int _slaveSelectPin = 10; 
const int dio0pin = 29;
const int dio3pin = 31;
const int dio4pin = 32;
const int dio5pin = 33; 

void setup() {
  printf("Balloon Initializing...");
  setRFM98W();
  printf("Setup Complete");
}

int running() {

}

int main(void) { //int argc, char *argv[]
  setup();  
  while (1){
  
  
  
  
  
  
    
  }
  return;
}

void setRFM98W(void)
{
  // initialize the pins
  pinModeGpio( _slaveSelectPin, OUTPUT);
  pinModeGpio(dio0pin, INPUT);
  pinModeGpio(dio3pin, INPUT);
  pinModeGpio(dio4pin, INPUT);
  pinModeGpio(dio5pin, INPUT);
  //setInterrupts();
  //SPI.begin();
  //SetFSKMod();
  //testCommunication();
  //Receiver_Startup();
}


/*
int readadc(adcnum)
{
 uint8_t buff[3];
 int adc;
 if ((adcnum > 7) || (adcnum < 0))
            return -1;
 buff[0] = 1;
 buff[1] = (8+adcnum)<<4;
 buff[2] = 0;
 wiringPiSPIDataRW(0, buff, 3);
 adc = ((buff[1]&3) << 8) + buff[2];
 return adc;
}

int main(int argc, char *argv[])
{
 int i, chan;
 uint32_t x1, tot ;
 
 printf ("SPI test program\n") ;
 // initialize the WiringPi API
 if (wiringPiSPISetup (0, 1000000) < 0)
  return -1 ;
 
 // get the channel to read, default to 0
 if (argc>1)
  chan = atoi(argv[1]);
 else
  chan = 0;
 
 // run until killed with Ctrl-C
 while (1)
 {
  tot = 0;
  for (i=0; i<100; i++)
  {
   // read data and add to total
   x1 = readadc(chan);
   tot += x1;
   delay(10);
  }
  // display the average value
  printf("chan %d:  %d \n", chan, (tot/100)) ;
 }
  
 return 0 ;
}
*/
