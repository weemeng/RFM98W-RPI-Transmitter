//RPI for RFM98W
//by Ng Wee Meng

#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>


#define REG_FIFO                    0x00 	//okay
#define REG_OPMODE                  0x01	//ever changing
#define REG_BITRATEMSB				0x02	//0x00 fixed
#define REG_BITRATELSB				0x03	//0x6B fixed
#define REG_FDEVMSB					0x04	//0x00
#define REG_FDEVLSB					0x05	//0x52
#define REG_FRFMSB					0x06	//0x6C
#define REG_FRFMID					0x07	//0x40
#define REG_FRFLSB					0x08	//0x00
#define REG_PACONFIG				0x09	//0xFF
#define REG_PARAMP					0x0A	//01001100
#define REG_OCP						0x0B	//00101101

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

// Oscillator
#define REG_OSC						0x24	//0x03 clockout off

// Packet Handling
#define REG_PREAMBLEMSB				0x25 	//0x5A
#define REG_PREAMBLELSB				0x26 	//0xA5
#define REG_SYNCCONFIG				0x27	//0x98 10011000
#define REG_SYNCVALUE1				0x28	//0x48
#define REG_SYNCVALUE2				0x29
#define REG_SYNCVALUE3				0x2A
#define REG_SYNCVALUE4				0x2B
#define REG_SYNCVALUE5				0x2C
#define REG_SYNCVALUE6				0x2D
#define REG_SYNCVALUE7				0x2E
#define REG_SYNCVALUE8				0x2F
#define REG_PACKETCONFIG1			0x30	//0x18 00011000
#define REG_PACKETCONFIG2			0x31	//0x40 01000-000//111
#define REG_PAYLOADLENGTH			0x32	//change to 7FF - 2047 bytes
#define REG_FIFOTHRESH				0x35    //0xBC 1011 1100 

//	Sequencer
#define REG_SEQCONFIG1				0x36
#define REG_SEQCONFIG2				0x37

#define REG_IMAGECAL				0x3B	//0100-0001
#define REG_IRQFLAGS1				0x3E	//trigger
#define REG_IRQFLAGS2				0x3F	//trigger
#define REG_DIOMAPPING1				0x40	//00-00-00-01
#define REG_DIOMAPPING2				0x41	//00-11-000-1
#define REG_PADAC					0x4D	//00000100 //regular boost
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
uint8_t currentMode = 0x09;

void setup() {
  printf("Balloon Initializing...");
  setRFM98W();
  printf("Setup Complete");
}

int running() {
  //spitx.tx_buf(0x0D | 
}

/*
void SPI_Send_Byte(unsigned char Data) {
    digitalWrite(24, LOW);
	unsigned char buf[2];
    buf[0] = (Data>>8);
	buf[1] = (0x00FF & Data);
    wiringPiSPIDataRW(0, buf, 2);
	digitalWrite(24, HIGH);
}

unsigned char SPI_Read_Byte(unsigned char Data) {
    digitalWrite(24, LOW);
	unsigned char buf[1];
    buf[0] = Data;
    wiringPiSPIDataRW(0, buf, 1);
	digitalWrite(24, HIGH);
	return buf;
}*/
void spi_send_byte(uint16_t Data) {
    digitalWrite(24, LOW);
	uint8_t txbuf[2];
    txbuf[0] = (0x80 | Data>>8);
	txbuf[1] = (0x00FF & Data);
    wiringPiSPIDataRW(0, txbuf, 2);
	digitalWrite(24, HIGH);
}

uint8_t spi_rcv_data(uint8_t Data) {
    digitalWrite(24, LOW);
	uint8_t rxbuf[1];
    rxbuf[0] = Data;
    wiringPiSPIDataRW(0, rxbuf, 1);
	digitalWrite(24, HIGH);
	return buf[0];
}

int main(void) { //int argc, char *argv[]
	int i, j, k;
	wiringPiSetup();
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
	if ((i = wiringPiSPISetup(0, 8000000))<0)
		return -1;
	SetFSKMod();
	//testCommunication();
	//Receiver_Startup();
}

void SetFSKMod()
{
  printf("Setting FSK Mode\n");
  setMode(RF98M_MODE_SLEEP);
  //spi_send_byte(REG_OPMODE,0x80);
   
  // frequency  
  //setMode(RF98M_MODE_SLEEP);
  /*
  spi_send_byte(0x06, 0x6C);
  spi_send_byte(0x07, 0x9C);
  spi_send_byte(0x08, 0xCC);
  
  spi_send_byte(0x06, 0x6C);
  spi_send_byte(0x07, 0x9C);
  spi_send_byte(0x08, 0x8E);
  */
  spi_send_byte(REG_BITRATEMSB, 0x00);
  spi_send_byte(REG_BITRATELSB, 0x68);
  spi_send_byte(REG_FRFMSB, 0x6C); //exact at 433Mhz
  spi_send_byte(REG_FRFMID, 0x40);
  spi_send_byte(REG_FRFLSB, 0x00);
   
  Serial.println("FSK Mode Set");
  
  Serial.print("Mode = "); 
  Serial.println(spi_rcv_data(REG_OPMODE));
  
  return;
}
void setMode(uint8_t newMode)
{
  if(newMode == currentMode)
    return;  
  
  switch (newMode) 
  {
    case RF98M_MODE_SLEEP:
      Serial.println("Changing to Sleep Mode"); 
      spi_send_byte(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    case RFM98_MODE_STANDBY:
      Serial.println("Changing to Standby Mode");
      spi_send_byte(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
	case RFM98_MODE_FSTX:
      Serial.println("Changing to FSTx Mode");
      spi_send_byte(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
	case RFM98_MODE_TX:
      Serial.println("Changing to Tx Mode");
      spi_send_byte(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
	case RFM98_MODE_FSRX:
      Serial.println("Changing to FSRx Mode");
      spi_send_byte(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
	case RFM98_MODE_RX:
      Serial.println("Changing to Rx Mode");
      spi_send_byte(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    default: return;
  } 
  
  if(newMode != RF98M_MODE_SLEEP){ //test on ModeReady
    while(digitalRead(dio5) == 0)
    {
      Serial.println("Wait for it...");
    } 
  }
  Serial.println("Mode Change Done");
  return;
}
void Receiver_Startup()
{
  //initialize
  setMode(RFM98_MODE_STANDBY);
  spi_send_byte(REG_LNA, 0xC0);
  spi_send_byte(REG_RXCONFIG, 0x1E);
  spi_send_byte(REG_RSSICONFIG, 0x03);
  spi_send_byte(REG_RSSICOLLISION, 0x0A);  
  spi_send_byte(REG_RXBW, 0x09);
  spi_send_byte(REG_AFCFEI, 0x03);
  spi_send_byte(REG_PREAMBLEDETECT, 0xAA);
  spi_send_byte(REG_RXTIMEOUT1, 0x00);
  spi_send_byte(REG_RXTIMEOUT2, 0x00);
  spi_send_byte(REG_RXTIMEOUT3, 0x00);
  spi_send_byte(REG_OSC, 0x03); //@standby Clkout turned off
  spi_send_byte(REG_PREAMBLEMSB, 0x5A); //23205
  spi_send_byte(REG_PREAMBLELSB, 0xA5);
  spi_send_byte(REG_SYNCCONFIG, 0x98);
  spi_send_byte(REG_SYNCVALUE1, 0x48);
  spi_send_byte(REG_PACKETCONFIG1, 0x18);
  spi_send_byte(REG_PACKETCONFIG2, 0x40);
  spi_send_byte(REG_PAYLOADLENGTH, 0xFF);
  spi_send_byte(REG_IMAGECAL, 0x41);
  spi_send_byte(REG_DIOMAPPING1, 0x04);
  spi_send_byte(REG_DIOMAPPING2, 0xF1);
  spi_send_byte(REG_PLLHOP, 0x00);
  setMode(RFM98_MODE_FSRX);
  setMode(RFM98_MODE_RX);
  //Serial.print(digitalRead(dio5)); 	//check these values
  //Serial.println(digitalRead(dio0));	//check these values
}

void CheckRx()
{
  char Message[256], RSSIString[6];
  //SentenceCount
  Serial.print("Signal Strength is at "); 
  Serial.print(-(spi_rcv_data(REG_RSSIVALUE))/2);
  Serial.println("dBm"); 
  
  if  ((digitalRead(dio0) == 0) && (digitalRead(dio3) == 0))
	state = 1; 	//payload is not ready and fifo is not ready
  else if ((digitalRead(dio0) == 1) && (digitalRead(dio3) == 0))
	state = 2; 	//payload is ready and fifo is not ready
  else if ((digitalRead(dio0) == 1) && (digitalRead(dio3) == 1))
	state = 3; 	//payload is ready and fifo is ready
  else
    state = 4;  //settled by interrupt
  
  switch (state) {
  case 1:
	Serial.println("Case 1 Triggered");
	Bytes = receiveMessage(Message, CurrentCount);
	state = 3;
	Serial.println("state transition from 1 to 3");
	break;
  case 2:
	Serial.println("Case 2 Triggered");
	Bytes = receiveMessage(Message, CurrentCount);
	state = 3;
	Serial.println("state transition from 2 to 3");
	break;
  case 3:
	Serial.println("Case 3 Triggered");
	//check CRC okay, Reset and load to SD card
	for (int k = 0; k < Bytes; k++) {
	  Serial.print(Message[k]);
	}
	Serial.println();
	Bytes = 0;
	Serial.println("wait for new packet...");
	break;
  case 4:
	Serial.println("Case 4 Triggered");
	//wait for interrupt trigger
	if (Bytes == 0) 
	  Serial.println("Think I'm waiting for some data..");
	else
	  Serial.println("Think there is some transmission problem, please wait"); //dont think it will reach here
	break;
  }

}
int receiveMessage(char *message, int i)
{
  int Package = 256;
  
  while (digitalRead(dio3) == 0) {	//Fifo isnt empty
	if (i < Package) {
	  message[i] = (unsigned char)spi_rcv_data(REG_FIFO);
	}
  }
  message[i+1] = '\0';
  return i+1;
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
