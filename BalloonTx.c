//RPI for RFM98W
//by Ng Wee Meng

#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <stdint.h>
//#include <fcntl.h>
//#include <sys/ioctl.h>
//#include <linux/spi/spidev.h>


#define REG_FIFO                    0x00 	//okay
#define REG_OPMODE                  0x01	//ever changing
#define REG_BITRATEMSB				0x02	//0x00 fixed
#define REG_BITRATELSB				0x03	//0x6B fixed
#define REG_FDEVMSB					0x04	//0x00
#define REG_FDEVLSB					0x05	//0x52 //Set to 5Khz
#define REG_FRFMSB					0x06	//0x6C
#define REG_FRFMID					0x07	//0x40
#define REG_FRFLSB					0x08	//0x00
#define REG_PACONFIG				0x09	//0x4B 
#define REG_PARAMP					0x0A	//0x09 00001001
#define REG_OCP						0x0B	//0x00 00010001

// for Receiver
/*
#define REG_LNA						0x0C	//0xC0 1100-0000
#define REG_RXCONFIG				0x0D	//0x1E 00011-110
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
*/
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

#define REG_IMAGECAL				0x3B	//0000-0000	//prev41
#define REG_IRQFLAGS1				0x3E	//trigger
#define REG_IRQFLAGS2				0x3F	//trigger
#define REG_DIOMAPPING1				0x40	//00-00-00-01
#define REG_DIOMAPPING2				0x41	//00-11-000-1
//#define REG_PLLHOP					0x44
//#define REG_PADAC					0x4D	//00000100 //regular boost
#define REG_BITRATEFRAC				0x5D	//super minute accuracy which can be accounted for by the doppler shift
#define REG_AGCREFLF				0x61
#define REG_AGCTHRESHLF1			0x62
#define REG_AGCTHRESHLF2			0x63
#define REG_AGCTHRESHLF3			0x64
	

//MODES
#define RFM98_MODE_SLEEP			0x08	
#define RFM98_MODE_STANDBY			0x09	//00001001
#define RFM98_MODE_FSTX				0x0A	//00001010
#define RFM98_MODE_TX				0x0B	//00001011
#define RFM98_MODE_FSRX				0x0C	//00001100
#define RFM98_MODE_RX				0x0D	//00001101

const int SSpin = 24; 
const int dio0pin = 5; //29 WiringPi BCM pin numbers
const int dio1pin = 23; //16
const int dio2pin = 24; //18
const int dio3pin = 6; //31
const int dio4pin = 12; //32
const int dio5pin = 13; //33
unsigned long Message[32] = {	0xABCDEF12, //4 bytes = 1 word
								0x3456789A,
								0xBCDEF123,
								0x456789AB,
								0xCDEF1234,
								0x56789ABC,
								0xDEF12345,
								0x6789ABCD,
								0xEF123456,
								0x789ABCDE,
								0xF1234567,
								0x89ABCDEF,
								0x12345678,
								0x9ABCDEF1,
								0x23456789,
								0xABCDEF12,
								0x3456789A,
								0xBCDEF123,
								0x456789AB,
								0xCDEF1234,
								0x56789ABC,
								0xDEF12345,
								0x6789ABCD,
								0xEF123456,
								0x789ABCDE,
								0xF1234567,
								0x89ABCDEF,
								0x12345678,
								0x9ABCDEF1,
								0x23456789,
								0xABCDEF12,
								0x34567890 };

uint8_t currentMode = 0x09;
uint8_t nextByte = 0x00;
//boolean reading = 0;
int CurrentCount = 0, Word=0, Byte=0, state; 

void spi_send_byte(uint8_t Data1, uint8_t Data2) { 
    digitalWrite(24, LOW);
	uint8_t txbuf[2];
    txbuf[0] = (0x80 | Data1); //addr
	txbuf[1] = Data2;
    wiringPiSPIDataRW(0, txbuf, 2);
	digitalWrite(24, HIGH);
}
uint8_t spi_rcv_data(uint8_t Data) {
    digitalWrite(24, LOW);
	uint8_t rxbuf[2];
    rxbuf[0] = Data;
	rxbuf[1] = 0x00;
    wiringPiSPIDataRW(0, rxbuf, 2);
	digitalWrite(24, HIGH);
	return rxbuf[1];
}
uint8_t getByte() {
	uint8_t output;
	if (Byte == 4) {
		Byte = 0;
		Word++;
	}
	if (Word == 32) {
		printf("New Message");
	}
	output = (Message[Word] >> ((3-Byte)*8));	//take MSB
	Byte++;
	return output;
}
void dio1interrupt () { 	//FIFO Threshold FALLING
  
  printf("Fifo Threshold\n");
  while (digitalRead(dio2pin) == 0) {
	if (CurrentCount < 256) { //push it in    
		nextByte = getByte();
		spi_send_byte(0x00, nextByte);
		CurrentCount++;
	}
	else {
		printf("no more message.. program shouldn't come here\n");//shouldn't come here
		break;
	}
  }
}
void setInterrupts() {
  wiringPiISR (dio1pin, INT_EDGE_FALLING,  &dio1interrupt);
}
void setMode(uint8_t newMode)
{
  if(newMode == currentMode)
    return;  
  
  switch (newMode) 
  {
    case RFM98_MODE_SLEEP:
      printf("Changing to Sleep Mode\n"); 
      spi_send_byte(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    case RFM98_MODE_STANDBY:
      printf("Changing to Standby Mode\n");
      spi_send_byte(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
	case RFM98_MODE_FSTX:
      printf("Changing to FSTx Mode\n");
      spi_send_byte(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
	case RFM98_MODE_TX:
      printf("Changing to Tx Mode\n");
      spi_send_byte(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
	case RFM98_MODE_FSRX:
      printf("Changing to FSRx Mode\n");
      spi_send_byte(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
	case RFM98_MODE_RX:
      printf("Changing to Rx Mode\n");
      spi_send_byte(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    default: return;
  } 
  
  if(newMode != RFM98_MODE_SLEEP){ //test on ModeReady
    uint8_t testmode = spi_rcv_data(0x3E);
	while((testmode & 0x80) != 0x80)
    {
      testmode = spi_rcv_data(0x3E);
	  delay(500);
	  printf("Wait for it...\n");
    } 
  }
  printf("Mode Change Done\n");
  return;
}

void SetFSKMod()
{
  uint8_t cntMode;
  printf("Setting FSK Mode\n");
  setMode(RFM98_MODE_SLEEP);
  spi_send_byte(REG_BITRATEMSB, 0x00);
  spi_send_byte(REG_BITRATELSB, 0x68);
  spi_send_byte(REG_FDEVMSB, 0x00);
  spi_send_byte(REG_FDEVLSB, 0x52);
  spi_send_byte(REG_FRFMSB, 0x6C); //exact at 433Mhz
  spi_send_byte(REG_FRFMID, 0x40);
  spi_send_byte(REG_FRFLSB, 0x00);  
  printf("FSK Mode Set\n");
  cntMode = spi_rcv_data(REG_OPMODE);
  printf("Mode = "); 
  printf("%d", cntMode);
  printf("\n"); 
  return;
}

void Transmitter_Startup()
{
  //initialize
  setMode(RFM98_MODE_STANDBY);
  
  //transmitter settings
  spi_send_byte(REG_PACONFIG, 0x4B);
  spi_send_byte(REG_PARAMP, 0x09);
  spi_send_byte(REG_OCP, 0x00);
  
  //receiver settings
/*  spi_send_byte(REG_LNA, 0xC0);
  spi_send_byte(REG_RXCONFIG, 0x1E);
  spi_send_byte(REG_RSSICONFIG, 0x03);
  spi_send_byte(REG_RSSICOLLISION, 0x0A);  
  spi_send_byte(REG_RXBW, 0x09);
  spi_send_byte(REG_AFCFEI, 0x03);
  spi_send_byte(REG_PREAMBLEDETECT, 0xAA);
  spi_send_byte(REG_RXTIMEOUT1, 0x00);
  spi_send_byte(REG_RXTIMEOUT2, 0x00);
  spi_send_byte(REG_RXTIMEOUT3, 0x00);
  */
  //Basic Settings
  spi_send_byte(REG_OSC, 0x03); //@standby Clkout turned off
  spi_send_byte(REG_PREAMBLEMSB, 0x5A); //23205
  spi_send_byte(REG_PREAMBLELSB, 0xA5);
  spi_send_byte(REG_SYNCCONFIG, 0x98);
  spi_send_byte(REG_SYNCVALUE1, 0x48);
  spi_send_byte(REG_PACKETCONFIG1, 0x18);
  spi_send_byte(REG_PACKETCONFIG2, 0x40);
  spi_send_byte(REG_PAYLOADLENGTH, 0xFF);
  spi_send_byte(REG_FIFOTHRESH, 0x85);
  spi_send_byte(REG_IMAGECAL, 0x00); 
  spi_send_byte(REG_DIOMAPPING1, 0x01);
  spi_send_byte(REG_DIOMAPPING2, 0x31);
//  spi_send_byte(REG_PLLHOP, 0x00);
//  spi_send_byte(REG_PADAC, 0x04);
  //printf(digitalRead(dio5pin)); 	//check these values
  //printf(digitalRead(dio0pin));	//check these values
  setMode(RFM98_MODE_FSTX);
  setMode(RFM98_MODE_TX);	
}
void Tx() {
  uint8_t Irq2;
  
  if ((digitalRead(dio2pin) == 0) && (digitalRead(dio0pin) == 1))
	state = 1;
  else if ((digitalRead(dio2pin) == 0) && (digitalRead(dio0pin) == 0))
    state = 2;
  else if ((digitalRead(dio1pin) == 1) && (digitalRead(dio0pin) == 0))
    state = 3;
  else if ((digitalRead(dio1pin) == 0) && (digitalRead(dio0pin) == 0))
    state = 4; //run interrupt
  else if ((digitalRead(dio1pin) == 1) && (digitalRead(dio0pin) == 1))
    state = 5;
  else if ((digitalRead(dio1pin) == 0) && (digitalRead(dio0pin) == 1))
    state = 6;	
  
  switch (state) {
  case 1:
	printf("Case 1 Triggered\n");
	state = 5;
	printf("state transition from 1 to 5\n");
	break;
  case 2:
	printf("Case 2 Triggered\n");
	while (digitalRead(dio2pin)){
	}
	state = 3;
	printf("state transition from 2 to 3");
	break;
  case 3:
	printf("Case 3 Triggered\n");	
	if (digitalRead(dio1pin) == 0) {
		state = 4;
		printf("state transition from 3 to 4");
	}
	break;
  case 4:
	printf("Case 4 Triggered\n");
	//wait for interrupt trigger
	//interrupt should make me skip this code
	while (digitalRead(dio2pin) == 0) {
	if (CurrentCount < 256) { //push it in    
		nextByte = getByte();
		spi_send_byte(0x00, nextByte);
		CurrentCount++;
	}
	else {
		printf("no more message.. program shouldn't come here\n");//shouldn't come here
		break;
	}
  }printf("state transition from 4 to 1"); 
	break;
  case 5:
	printf("Case 5 Triggered\n");
	while (digitalRead(dio1pin)){ //while we are still more than threshold
	}
	state = 6;
	printf("state transition from 5 to 6");
	break;
  case 6:
	printf("Case 6 Triggered\n");
	Irq2 = spi_rcv_data(REG_IRQFLAGS2);
	if ((Irq2 & 0x40) == 0x40) {
		state = 7;
		printf("state transition from 6 to 7");
	}
	break;
  case 7:
	printf("Case 7 Triggered\n");
	printf("Prepare next Message\n");
	CurrentCount = 0;
	break;
  }
}

int setRFM98W(void)
{
	// initialize the pins
	int pisetupbit;
	pinMode( SSpin, OUTPUT);
	pinMode(dio0pin, INPUT);
	pinMode(dio1pin, INPUT);
	pinMode(dio2pin, INPUT);
	pinMode(dio3pin, INPUT);
	pinMode(dio4pin, INPUT);
	pinMode(dio5pin, INPUT);
	setInterrupts();
	if ((pisetupbit = wiringPiSPISetup(0, 8000000))<0)
		return -1;
	SetFSKMod();
	//testCommunication();
	Transmitter_Startup();
	return 0;
}
void setup() {
  printf("Balloon Initializing...");
  setRFM98W();
  printf("Setup Complete");
}
int main(void) { //int argc, char *argv[]
	int i;
	wiringPiSetup();
	setup();
	while (1){
	//for (i=0;i<5;i++)	
		Tx();   
	}
	return 0;
}