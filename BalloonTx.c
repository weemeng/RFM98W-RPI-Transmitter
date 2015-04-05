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
#define REG_FDEVLSB					0x05	//0x52 //Set to 5Khz
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
#define REG_PLLHOP					0x44
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

const int SSpin = 24; 
const int dio0pin = 29;
const int dio1pin = ; //get this done 
const int dio3pin = 31;
const int dio4pin = 32;
const int dio5pin = 33; 
unsigned char Message[256] = {0xABCDEF123456789ABCDEF123456789ABCDEF123456789ABCDEF123456789ABCDEF123456789ABCDEF123456789ABCDEF123456789ABCDEF123456789ABCDEF123456789ABCDEF123456789ABCDEF123456789ABCDEF123456789ABCDEF123456789ABCDEF123456789ABCDEF123456789ABCDEF123456789ABCDEF1234567890};
uint8_t currentMode = 0x09;
//boolean reading = 0;
int CurrentCount = 0, Bytes, state; 

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
	uint8_t rxbuf[1];
    rxbuf[0] = Data;
    wiringPiSPIDataRW(0, rxbuf, 1);
	digitalWrite(24, HIGH);
	return rxbuf[0];
}
/*void dio0interrupt () {		//PAYLOAD READY on RISING
  printf("Packet Sent\n");
}*/
void dio1interrupt () { 	//FIFO Threshold FALLING
  printf("Fifo Threshold\n");
  while (digitalRead(dio2pin) == 0) {
	if (CurrentCount < 256) { //push it in
		spi_send_byte(0x00, (uint8_t) Message[CurrentCount]);
		CurrentCount++;
	}
	else {
		printf("no more message.. program shouldn't come here\n");//shouldn't come here
		break;
	}
  }
}
/*void dio3interrupt () { 	//FIFO EMPTY either low or high
  printf("Tx Ready\n");
}
void dio4interrupt () { 	//Preamble Detect on RISING
  //might need to make a condition to avoid certain states
  printf("Temp Changed to some weird temperature\n");
}*/
void setInterrupts() {
  //wiringPiISR (dio0pin, INT_EDGE_RISING,  &dio0interrupt);
  wiringPiISR (dio1pin, INT_EDGE_FALLING,  &dio1interrupt);
  //wiringPiISR (dio3pin, INT_EDGE_BOTH,  &dio3interrupt);
  //wiringPiISR (dio4pin, INT_EDGE_RISING,  &dio4interrupt);
}
void setMode(uint8_t newMode)
{
  if(newMode == currentMode)
    return;  
  
  switch (newMode) 
  {
    case RF98M_MODE_SLEEP:
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
  
  if(newMode != RF98M_MODE_SLEEP){ //test on ModeReady
    while(digitalRead(dio5pin) == 0)
    {
      printf("Wait for it...\n");
    } 
  }
  printf("Mode Change Done\n");
  return;
}

void SetFSKMod()
{
  printf("Setting FSK Mode\n");
  setMode(RF98M_MODE_SLEEP);
  spi_send_byte(REG_BITRATEMSB, 0x00);
  spi_send_byte(REG_BITRATELSB, 0x68);
  spi_send_byte(REG_FDEVMSB, 0x00);
  spi_send_byte(REG_FDEVLSB, 0x52);
  spi_send_byte(REG_FRFMSB, 0x6C); //exact at 433Mhz
  spi_send_byte(REG_FRFMID, 0x40);
  spi_send_byte(REG_FRFLSB, 0x00);
   
  printf("FSK Mode Set\n");
  uint8_t cntMode = spi_rcv_data(REG_OPMODE);
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
  spi_send_byte(REG_PACONFIG, 0xFF);
  spi_send_byte(REG_PARAMP, 0x4C);
  spi_send_byte(REG_OCP, 0x2D);
  
  //receiver settings
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
  spi_send_byte(REG_IMAGECAL, 0x41);
  spi_send_byte(REG_DIOMAPPING1, 0x01);
  spi_send_byte(REG_DIOMAPPING2, 0x31);
  spi_send_byte(REG_PLLHOP, 0x00);
  spi_send_byte(REG_PADAC, 0x04);
  //printf(digitalRead(dio5pin)); 	//check these values
  //printf(digitalRead(dio0pin));	//check these values
  setMode(RFM98_MODE_FSTX);
  setMode(RFM98_MODE_TX);	
}
/*
int receiveMessage(char *message, int i)
{
  int Package = 256;
  
  while (digitalRead(dio3pin) == 0) {	//Fifo isnt empty
	if (i < Package) {
	  message[i] = (unsigned char)spi_rcv_data(REG_FIFO);
	}
  }
  message[i+1] = '\0';
  return i+1;
}  
*/
void Tx() {
  uint8_t Irq2;
  unsigned char onebyte;
  

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
			spi_send_byte(0x00, (uint8_t) Message[CurrentCount]);
			CurrentCount++;
		}
		else {
			printf("no more message.. program shouldn't come here\n");//shouldn't come here
			break;
	    }
	}
	printf("state transition from 4 to 1"); 
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

  	
  
  
  
  
  
  /*
  if  ((digitalRead(dio0pin) == 0) && (digitalRead(dio3pin) == 0))
	state = 1; 	//payload is not ready and fifo is not ready
  else if ((digitalRead(dio0pin) == 1) && (digitalRead(dio3pin) == 0))
	state = 2; 	//payload is ready and fifo is not ready
  else if ((digitalRead(dio0pin) == 1) && (digitalRead(dio3pin) == 1))
	state = 3; 	//payload is ready and fifo is ready
  else
    state = 4;  //settled by interrupt
  
  switch (state) {
  case 1:
	printf("Case 1 Triggered\n");
	Bytes = receiveMessage(Message, CurrentCount);
	state = 3;
	printf("state transition from 1 to 3\n");
	break;
  case 2:
	printf("Case 2 Triggered\n");
	Bytes = receiveMessage(Message, CurrentCount);
	state = 3;
	printf("state transition from 2 to 3");
	break;
  case 3:
	printf("Case 3 Triggered\n");
	//check CRC okay, Reset and load to SD card
	for (k = 0; k < Bytes; k++) {
	  printf("%d", Message[k]);
	  if (k == Bytes-1)
		printf("\n");
	}
	printf("\n");
	Bytes = 0;
	printf("wait for new packet...\n");
	break;
  case 4:
	printf("Case 4 Triggered\n");
	//wait for interrupt trigger
	if (Bytes == 0) 
	  printf("Think I'm waiting for some data..\n");
	else
	  printf("Think there is some transmission problem, please wait\n"); //dont think it will reach here
	break;
  }
*/
}

void setRFM98W(void)
{
	// initialize the pins
	int pisetupbit;
	pinMode( SSpin, OUTPUT);
	pinMode(dio0pin, INPUT);
	pinMode(dio3pin, INPUT);
	pinMode(dio4pin, INPUT);
	pinMode(dio5pin, INPUT);
	setInterrupts();
	if ((pisetupbit = wiringPiSPISetup(0, 1000000))<0)
		return -1;
	SetFSKMod();
	//testCommunication();
	Transmitter_Startup();
}
void setup() {
  printf("Balloon Initializing...");
  setRFM98W();
  printf("Setup Complete");
}
int main(void) { //int argc, char *argv[]
	//int i, j, k;
	wiringPiSetup();
	setup();
	while (1){
		SendTx();   
	}
	return;
}