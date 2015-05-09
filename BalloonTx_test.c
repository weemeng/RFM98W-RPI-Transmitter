//RPI for RFM98W
//by Ng Wee Meng
  
#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <wiringSerial.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
//#include <fcntl.h>
//#include <sys/ioctl.h>
//#include <linux/spi/spidev.h>
  
  
#define REG_FIFO                    0x00    //okay
#define REG_OPMODE                  0x01    //ever changing
#define REG_BITRATEMSB              0x02    //0x00 fixed
#define REG_BITRATELSB              0x03    //0x6B fixed
#define REG_FDEVMSB                 0x04    //0x04
#define REG_FDEVLSB                 0x05    //0x00 62.5khz
#define REG_FRFMSB                  0x06    //0x6C
#define REG_FRFMID                  0x07    //0x40
#define REG_FRFLSB                  0x08    //0x00
  
#define REG_PACONFIG                0x09    //0x4B 0-100-1011 //0xCF 11001111
#define REG_PARAMP                  0x0A    //0x09 00001001
#define REG_OCP                     0x0B    //0x0B 00001011 //0x2B
  
// for Receiver
/*
#define REG_LNA                     0x0C    //0xC0 1100-0000
#define REG_RXCONFIG                0x0D    //0x1E 00011-110
#define REG_RSSICONFIG              0x0E    //00000011
#define REG_RSSICOLLISION           0x0F    //0x0A
#define REG_RSSITHRESH              0x10    //dont care for now
#define REG_RSSIVALUE               0x11    //read
#define REG_RXBW                    0x12    //00001001 0x09 InitialConfig as AfcAutoOn=1
#define REG_AFCBW                   0x13    //read
#define REG_AFCFEI                  0x1A    //00000000 trigger
#define REG_AFCMSB                  0x1B    //auto
#define REG_AFCLSB                  0x1C    //auto
#define REG_FEIMSB                  0x1D    //read
#define REG_FEILSB                  0x1E    //read
#define REG_PREAMBLEDETECT          0x1F    //101-01010
#define REG_RXTIMEOUT1              0X20
#define REG_RXTIMEOUT2              0X21
#define REG_RXTIMEOUT3              0X22
*/
// Oscillator
#define REG_OSC                     0x24    //0x07 clockout off
  
// Packet Handling
#define REG_PREAMBLEMSB             0x25    //0x00
#define REG_PREAMBLELSB             0x26    //0x64 send 100 preamble
#define REG_SYNCCONFIG              0x27    //0x18 00011000
#define REG_SYNCVALUE1              0x28    //0x48
#define REG_SYNCVALUE2              0x29
#define REG_SYNCVALUE3              0x2A
#define REG_SYNCVALUE4              0x2B
#define REG_SYNCVALUE5              0x2C
#define REG_SYNCVALUE6              0x2D
#define REG_SYNCVALUE7              0x2E
#define REG_SYNCVALUE8              0x2F
#define REG_PACKETCONFIG1           0x30    //0x00 00000000
#define REG_PACKETCONFIG2           0x31    //0x40 01000-000//111
#define REG_PAYLOADLENGTH           0x32    //change to 7FF - 2047 bytes
#define REG_FIFOTHRESH              0x35    //0x85 //not 0xBC 1011 1100 
  
//  Sequencer
#define REG_SEQCONFIG1              0x36
#define REG_SEQCONFIG2              0x37
  
#define REG_IMAGECAL                0x3B    //0xC2
#define REG_TEMP                	0x3C    //read 
#define REG_IRQFLAGS1               0x3E    //trigger
#define REG_IRQFLAGS2               0x3F    //trigger
#define REG_DIOMAPPING1             0x40    //00-00-00-00
#define REG_DIOMAPPING2             0x41    //0x31 00-11-000-1
#define REG_REGVERSION              0x42    //read
//#define REG_PLLHOP                0x44
#define REG_PADAC               0x4D    //0x07 00000100 //regular boost
//#define REG_BITRATEFRAC           0x5D    //super minute accuracy which can be accounted for by the doppler shift
#define REG_AGCREFLF                0x61
#define REG_AGCTHRESHLF1            0x62
#define REG_AGCTHRESHLF2            0x63
#define REG_AGCTHRESHLF3            0x64
      
//MODES
#define RFM98_MODE_SLEEP            0x08    
#define RFM98_MODE_STANDBY          0x09    //00001001
#define RFM98_MODE_FSTX             0x0A    //00001010
#define RFM98_MODE_TX               0x0B    //00001011
#define RFM98_MODE_FSRX             0x0C    //00001100
#define RFM98_MODE_RX               0x0D    //00001101
  
const int PacketSize = 64; //change at payloadlength
const int Packetaddlast = 63;
const int Packetaddm1 = 62; 

const int LEDIndicationpin = 27;
 
// RFM98W IO Pins
const int SSpin = 24; 
const int dio0pin = 21;  //WiringPi Pin 
const int dio1pin = 4; 
const int dio2pin = 5; 
const int dio3pin = 22; 
const int dio4pin = 26; 
const int dio5pin = 23;
 
uint8_t currentMode = 0x09;
uint8_t nextByte = 0x00;
int plusone = 1;
int  max_Image_Packet_Count= 0, Image_Packet_Count = 0, Buffer_Count = 0; 
int state, packetfinished = 0, imagefinished = 0;
int  Packet_Byte_Count= 0; 
int Word = 0, Byte = 0;
uint8_t Redundancy = 0x00;
 
// Raspberry Pi Camera
char imagename[15];
FILE * pFile;
long lSize = 0;
unsigned char * buffer;
size_t result;
uint32_t file_byte_size[0];
 
// GPS
int fd = 0;
uint8_t GPSBuffer[82] = {0};
uint8_t GPSIndex=0;
// GPS Variables
unsigned char GPS_Time[7] = "000000";
unsigned char GPS_Date[7] = "000000";
unsigned int GPS_Latitude_Degrees=0, GPS_Longitude_Degrees=0, GPS_Altitude=0;
double GPS_Latitude_Minutes=0, GPS_Longitude_Minutes=0;
char *GPS_LatitudeSign="";
char *GPS_LongitudeSign="";
int GPS_AltitudeSign=0; // 0 is positive
uint8_t GotGPSThisSentence = 0; //validity
//unsigned int PreviousAltitude=0;
unsigned int GPS_Satellites=0;
char Hex[] = "0123456789ABCDEF";
int lat_minute=0, long_minute=0;
double lat_second=0, long_second=0;
  
int Message[31] = {0x12345678,
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
					0x89ABCDEF};
 
void spi_send_byte(uint8_t Data1, uint8_t Data2) { 
    digitalWrite(24, LOW);
    uint8_t txbuf[2];
    txbuf[0] = (0x80 | Data1); //addr
    txbuf[1] = Data2;
    wiringPiSPIDataRW(0, txbuf, 2);
    printf("I am using register %d and i am setting it to %d\n", Data1, Data2);
    digitalWrite(24, HIGH);
}
uint8_t spi_rcv_data(uint8_t Data) {
    digitalWrite(24, LOW);
    uint8_t rxbuf[2];
    rxbuf[0] = Data;
    rxbuf[1] = 0x00;
    wiringPiSPIDataRW(0, rxbuf, 2);
    printf("I am reading register %d and i am getting %d\n", Data, rxbuf[1]);
    digitalWrite(24, HIGH);
    return rxbuf[1];
}
uint8_t getByte() {
    uint8_t output;
    if (Buffer_Count < lSize-1) {
        output = buffer[Buffer_Count];
        Buffer_Count++;
    }
    else
        output = 0x00; //fill with 0's and see how this goes
    return output;
}
void arrangePacket() {
    while (digitalRead(dio2pin) == 0) { //while Fifo isnt full
//      delay(100);
        if (Image_Packet_Count <= max_Image_Packet_Count) { //cause of residual bytes
            if (Packet_Byte_Count < Packetaddm1) { //push it in || max = PacketSize    
                nextByte = getByte();
                printf("This is byte %d ------- ", Packet_Byte_Count);
//              if (Packet_Byte_Count == Packetaddm1)
//                  nextByte = nextByte + plusone;
//              plusone++;
                spi_send_byte(0x00, nextByte);
//              delay(10); //might remove this when going higher freq
                Packet_Byte_Count++;
            }
            else {
                spi_send_byte(0x00, Redundancy); //byte Packetaddm1
                spi_send_byte(0x00, (unsigned int) Image_Packet_Count); //byte Packetaddlast
                Redundancy++;
                packetfinished = 1;
                if (Redundancy == 1) {
                    Image_Packet_Count++;
                    Redundancy = 0;
                }
                else
                    Buffer_Count = Buffer_Count - Packetaddm1; //0 to Packetaddm1
                printf("Packet finished sending\n");
                break;
            }
        }
        else
			printf("Broke from here");
			break;
    }
}
void dio1interrupt () {   //FIFO Threshold FALLING
  int curr;
  curr = millis();
  printf("time = %d\n", curr);
  printf("Running Dio1 interrupt\n");
  printf("Fifo Threshold interrupt\n");
  arrangePacket();              //might have to disable interrupt here
  printf("Getting out of interrupt\n");
  curr = millis();
  printf("time = %d", curr);
  return;
}
int setInterrupts() {
  if (wiringPiISR (dio1pin, INT_EDGE_FALLING, &dio1interrupt) < 0)
  {
    fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno)) ;
    delay(10000);
    return 1 ;
  }
  printf("Interrupts set up\n");
  return 0;
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
//      delay(500);
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
  spi_send_byte(REG_BITRATEMSB, 0x80); //00
  spi_send_byte(REG_BITRATELSB, 0x00); //6B
  spi_send_byte(REG_FDEVMSB, 0x06); //9
  spi_send_byte(REG_FDEVLSB, 0x66); //99
  spi_send_byte(REG_FRFMSB, 0x6C); //exact at 433Mhz
  spi_send_byte(REG_FRFMID, 0x9C);
  spi_send_byte(REG_FRFLSB, 0x8E);  
  printf("FSK Mode Set\n");
  cntMode = spi_rcv_data(REG_OPMODE);
  printf("Mode = "); 
  printf("%d", cntMode);
  printf("\n"); 
  return;
}
void sendInitialisingBits() { //send initial sequence including 
    int used = 25, fill;
	printf("GOT HERE!!!\n");
    while (digitalRead(dio2pin) == 0) { //while Fifo isnt full
        //byte padding for recognition at receiver
		spi_send_byte(0x00, 0xE1); //EL
		spi_send_byte(0x00, 0x10); //L0
		spi_send_byte(0x00, 0x59); //SP
		spi_send_byte(0x00, 0xAC); //AC
		spi_send_byte(0x00, 0xEC); //EC
		spi_send_byte(0x00, 0x1A); //IA
		spi_send_byte(0x00, 0x11); //LI
		spi_send_byte(0x00, 0x57); //ST		
		
		* file_byte_size = (uint32_t) lSize; 
        spi_send_byte(0x00, file_byte_size[0]>>16); //24bit length
        spi_send_byte(0x00, file_byte_size[0]>>8);
        spi_send_byte(0x00, file_byte_size[0]);
		
		//GPS
		//Time
		spi_send_byte(0x00, (uint8_t) ((GPS_Time[0]-'0')<<4 | (GPS_Time[1]-'0'));
		spi_send_byte(0x00, (uint8_t) ((GPS_Time[2]-'0')<<4 | (GPS_Time[3]-'0'));
		spi_send_byte(0x00, (uint8_t) ((GPS_Time[4]-'0')<<4 | (GPS_Time[5]-'0'));
		
		//Latitude
		if (*GPS_LatitudeSign == 'N')
			spi_send_byte(0x00, 0x00);
		else
			spi_send_byte(0x00, 0xFF);
		spi_send_byte(0x00, (uint8_t) GPS_Latitude_Degrees);
		spi_send_byte(0x00, (uint8_t) lat_minute);
		spi_send_byte(0x00, (uint8_t) lat_second);
		//Longitude
		if (*GPS_LongitudeSign == 'E')
			spi_send_byte(0x00, 0x00);
		else
			spi_send_byte(0x00, 0xFF);
		spi_send_byte(0x00, (uint8_t) GPS_Longitude_Degrees);
		spi_send_byte(0x00, (uint8_t) long_minute);
		spi_send_byte(0x00, (uint8_t) long_second);
		//Altitude
		if (GPS_AltitudeSign == 0)
			spi_send_byte(0x00, 0x00); //positive
		else
			spi_send_byte(0x00, 0xFF); //negative
		spi_send_byte(0x00, (uint32_t) GPS_Altitude>>24);
		spi_send_byte(0x00, (uint32_t) GPS_Altitude>>16);		
		spi_send_byte(0x00, (uint32_t) GPS_Altitude>>8);
		spi_send_byte(0x00, (uint32_t) GPS_Altitude);
		
		//Temp
		spi_send_byte(0x00, spi_rcv_data(REG_TEMP));
		
        printf("GOT HERE2!!!\n");
        printf("Actual Size is %ld\n", lSize); // divide by Packetaddmi +1 for total packet for 1 image
        //printf("Byte Form Size is %x\n", (uint8_t) lSize); //byte form
        printf("Byte Form is %x\n", (uint16_t) lSize);
        printf("Sending Initialising Bytes...\n");
        for (fill = used; fill < PacketSize; fill++) 
			spi_send_byte(0x00, 0x00);
		//delay(3000);
        break;
		
    }
	Image_Packet_Count = max_Image_Packet_Count - 2;
    return;
}  
void sendEndImagePacket () {
    int endpackcount = 0;
    uint8_t endoutput;
	
	Byte = 0;
	Word = 0;
	
	setMode(RFM98_MODE_FSTX);
    spi_send_byte(0x00, 0xE1); //EL
	spi_send_byte(0x00, 0x10); //L0
	spi_send_byte(0x00, 0x59); //SP
	spi_send_byte(0x00, 0xAC); //AC
	spi_send_byte(0x00, 0xEC); //EC
	spi_send_byte(0x00, 0x1A); //IA
	spi_send_byte(0x00, 0x11); //LI
	spi_send_byte(0x00, 0x57); //ST		
	for (endpackcount = 0; endpackcount < PacketSize-8; endpackcount++) {
        if (Byte == 4) {
            Byte = 0;
            Word++;
        }
        if ((Word == 15) && (Byte == 3))  {
            //printf("New Message");
            Word = 0;
            Byte = 0;
        }
        endoutput = (Message[Word]>>((3-Byte)*8)); //fill with 0's and see how this goes
        Byte++;
        spi_send_byte(0x00, endoutput);
        //might need to reset the endpackcount  
    }
	
	setMode(RFM98_MODE_TX);
	while (digitalRead(dio0pin) == 0) { //wait for packetsent
	} //seems dangerous
}
void Transmitter_Startup()
{
  //initialize
  setMode(RFM98_MODE_STANDBY);
    
  //transmitter settings
  spi_send_byte(REG_PACONFIG, 0xCF);
  spi_send_byte(REG_PARAMP, 0x29);
  spi_send_byte(REG_OCP, 0x2B);
    
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
  spi_send_byte(REG_OSC, 0x07); //@standby Clkout turned off
  spi_send_byte(REG_PREAMBLEMSB, 0x00); 
  spi_send_byte(REG_PREAMBLELSB, 0x64);
  spi_send_byte(REG_SYNCCONFIG, 0x17);
  spi_send_byte(REG_SYNCVALUE1, 0x28);
  spi_send_byte(REG_SYNCVALUE2, 0x39);
  spi_send_byte(REG_SYNCVALUE3, 0x4A);
  spi_send_byte(REG_SYNCVALUE4, 0x5B);
  spi_send_byte(REG_SYNCVALUE5, 0x6C);
  spi_send_byte(REG_SYNCVALUE6, 0x7D);
  spi_send_byte(REG_SYNCVALUE7, 0x8E);
  spi_send_byte(REG_SYNCVALUE8, 0x9F);
  spi_send_byte(REG_PACKETCONFIG1, 0x50);
  spi_send_byte(REG_PACKETCONFIG2, 0x40);
  spi_send_byte(REG_PAYLOADLENGTH, 0x40); //FF
  spi_send_byte(REG_FIFOTHRESH, 0x85);
  spi_send_byte(REG_IMAGECAL, 0xC2); 
  spi_send_byte(REG_DIOMAPPING1, 0x00);
  spi_send_byte(REG_DIOMAPPING2, 0x31);
//  spi_send_byte(REG_PLLHOP, 0x00);
  spi_send_byte(REG_PADAC, 0x07);
  //printf(digitalRead(dio5pin));   //check these values
  //printf(digitalRead(dio0pin));   //check these values   
  return;
}
  
void Tx() {

	sendInitialisingBits();
	setMode(RFM98_MODE_FSTX);
	setMode(RFM98_MODE_TX);

	while (digitalRead(dio0pin) == 0) {
		//delay(4000);
		if ((digitalRead(dio3pin) == 1) && (digitalRead(dio1pin) == 0)) {
			arrangePacket();
		}
	}
	
	if (digitalRead(dio0pin) == 1) {
		if (packetfinished==1) {
			setMode(RFM98_MODE_FSTX);
			Packet_Byte_Count = 0;
			packetfinished = 0;
			//delay(100);
			setMode(RFM98_MODE_TX);
		}
		printf("SIAOLIAO NEXT Packet");
	}
	return;
  //dio2pin = FIFO FULL //dio0pin packet sent //dio1pin Fifo Threshold //dio3pin fifioempty
  /*if ((digitalRead(dio2pin) == 1) && (digitalRead(dio0pin) == 1)) 
    state = 1;  //FIFO full and packet sent
  else if ((digitalRead(dio2pin) == 1) && (digitalRead(dio0pin) == 0)) 
    state = 2;  //FIFO full and packet not sent
  else if ((digitalRead(dio3pin) == 1) && (digitalRead(dio0pin) == 1)) 
    state = 3;  //FIFO empty and packet sent 
  else if ((digitalRead(dio3pin) == 1) && (digitalRead(dio0pin) == 0)) 
    state = 4;  //FIFO empty and packet not sent 
  else if ((digitalRead(dio1pin) == 1) && (digitalRead(dio0pin) == 0)) 
    state = 5;  //FIFO level above threshold and packet not sent 
  else if ((digitalRead(dio1pin) == 0) && (digitalRead(dio0pin) == 0)) 
    state = 6;  //FIFO level below threshold and packet not sent 
  else if ((digitalRead(dio1pin) == 1) && (digitalRead(dio0pin) == 1)) 
    state = 7;  //FIFO level above threshold and packet sent 
  else if ((digitalRead(dio1pin) == 0) && (digitalRead(dio0pin) == 1)) 
    state = 8;  //FIFO level below threshold and packet sent
  
  switch (state) {
  case 1:
    printf("Case 1 Triggered\n");
    state = 7;
    printf("state transition from 1 to 7\n");
    break;
  case 2:
    printf("Case 2 Triggered\n");
    while (digitalRead(dio2pin)){ //while fifo full, wait for buffer to not be full...
    }
    state = 5;
    printf("state transition from 2 to 5\n");
    break;
  case 3:
    printf("Case 3 Triggered\n");   
    //prepare next message and reset the packetsent (by exiting Tx)
    printf("End of test package\n");
    printf("%d", spi_rcv_data(0x3F));
   if (packetfinished==1) {
        setMode(RFM98_MODE_FSTX);
        Packet_Byte_Count = 0;
        packetfinished = 0;
        //delay(100);
        setMode(RFM98_MODE_TX);
        state = 4;
        printf("state transition from 3 to 4\n");
    }
    break;
  case 4:
    printf("Case 4 Triggered\n");
    if (packetfinished == 0) { //assuming arrangePacket will fill the buffer to PacketSize though this shouldn't matter as it will change state in the next loop.
        arrangePacket();
        state = 2; //tentatively should go to 2 but can go to 5
        printf("state transition from 4 to 2\n");
    }
    else {
        while(digitalRead(dio0pin) == 0){
        }
        state = 3; //tentatively should go to 2 but can go to 5
        printf("state transition from 4 to 3\n");
    }
    break;
  case 5:
    printf("Case 5 Triggered\n");
    while (digitalRead(dio1pin)){ //while we are still more than threshold
    }
    if (digitalRead(dio1pin) == 0) {
        state = 6;
        printf("state transition from 5 to 6\n");
    }
    break;
  }
  */
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
    //setInterrupts();
    if ((pisetupbit = wiringPiSPISetup(0, 8000000))<0)
        return -1;
    SetFSKMod();
    //testCommunication();
    Transmitter_Startup();
    return 0;
}
uint8_t GPSChecksumOK() {
  uint8_t XOR, i, c;
  XOR = 0;
  for (i = 1; i < (GPSIndex-4); i++) {
    c = GPSBuffer[i];
    XOR ^= c;
  }
  return (GPSBuffer[GPSIndex-4] == '*') && (GPSBuffer[GPSIndex-3] == Hex[XOR >> 4]) && (GPSBuffer[GPSIndex-2] == Hex[XOR & 15]);
}
 
void ProcessGPRMCCommand()
{
  int i, j, k;
  double Divider = 1;
   
  // $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
  // 0   220516     Time Stamp
  // 1   A          validity - A-ok, V-invalid
  // 2   5133.82    current Latitude
  // 3   N          North/South
  // 4   00042.24   current Longitude
  // 5   W          East/West
  // 6   130694     Date Stamp
   
  for (i=7, j=0, k=0; (i<GPSIndex) && (j<8); i++) {
    if (GPSBuffer[i] == ',') {
      j++;    // Segment index
      k=0;    // Index into target variable
    }
    else {
        switch (j) {
        case 0: // UTC Time
            if (k < 6) {
				GPS_Time[k++] = GPSBuffer[i];
//				GPS_Time[k] = 0;
            }
            break;
        case 1:
            // Validity
            if (GPSBuffer[i] == 'A')
				GotGPSThisSentence = 1;
            else
				GotGPSThisSentence = 0;
            GPS_Latitude_Degrees = 0;
            GPS_Latitude_Minutes = 0;
            GPS_Longitude_Degrees = 0;
            GPS_Longitude_Minutes = 0;
            GPS_LongitudeSign = "E";
			GPS_LatitudeSign = "N";
            GPS_Altitude = 0;
			GPS_AltitudeSign = 0;
            GPS_Satellites = 0;
            break;
 
        case 2:         // Latitude
          if (k <= 1) {
            GPS_Latitude_Degrees = GPS_Latitude_Degrees * 10 + (int)(GPSBuffer[i] - '0');
            Divider = 1;
          }
          else if ( k != 4) {
            Divider = Divider * 10;
            GPS_Latitude_Minutes = GPS_Latitude_Minutes  + (double)(GPSBuffer[i] - '0') / Divider ; //
          }
          k++;
          break;
 
        case 3:
          if (k < 1) {          // N or S
            // Latitude = GPS_Latitude_Degrees + GPS_Latitude_Minutes * 5 / 30000;
            if (GPSBuffer[i] == 'S')  {
				GPS_LatitudeSign = "S"; // -
            }
            else
				GPS_LatitudeSign = "N";
          }
          break;
        case 4:         // Longitude
          if (k <= 2) {
            GPS_Longitude_Degrees = GPS_Longitude_Degrees * 10 + (int)(GPSBuffer[i] - '0');
            Divider = 1;
          }
          else if ( k != 5) {
            Divider = Divider * 10;
            GPS_Longitude_Minutes = GPS_Longitude_Minutes + (double)(GPSBuffer[i] - '0') / Divider;
          }
          k++;
          break;  // Start bit
 
        case 5:
          // E or W
			if (k < 1) {
				if (GPSBuffer[i] == 'W')
					GPS_LongitudeSign = "W"; //-
				else
					GPS_LongitudeSign = "E";
			}
			break;  // Start bit
		
		case 7: //Date
            if (k < 6) {
				GPS_Date[k++] = GPSBuffer[i] - '0';
            }			
		default:
			break;
      }
    }
  }
}
void ProcessGPGGACommand() {
  int i, j;
  // $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
  // 8    = Antenna altitude above/below mean sea level (geoid)
  for (i=7, j=0; (i<GPSIndex) && (j<11); i++) {
    if (GPSBuffer[i] == ',') {
      j++;    // Segment index
    }
    else {
      if (j == 6) {
        //Satellite Count
        if ((GPSBuffer[i] >= '0') && (GPSBuffer[i] <= '9')) {
          GPS_Satellites = GPS_Satellites * 10;
          GPS_Satellites += (unsigned int)(GPSBuffer[i] - '0');
        }
      }
      else if (j == 8) {
        //Altitude
	if (GPSBuffer[i] == '-')
	  GPS_AltitudeSign = 1;
	else if ((GPSBuffer[i] >= '0') && (GPSBuffer[i] <= '9')) {
		GPS_Altitude = GPS_Altitude * 10;
		GPS_Altitude += (unsigned int)(GPSBuffer[i] - '0');
        }   
      }
    }
  }
}
void ProcessGPSLine() {
  if (GPSChecksumOK()) {
    if ((GPSBuffer[1] == 'G') && (GPSBuffer[2] == 'P') && (GPSBuffer[3] == 'R') && (GPSBuffer[4] == 'M') && (GPSBuffer[5] == 'C')) {
      printf("-----------------------------------RMC\n");
      ProcessGPRMCCommand();
      if (GotGPSThisSentence == 1)
        printf("valid\n");
      else
        printf("invalid\n");
      printf("GPS TIME : %s\n", GPS_Time);
      printf("Latitude = %c", *GPS_LatitudeSign); //supposedly give minus if it is
      printf("%d\n", GPS_Latitude_Degrees);
      lat_minute = (int) (GPS_Latitude_Minutes*100)/1;
      printf("Minutes = %2d\n", lat_minute);
      lat_second = (((GPS_Latitude_Minutes*100)/1 - lat_minute)*60)/1;
      printf("seconds = %.0f\n", lat_second);
     
      printf("Longitude = %c", *GPS_LongitudeSign); //supposedly give minus if it is
      printf("%d\n", GPS_Longitude_Degrees);
      long_minute = (int) (GPS_Longitude_Minutes*100)/1;
      printf("Minutes = %2d\n", long_minute);
      long_second = (((GPS_Longitude_Minutes*100)/1 - long_minute)*60)/1;
      printf("seconds = %.0f\n", long_second);
    }
    if ((GPSBuffer[1] == 'G') && (GPSBuffer[2] == 'P') && (GPSBuffer[3] == 'G') && (GPSBuffer[4] == 'G') && (GPSBuffer[5] == 'A')) {
      printf("-----------------------------------GGA\n");
      ProcessGPGGACommand();
      printf("Satellite Count = %d\n", GPS_Satellites);
      printf("Altitude = %d", GPS_AltitudeSign); //supposedly give minus if it is
      printf("%d\n", GPS_Altitude/10);
    }
  }
} 
void getGPS() {
  int inByte;         // incoming serial byte
  // Check for GPS data
  while (serialDataAvail(fd) > 0) {
    inByte = serialGetchar(fd);
    if (inByte != '$') {    
      //printf("%2x", (char)inByte);
    }
    if ((inByte =='$') || (GPSIndex >= 80)) {
      GPSIndex = 0;
    }
    if (inByte != '\r') {
      GPSBuffer[GPSIndex++] = inByte;
    }
    if (inByte == '\n') {
      ProcessGPSLine();
      GPSIndex = 0;
    }
  }
}
void prepBuffer() {
	
    pFile = fopen (imagename, "rb" );
    if (pFile==NULL) {fputs ("File error",stderr); exit (1);}
    
    //free (buffer);
    // obtain file size:
    fseek (pFile , 0 , SEEK_END);
    lSize = ftell (pFile);
    rewind (pFile);
  
    // allocate memory to contain the whole file:
    buffer = (unsigned char*) malloc (sizeof(unsigned char)*lSize); //buffer will fill from 0 to lSize - 1
    if (buffer == NULL) {fputs ("Memory error",stderr); exit (2);}
      
    // copy the file into the buffer:
    result = fread (buffer,1,lSize,pFile);
    if (result != lSize) {fputs ("Reading error",stderr); exit (3);}
    printf("File opened, some byte values: %i %i %i %i\n", buffer[0], buffer[1], buffer[2], buffer[3]);
    /* the whole file is now loaded in the memory buffer. */
    fclose (pFile);
    //free (buffer);
    max_Image_Packet_Count = lSize/Packetaddm1; //256
    return;
}
void takingPicture() { //Use system commands to do that.
    char raspistring[200]; //im avoiding overwriting other data as this string is long
    sprintf(imagename,  "Stillpic%s.jpg", GPS_Time);
    printf("TakingPichere\n");
	printf("New File name is : Stillpic%s.jpg\n", GPS_Time);
    sprintf(raspistring, "raspistill -w 640 -h 480 -th none -e jpg -o %s -q 10 -x GPS.GPSAltitude=%d/10 -x GPS.GPSAltitudeRef=%d -x GPS.GPSLongitude=%d/1,%d/1,%.0f/1 -x GPS.GPSLongitudeRef=%c -x GPS.GPSLatitude=%d/1,%d/1,%.0f/1 -x GPS.GPSLatitudeRef=%c", imagename, GPS_Altitude, GPS_AltitudeSign, GPS_Longitude_Degrees, long_minute, long_second, *GPS_LongitudeSign, GPS_Latitude_Degrees, lat_minute, lat_second, *GPS_LatitudeSign);
    //system("mkdir BalloonCamera"); 																//check whether the altitude need to divide by 10    
    //system("cd BalloonCamera");                                         //doesnt work somehow
    system(raspistring); //"raspistill -w 800 -h 600 -e jpg -o Stillpic.jpg -q 10"
    prepBuffer();
    //system("cd");
    printf("finished");
    return;
}
void initialiseGPS(){
    if((fd = serialOpen("/dev/ttyAMA0", 9600)) < 0 ) /*device is the serial uart port in the RPi*/
        perror("device not opened \n");
    return;
}
void setPicture() {
    while (1) {
		digitalWrite(LEDIndicationpin, 1);
		delay(70);
		digitalWrite(LEDIndicationpin, 0);
		delay(70);
		getGPS();
		if ((GotGPSThisSentence == 1) && (GPS_Altitude > 0)) { //gives if gps_altitude is not 0 as sign is in another variable
			digitalWrite(LEDIndicationpin, 1);
			delay(500);
			digitalWrite(LEDIndicationpin, 0);
			delay(500);
			digitalWrite(LEDIndicationpin, 1);
			delay(500);
			digitalWrite(LEDIndicationpin, 0);
			delay(500);
			digitalWrite(LEDIndicationpin, 1);
			delay(500);
			digitalWrite(LEDIndicationpin, 0);
			delay(500);
			break; //need to check if Exif can go negative
        }
    }
    takingPicture();
}
void setup() {
  printf("Balloon Initializing...\n");
  initialiseGPS();
  setRFM98W();
  printf("Setup Complete\n");
}
int main(void) { //int argc, char *argv[]
    int newimage = 1;
    if ( wiringPiSetup () < 0 ) //Setup WiringPi
        perror("WiringPiSetup problem \n ");
    //takingPicture(); //fork out this command
    setup();
	//exit as standby
	setInterrupts();
    while (1){
		if (newimage == 1) {
			setPicture(); //GotGPSThisSentence
			newimage = 0;
			printf("SETPICTURE");
			
			delay(200);
		}
		else {
			Tx();   
			printf("This is the packetCount = %d\n", Image_Packet_Count);
			printf("This is the max packetCount = %d\n", max_Image_Packet_Count);
			if (Image_Packet_Count > max_Image_Packet_Count) {
				newimage = 1;
				Image_Packet_Count = 0;
				Buffer_Count = 0;
				sendEndImagePacket();
				 //might cause trouble if packet is big //set into a function
				setMode(RFM98_MODE_STANDBY);
				printf("Time for next picture");
			}
		}		
    }
    return 0;
}