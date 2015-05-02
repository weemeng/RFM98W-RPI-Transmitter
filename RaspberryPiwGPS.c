//  running on Arduino Uno R3 and GPS shield with Antenna. Ublox NEO-6M
//  at 9600 bps
//	GPS code adaptation for RASPBERRY PI

#include <wiringPi.h>
#include <wiringSerial.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <sys/types.h>

char buffer[256]; // buffer array for data recieve over serial port
int count=0, i, j, k, next;     // counter for buffer array 
int fd;

// GPS:
byte GPSBuffer[82];
byte GPSIndex=0;

// GPS Variables
unsigned long SendGPSConfig;
char GPS_Time[9] = "00:00:00";
unsigned int GPS_Latitude_Minutes, GPS_Longitude_Minutes;
double GPS_Latitude_Seconds, GPS_Longitude_Seconds;
char *GPS_LatitudeSign="";
char *GPS_LongitudeSign="";
float Longitude, Latitude;
unsigned int GPS_Altitude=0, MaximumAltitude=0, MaxAltitudeThisSentence=0;
byte GotGPSThisSentence=0;
unsigned int PreviousAltitude=0;
unsigned int GPS_Satellites=0;
unsigned int GPS_Speed=0;
unsigned int GPS_Direction=0;

char Hex[] = "0123456789ABCDEF";

void getGPS()
{
  int inByte;         // incoming serial byte
  
  // Check for GPS data
  while (serialDataAvail(fd) > 0)
  {
    inByte = serialGetchar(fd);
    
    if (inByte != '$')
    {	
      printf("%x", (char)inByte);
    }

    if ((inByte =='$') || (GPSIndex >= 80))
    {
      GPSIndex = 0;
    }
    
    if (inByte != '\r')
    {
      GPSBuffer[GPSIndex++] = inByte;
    }
    
    if (inByte == '\n')
    {
      ProcessGPSLine();
      GPSIndex = 0;
    }
  }
}

byte GPSChecksumOK()
{
  byte XOR, i, c;
  
  XOR = 0;
  for (i = 1; i < (GPSIndex-4); i++)
  {
    c = GPSBuffer[i];
    XOR ^= c;
  }
  
  return (GPSBuffer[GPSIndex-4] == '*') && (GPSBuffer[GPSIndex-3] == Hex[XOR >> 4]) && (GPSBuffer[GPSIndex-2] == Hex[XOR & 15]);
}

void ProcessGPSLine()
{
  if (GPSChecksumOK())
  {
    if ((GPSBuffer[1] == 'G') && (GPSBuffer[2] == 'P') && (GPSBuffer[3] == 'R') && (GPSBuffer[4] == 'M') && (GPSBuffer[5] == 'C'))
    {
      ProcessGPRMCCommand();
    }
    else if ((GPSBuffer[1] == 'G') && (GPSBuffer[2] == 'P') && (GPSBuffer[3] == 'G') && (GPSBuffer[4] == 'G') && (GPSBuffer[5] == 'A'))
    {
      ProcessGPGGACommand();
    }
  }
}
void ProcessGPRMCCommand()
{
  int i, j, k, IntegerPart;
  double Divider = 1;
  
  // $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
  // 0   220516     Time Stamp
  // 1   A          validity - A-ok, V-invalid
  // 2   5133.82    current Latitude
  // 3   N          North/South
  // 4   00042.24   current Longitude
  // 5   W          East/West
  // 6   130694     Date Stamp
  
  for (i=7, j=0, k=0; (i<GPSIndex) && (j<8); i++)
  {
    if (GPSBuffer[i] == ',')
    {
      j++;    // Segment index
      k=0;    // Index into target variable
      IntegerPart = 1;
    }
    else
    {
      switch (j)
      {
        case 0:
          // UTC Time
          if (k < 8)
          {
            GPS_Time[k++] = GPSBuffer[i];
            if ((k==2) || (k==5))
            {
              GPS_Time[k++] = ':';
            }
            GPS_Time[k] = 0;
          }
          break;  // Start bit
          
        case 1:
          // Validity
          if (GPSBuffer[i] == 'A') {
            // Message OK
            GPS_Latitude_Minutes = 0;
            GPS_Latitude_Seconds = 0;
            GPS_Longitude_Minutes = 0;
            GPS_Longitude_Seconds = 0;
            GPS_Speed = 0;
            GPS_Direction = 0;
            GPS_LongitudeSign = "-";  // new
          }
          else
            exit; 
          break;

        case 2:
          // Latitude
          if (k <= 1) {
            GPS_Latitude_Minutes = GPS_Latitude_Minutes * 10 + (int)(GPSBuffer[i] - '0');
            Divider = 1;
          }
          else if ( k != 4) {
            Divider = Divider * 10;
            GPS_Latitude_Seconds = GPS_Latitude_Seconds  + (double)(GPSBuffer[i] - '0') / Divider;
          }
          /*
          if (k < 9)
          {
            GPS_Latitude[k++] = GPSBuffer[i];
            GPS_Latitude[k] = 0;
          }
          */
          k++;
          break;  // Start bit

        case 3:
          // N or S
          if (k < 1) {
            // Latitude = GPS_Latitude_Minutes + GPS_Latitude_Seconds * 5 / 30000;
            if (GPSBuffer[i] == 'S')  {
              GPS_LatitudeSign = "-";
            }
            else
			  GPS_LatitudeSign = "";
          }
          break;  // Start bit

        case 4:
          // Longitude
          if (k <= 2) {
            GPS_Longitude_Minutes = GPS_Longitude_Minutes * 10 + (int)(GPSBuffer[i] - '0');
            Divider = 1;
          }
          else if ( k != 5) {
            Divider = Divider * 10;
            GPS_Longitude_Seconds = GPS_Longitude_Seconds + (double)(GPSBuffer[i] - '0') / Divider;
          }
          /*
          if (k < 10)
          {
            GPS_Longitude[k++] = GPSBuffer[i];
            GPS_Longitude[k] = 0;
          }
          */
          k++;
          break;  // Start bit

        case 5: //might have prob
          // E or W
          if (k < 1) {
            if (GPSBuffer[i] == 'E') {
              GPS_LongitudeSign = "";
            }
          }
          break;  // Start bit
		  
        default:
          break;  
      }
    }
  }
}

void ProcessGPGGACommand() {
  int i, j, k, IntegerPart;
  double Divider;

  // $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
  // 8    = Antenna altitude above/below mean sea level (geoid)
  
  Divider = 1;
  
  for (i=7, j=0, k=0; (i<GPSIndex) && (j<8); i++)
  {
    if (GPSBuffer[i] == ',')
    {
      j++;    // Segment index
      k=0;    // Index into target variable
      IntegerPart = 1;
    }
    else
    {
      switch (j)
      {
        case 8:
          if (IntegerPart) // Height
          {
            if ((GPSBuffer[i] >= '0') && (GPSBuffer[i] <= '9'))
            {
              GPS_Direction = GPS_Direction * 10;
              GPS_Direction += (unsigned int)(GPSBuffer[i] - '0');
            }
            else
            {
              IntegerPart = 0;
            }
          }
          break;  // Start bit
          
        default:
          break;  
      }
    }
  }
}

void initialiseGPS(){

	/* PINS configuration */
	// pinMode (int pin, int mode) ; wiringpi pins numbers ==> view wiringPi pins table
	//pinMode (15 , OUTPUT); //TxD UART
	//pinMode (16 , INPUT); //RxD UART
	// ...
	/* ************** Device init ************** */
	/* OPENING SERIAL CNX */
	if((fd = serialOpen("/dev/ttyAMA0", 9600)) < 0 ) /*device must be replaced by the serial port in the RPi*/
		perror("device not opened \n");
	if ( wiringPiSetup () < 0 )
		perror("WiringPiSetup problem \n ");
	/* *********************************************************************** */
	//sleep(5);
	//serialPutchar (fd, '1') ; //Forward
	return;
}

int main (void) {
	//setup SErial Port
	
	//GPGGA gives dddmm.mmmmmm
	//Rpi Camera needs --exif GPS.GPSLongitude=5/1,10/1,15/100 degree minute seconds
	int read = 0;
	initialiseGPS();
	while(serialDataAvail(fd)) {
		serialGetchar(fd);
		read = serialGetchar(fd);
		//getGPS();
	}

}