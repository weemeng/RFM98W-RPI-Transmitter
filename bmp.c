#include <stdio.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <wiringSerial.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#define BMP180_ADDR 0x77 // 7-bit address
#define	BMP180_REG_CONTROL 0xF4
#define	BMP180_REG_RESULT 0xF6
#define	BMP180_COMMAND_TEMPERATURE 0x2E
#define	BMP180_COMMAND_PRESSURE0 0x34
#define	BMP180_COMMAND_PRESSURE1 0x74
#define	BMP180_COMMAND_PRESSURE2 0xB4
#define	BMP180_COMMAND_PRESSURE3 0xF4

// int wiringPiI2CReadReg8 (int fd, int reg) ;
// int wiringPiI2CReadReg16 (int fd, int reg) ;
// int wiringPiI2CWriteReg8 (int fd, int reg, int data) ;
// int wiringPiI2CWriteReg16 (int fd, int reg, int data) ;

double temp, pressure, sealevel, altitude;


int main void() {
	int openi2c, datain, checktmp, checkpressuredelay, sample;
	int16_t AC1,AC2,AC3,VB1,VB2,MB,MC,MD;
	uint16_t AC4,AC5,AC6; 
	double c5,c6,mc,md,x0,x1,x2,y0,y1,y2,p0,p1,p2;

	openi2c = wiringPiI2CSetup(0x77);
	BMP180begin();
	checktmp = startTemperature();
	if (check == 1) {
		getTemperature(temp);
		printf("temp is %lf", temp);
	}
	for (sample = 0; sample<4; sample++) {
		checkpressuredelay = startPressure(sample);
		delay(checkpressuredelay);
		getPressure(pressure, temp);
		sealevel = getsealevel(pressure, p0);
	return 0;
}

void BMP180begin() {	
	AC1 = wiringPiI2CReadReg16(openi2c, 0xAA);
	AC2 = wiringPiI2CReadReg16(openi2c, 0xAC);
	AC3 = wiringPiI2CReadReg16(openi2c, 0xAE);
	AC4 = wiringPiI2CReadReg16(openi2c, 0xB0); //unsigned
	AC5 = wiringPiI2CReadReg16(openi2c, 0xB2); //unsigned
	AC6 = wiringPiI2CReadReg16(openi2c, 0xB4); //unsigned
	VB1 = wiringPiI2CReadReg16(openi2c, 0xB6);
	VB2 = wiringPiI2CReadReg16(openi2c, 0xB8);
	MB = wiringPiI2CReadReg16(openi2c, 0xBA);
	MC = wiringPiI2CReadReg16(openi2c, 0xBC);
	MD = wiringPiI2CReadReg16(openi2c, 0xBE);
	
	Serial.print("AC1: "); Serial.println(AC1);
	Serial.print("AC2: "); Serial.println(AC2);
	Serial.print("AC3: "); Serial.println(AC3);
	Serial.print("AC4: "); Serial.println(AC4);
	Serial.print("AC5: "); Serial.println(AC5);
	Serial.print("AC6: "); Serial.println(AC6);
	Serial.print("VB1: "); Serial.println(VB1);
	Serial.print("VB2: "); Serial.println(VB2);
	Serial.print("MB: "); Serial.println(MB);
	Serial.print("MC: "); Serial.println(MC);
	Serial.print("MD: "); Serial.println(MD);
		
		
	// Compute floating-point polynominals:

	c3 = 160.0 * pow(2,-15) * AC3;
	c4 = pow(10,-3) * pow(2,-15) * AC4;
	b1 = pow(160,2) * pow(2,-30) * VB1;
	c5 = (pow(2,-15) / 160) * AC5;
	c6 = AC6;
	mc = (pow(2,11) / pow(160,2)) * MC;
	md = MD / 160.0;
	x0 = AC1;
	x1 = 160.0 * pow(2,-13) * AC2;
	x2 = pow(160,2) * pow(2,-25) * VB2;
	y0 = c4 * pow(2,15);
	y1 = c4 * c3;
	y2 = c4 * b1;
	p0 = (3791.0 - 8.0) / 1600.0;
	p1 = 1.0 - 7357.0 * pow(2,-20);
	p2 = 3038.0 * 100.0 * pow(2,-36);

	Serial.println();
	Serial.print("c3: "); Serial.println(c3);
	Serial.print("c4: "); Serial.println(c4);
	Serial.print("c5: "); Serial.println(c5);
	Serial.print("c6: "); Serial.println(c6);
	Serial.print("b1: "); Serial.println(b1);
	Serial.print("mc: "); Serial.println(mc);
	Serial.print("md: "); Serial.println(md);
	Serial.print("x0: "); Serial.println(x0);
	Serial.print("x1: "); Serial.println(x1);
	Serial.print("x2: "); Serial.println(x2);
	Serial.print("y0: "); Serial.println(y0);
	Serial.print("y1: "); Serial.println(y1);
	Serial.print("y2: "); Serial.println(y2);
	Serial.print("p0: "); Serial.println(p0);
	Serial.print("p1: "); Serial.println(p1);
	Serial.print("p2: "); Serial.println(p2);
	return;
}

int startTemperature() {
	unsigned char result;
	//data[0] = BMP180_REG_CONTROL;
	//data[1] = BMP180_COMMAND_TEMPERATURE;
	
	result = wiringPiI2CWriteReg8(openi2c, BMP180_REG_CONTROL, BMP180_COMMAND_TEMPERATURE);
	if (result) {
		return 1; 
	else
		return 0; //got error
	}
}

void getTemperature (double &T) {
	unsigned char data[2];
	char result;
	double tu, a;
//	data[0] = BMP180_REG_RESULT;
	
	data[0] = wiringPiI2CReadReg8(openi2c, BMP180_REG_RESULT);
	data[1] = wiringPiI2CReadReg8(openi2c, BMP180_REG_RESULT);
	
	//assuming good calc
	tu = (data[0] * 256.0) + data[1];
	
	a = c5 * (tu - c6);
	T = a + (mc / (a + md));
	return;
}

int startPressure(char oversampling) {
// Oversampling: 0 to 3, higher numbers are slower, higher-res outputs.
	unsigned char data[2], result, delay;
	
	switch (oversampling)
	{
		case 0:
			result = wiringPiI2CWriteReg8(openi2c, BMP180_REG_CONTROL, BMP180_COMMAND_PRESSURE0);
			delay = 5;
		break;
		case 1:
			result = wiringPiI2CWriteReg8(openi2c, BMP180_REG_CONTROL, BMP180_COMMAND_PRESSURE1);
			delay = 8;
		break;
		case 2:
			result = wiringPiI2CWriteReg8(openi2c, BMP180_REG_CONTROL, BMP180_COMMAND_PRESSURE2);
			delay = 14;
		break;
		case 3:
			result = wiringPiI2CWriteReg8(openi2c, BMP180_REG_CONTROL, BMP180_COMMAND_PRESSURE3);
			delay = 26;
		break;
		default:
			result = wiringPiI2CWriteReg8(openi2c, BMP180_REG_CONTROL, BMP180_COMMAND_PRESSURE0);
			delay = 5;
		break;
	}
	if (result) // good write?
		return(delay); // return the delay in ms (rounded up) to wait before retrieving data
	else
		return(0); // or return 0 if there was a problem communicating with the BMP
}


void getPressure(double &P, double &T) {
	unsigned char data[3];
	char result;
	double pu,s,x,y,z;
	
	data[0] = wiringPiI2CReadReg8(openi2c, BMP180_REG_RESULT);
	data[1] = wiringPiI2CReadReg8(openi2c, BMP180_REG_RESULT);
	data[2] = wiringPiI2CReadReg8(openi2c, BMP180_REG_RESULT);
	
	//assuming good read
	pu = (data[0] * 256.0) + data[1] + (data[2]/256.0);
	s = T - 25.0;
	x = (x2 * pow(s,2)) + (x1 * s) + x0;
	y = (y2 * pow(s,2)) + (y1 * s) + y0;
	z = (pu - x) / y;
	P = (p2 * pow(z,2)) + (p1 * z) + p0;
	return;
}

double getsealevel(double P, double A) {
	//in mb
	return(P/pow(1-(A/44330.0),5.255));
}

double getaltitude(double P, double P0) {
	//in mb
	return(44330.0*(1-pow(P/P0,1/5.255)));
}

int getError() {
	return(_error);
}

