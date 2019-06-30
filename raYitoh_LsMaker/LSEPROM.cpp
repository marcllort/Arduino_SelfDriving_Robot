
/*************************************************** 
  This is a library for the LSMaker Motor Control
  Designed specifically to work with the LSMaker robot
  ----> http://www.salleurl.edu
  These Motor Control TAD use PWM and Encoders IRQs
  
  Written by Joan Camps/Marcos Hervas for LaSalle-URL.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "LSEPROM.h"

//variables
//byte macAddress[6];


byte variableBYTE;   

byte eeprom_output_data;
byte eeprom_input_data=0;
byte clr;
int address=0;
//data buffer
char buffer [128];

byte markadd = 0xA0;
byte markval = 0xaa;

LSEPROM::LSEPROM(void) {
  pinMode(SPI_A0,OUTPUT);    
  pinMode(SPI_A1,OUTPUT);
  pinMode(SPI_A2,OUTPUT);
  digitalWrite(SPI_A0,LOW);    
  digitalWrite(SPI_A1,LOW);
  digitalWrite(SPI_A2,LOW);
  SPI.begin();
}

void LSEPROM::fill_buffer(){
  for (int I=0;I<128;I++){
    buffer[I]=I;
  }
}

//void LSEPROM::EEPROM_setup() {
    // start the SPI library:
//  SPI.begin();
//}

void LSEPROM::lsGetMAC(byte *macAddress){
  //configurar CS (els tres valors A0-A2) que activen la sortida del decoder corresponent a la EEPROM
  digitalWrite(SLAVESELECT, HIGH);
  pinMode(SPI_A0,OUTPUT);    
  pinMode(SPI_A1,OUTPUT);
  pinMode(SPI_A2,OUTPUT);
  digitalWrite(SPI_A0,HIGH);    
  digitalWrite(SPI_A1,HIGH);
  digitalWrite(SPI_A2, LOW);
  SPI.begin();
  macAddress[0]=readSPI(0xFA);
  macAddress[1]=readSPI(0xFB);
  macAddress[2]=readSPI(0xFC);
  macAddress[3]=readSPI(0xFD);
  macAddress[4]=readSPI(0xFE);
  macAddress[5]=readSPI(0xFF);
//  SPI.end();
  for(int i = 5; i>0; i--){
    Serial.print(macAddress[i],HEX);  //mostra per pantalla les dues parts de la MAC
    Serial.print(":");  //mostra per pantalla les dues parts de la MAC
  }
  Serial.println(macAddress[0],HEX);
}

/*
void LSEPROM::lsPutMotorCalibrate(float PLF,float PRF,float PLB,float PRB) {
  //configurar CS (els tres valors A0-A2) que activen la sortida del decoder corresponent a la EEPROM
  digitalWrite(SPI_A0,HIGH);    
  digitalWrite(SPI_A1,HIGH);
  digitalWrite(SPI_A2, LOW);
  writeSPI(0xAF,0xAA);
  lsEPputFloat(0xB0,PLF);
  lsEPputFloat(0xB4,PRF);
  lsEPputFloat(0xB8,PLB);
  lsEPputFloat(0xBC,PRB);
}

void LSEPROM::lsGetMotorCalibrated(float *PLF,float *PRF,float *PLB,float *PRB) {
  //configurar CS (els tres valors A0-A2) que activen la sortida del decoder corresponent a la EEPROM
  digitalWrite(SPI_A0,HIGH);    
  digitalWrite(SPI_A1,HIGH);
  digitalWrite(SPI_A2, LOW);
  if (readSPI(0xAF)==0xAA) {
    lsEPgetFloat(0xB0,PLF);
    lsEPgetFloat(0xB4,PRF);
    lsEPgetFloat(0xB8,PLB);
    lsEPgetFloat(0xBC,PRB);
  } else {
    *PLF=1.0;
    *PRF=1.0;
    *PLB=1.0;
    *PRB=1.0;
  }
}
*/

void LSEPROM::lsEPgetMotorCalibrated(int *MTLF,int *MTRF,int *MTLB,int *MTRB,int *mSLF,int *mSRF,int *mSLB,int *mSRB,
                            int *MSLF,int *MSRF,int *MSLB,int *MSRB) {
  //configurar CS (els tres valors A0-A2) que activen la sortida del decoder corresponent a la EEPROM
  digitalWrite(SLAVESELECT, HIGH);
  digitalWrite(SPI_A0,HIGH);    
  digitalWrite(SPI_A1,HIGH);
  digitalWrite(SPI_A2, LOW);
  SPI.begin();
  if (readSPI(markadd)==markval) {
    lsEPgetInt(0xA8,MTLF);
    lsEPgetInt(0xAA,MTRF);
    lsEPgetInt(0xAC,MTLB);
    lsEPgetInt(0xAE,MTRB);
    lsEPgetInt(0xB0,mSLF);
    lsEPgetInt(0xB2,mSRF);
    lsEPgetInt(0xB4,mSLB);
    lsEPgetInt(0xB6,mSRB);
    lsEPgetInt(0xB8,MSLF);
    lsEPgetInt(0xBA,MSRF);
    lsEPgetInt(0xBC,MSLB);
    lsEPgetInt(0xBE,MSRB);
  } else {
    *MTLF=850;
    *MTRF=850;
    *MTLB=850;
    *MTRB=850;
    *mSLF=0;
    *mSRF=0;
    *mSLB=0;
    *mSRB=0;
    *MSLF=1023;
    *MSRF=1023;
    *MSLB=1023;
    *MSRB=1023;
  }
}

void LSEPROM::lsEPputMotorCalibrate(int MTLF,int MTRF,int MTLB,int MTRB,int mSLF,int mSRF,int mSLB,int mSRB,
                           int MSLF,int MSRF,int MSLB,int MSRB) {
  //configurar CS (els tres valors A0-A2) que activen la sortida del decoder corresponent a la EEPROM
  digitalWrite(SLAVESELECT, HIGH);
  digitalWrite(SPI_A0,HIGH);    
  digitalWrite(SPI_A1,HIGH);
  digitalWrite(SPI_A2, LOW);
  SPI.begin();
  writeSPI(markadd,markval);
  lsEPputInt(0xA8,MTLF);
  lsEPputInt(0xAA,MTRF);
  lsEPputInt(0xAC,MTLB);
  lsEPputInt(0xAE,MTRB);
  lsEPputInt(0xB0,mSLF);
  lsEPputInt(0xB2,mSRF);
  lsEPputInt(0xB4,mSLB);
  lsEPputInt(0xB6,mSRB);
  lsEPputInt(0xB8,MSLF);
  lsEPputInt(0xBA,MSRF);
  lsEPputInt(0xBC,MSLB);
  lsEPputInt(0xBE,MSRB);
}

bool LSEPROM::lsEPcalibrated() { 
  byte car;
  digitalWrite(SLAVESELECT, HIGH);
  digitalWrite(SPI_A0,HIGH);    
  digitalWrite(SPI_A1,HIGH);
  digitalWrite(SPI_A2, LOW);
  SPI.begin();
  car = readSPI(markadd);
  Serial.println(car,HEX);
  if (car==markval) return true;
  else return false;
}


void LSEPROM::lsEPputChar(byte ad,char val) {
  digitalWrite(SLAVESELECT, HIGH);
  digitalWrite(SPI_A0,HIGH);    
  digitalWrite(SPI_A1,HIGH);
  digitalWrite(SPI_A2, LOW);
  writeSPI(ad,val);
}

void LSEPROM::lsEPgetChar(byte ad,char *val) {
  digitalWrite(SLAVESELECT, HIGH);
  digitalWrite(SPI_A0,HIGH);    
  digitalWrite(SPI_A1,HIGH);
  digitalWrite(SPI_A2, LOW);
  *val = readSPI(ad);  
}

void LSEPROM::lsEPputInt(byte ad,int val) {
  digitalWrite(SLAVESELECT, HIGH);
  digitalWrite(SPI_A0,HIGH);    
  digitalWrite(SPI_A1,HIGH);
  digitalWrite(SPI_A2, LOW);
  writeSPI(ad,(val>>8)&0XFF);
  writeSPI(ad+1,val&0XFF);
}

void LSEPROM::lsEPgetInt(byte ad,int *val) {
  digitalWrite(SLAVESELECT, HIGH);
  digitalWrite(SPI_A0,HIGH);    
  digitalWrite(SPI_A1,HIGH);
  digitalWrite(SPI_A2, LOW);
  *val = (((int)readSPI(ad))<<8)&0xFF00 | ((int)readSPI(ad+1))&0x00FF;  
}

void LSEPROM::lsEPputLong(byte ad,long val) {
  digitalWrite(SLAVESELECT, HIGH);
  digitalWrite(SPI_A0,HIGH);    
  digitalWrite(SPI_A1,HIGH);
  digitalWrite(SPI_A2, LOW);
  writeSPI(ad,((char)(val>>24))&0XFF);
  writeSPI(ad+1,((char)(val>>16))&0XFF);
  writeSPI(ad+2,((char)(val>>8))&0XFF);
  writeSPI(ad+3,((char)val)&0XFF);
}

void LSEPROM::lsEPgetLong(byte ad,long *val) {
  digitalWrite(SLAVESELECT, HIGH);
  digitalWrite(SPI_A0,HIGH);    
  digitalWrite(SPI_A1,HIGH);
  digitalWrite(SPI_A2, LOW);
  *val = (((long)readSPI(ad))<<24)&0xFF000000 | (((long)readSPI(ad+1))<<16)&0x00FF0000 | (((long)readSPI(ad+2))<<8)&0x0000FF00 | ((long)readSPI(ad+3))&0x000000FF;
}

void LSEPROM::lsEPputFloat(byte ad,float val) {
  long vale=val*100000000;
  digitalWrite(SLAVESELECT, HIGH);
  digitalWrite(SPI_A0,HIGH);    
  digitalWrite(SPI_A1,HIGH);
  digitalWrite(SPI_A2, LOW);
  writeSPI(ad,((char)(vale>>24))&0XFF);
  writeSPI(ad+1,((char)(vale>>16))&0XFF);
  writeSPI(ad+2,((char)(vale>>8))&0XFF);
  writeSPI(ad+3,((char)vale)&0XFF);
}

void LSEPROM::lsEPgetFloat(byte ad,float *val) {
  long vale;
  digitalWrite(SLAVESELECT, HIGH);
  digitalWrite(SPI_A0,HIGH);    
  digitalWrite(SPI_A1,HIGH);
  digitalWrite(SPI_A2, LOW);
  vale = (((long)readSPI(ad))<<24)&0xFF000000 | (((long)readSPI(ad+1))<<16)&0x00FF0000 | (((long)readSPI(ad+2))<<8)&0x0000FF00 | ((long)readSPI(ad+3))&0x000000FF;
  *val = vale/100000000.0;
}

byte LSEPROM::readSPI(byte A){
   byte dada;
   delay(20);
   digitalWrite(SLAVESELECT,LOW);
   SPI.transfer(0x03);
   SPI.transfer(A);
   dada = SPI.transfer(0x00);
   digitalWrite(SLAVESELECT, HIGH);
   delay(20);
   return dada;
}

void LSEPROM::writeSPI(byte pos,byte val){
  byte dada;
  digitalWrite(SLAVESELECT,LOW);
  SPI.transfer(0x06);
  digitalWrite(SLAVESELECT,HIGH);
  delay(10);
  digitalWrite(SLAVESELECT,LOW);
  SPI.transfer(0x02);
  SPI.transfer(pos);
  SPI.transfer(val);
  delay(10);
  digitalWrite(SLAVESELECT, HIGH);
  delay(20);
}


//=========================================================================================  EEPROM ARDUINO MEGA


void EEPgetMotorCalibrated(int *MTLF,int *MTRF,int *MTLB,int *MTRB,int *mSLF,int *mSRF,int *mSLB,int *mSRB,
                           int *MSLF,int *MSRF,int *MSLB,int *MSRB) {
  if (EEPROM.read(0x01FF)==0xAA) {
    // MaxTics
    (*MTLF) = (int)((((int)EEPROM.read(0x0200)<<8)&0xFF00) | ((int)EEPROM.read(0x0201)&0x00FF));
    (*MTRF) = (int)((((int)EEPROM.read(0x0202)<<8)&0xFF00) | ((int)EEPROM.read(0x0203)&0x00FF));
    (*MTLB) = (int)((((int)EEPROM.read(0x0204)<<8)&0xFF00) | ((int)EEPROM.read(0x0205)&0x00FF));
    (*MTRB) = (int)((((int)EEPROM.read(0x0206)<<8)&0xFF00) | ((int)EEPROM.read(0x0207)&0x00FF));
    // MinSpeed
    (*mSLF) = (int)((((int)EEPROM.read(0x0208)<<8)&0xFF00) | ((int)EEPROM.read(0x0209)&0x00FF));
    (*mSRF) = (int)((((int)EEPROM.read(0x020A)<<8)&0xFF00) | ((int)EEPROM.read(0x020B)&0x00FF));
    (*mSLB) = (int)((((int)EEPROM.read(0x020C)<<8)&0xFF00) | ((int)EEPROM.read(0x020D)&0x00FF));
    (*mSRB) = (int)((((int)EEPROM.read(0x020E)<<8)&0xFF00) | ((int)EEPROM.read(0x020F)&0x00FF));
    // MaxSpeed
    (*MSLF) = (int)((((int)EEPROM.read(0x0210)<<8)&0xFF00) | ((int)EEPROM.read(0x0211)&0x00FF));
    (*MSRF) = (int)((((int)EEPROM.read(0x0212)<<8)&0xFF00) | ((int)EEPROM.read(0x0213)&0x00FF));
    (*MSLB) = (int)((((int)EEPROM.read(0x0214)<<8)&0xFF00) | ((int)EEPROM.read(0x0215)&0x00FF));
    (*MSRB) = (int)((((int)EEPROM.read(0x0216)<<8)&0xFF00) | ((int)EEPROM.read(0x0217)&0x00FF));
  } else {
    (*MTLF)=850;
    (*MTRF)=850;
    (*MTLB)=850;
    (*MTRB)=850;
    (*mSLF)=0;
    (*mSRF)=0;
    (*mSLB)=0;
    (*mSRB)=0;
    (*MSLF)=1023;
    (*MSRF)=1023;
    (*MSLB)=1023;
    (*MSRB)=1023;
  }
}

void EEPputMotorCalibrate(int MTLF,int MTRF,int MTLB,int MTRB,int mSLF,int mSRF,int mSLB,int mSRB,
                          int MSLF,int MSRF,int MSLB,int MSRB) {  EEPROM.write(0x01FF,0xAA);
  // MaxTics
  EEPROM.write(0x0200,(MTLF>>8)&0xFF);
  EEPROM.write(0x0201,MTLF&0xFF);
  EEPROM.write(0x0202,(MTRF>>8)&0xFF);
  EEPROM.write(0x0203,MTRF&0xFF);
  EEPROM.write(0x0204,(MTLB>>8)&0xFF);
  EEPROM.write(0x0205,MTLB&0xFF);
  EEPROM.write(0x0206,(MTRB>>8)&0xFF);
  EEPROM.write(0x0207,MTRB&0xFF);
  // MinSpeed
  EEPROM.write(0x0208,(mSLF>>8)&0xFF);
  EEPROM.write(0x0209,mSLF&0xFF);
  EEPROM.write(0x020A,(mSRF>>8)&0xFF);
  EEPROM.write(0x020B,mSRF&0xFF);
  EEPROM.write(0x020C,(mSLB>>8)&0xFF);
  EEPROM.write(0x020D,mSLB&0xFF);
  EEPROM.write(0x020E,(mSRB>>8)&0xFF);
  EEPROM.write(0x020F,mSRB&0xFF);
  // MaxSpeed
  EEPROM.write(0x0210,(MSLF>>8)&0xFF);
  EEPROM.write(0x0211,MSLF&0xFF);
  EEPROM.write(0x0212,(MSRF>>8)&0xFF);
  EEPROM.write(0x0213,MSRF&0xFF);
  EEPROM.write(0x0214,(MSLB>>8)&0xFF);
  EEPROM.write(0x0215,MSLB&0xFF);
  EEPROM.write(0x0216,(MSRB>>8)&0xFF);
  EEPROM.write(0x0217,MSRB&0xFF);
}

bool EEPcalibrated() {
  byte car;
  car = EEPROM.read(0x01FF);
  if (car==0xAA) return true;
  else return false;
}


