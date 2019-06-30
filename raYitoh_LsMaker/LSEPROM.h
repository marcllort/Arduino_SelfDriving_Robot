
/*************************************************** 
  This is a library for the LSMaker Motor Control
  Designed specifically to work with the LSMaker robot
  ----> http://www.salleurl.edu
  These Motor Control TAD use PWM and Encoders IRQs
  
  Written by Joan Camps/Marcos Hervas for LaSalle-URL.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#ifndef LSEPROM_H
#define LSEPROM_H

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <EEPROM.h>
#include <SPI.h>

#define SLAVESELECT 53   //ss

#define SPI_A0 49    //aquests 3 fan activar la sortida del decoder corresponent a la EEPROM
#define SPI_A1 48
#define SPI_A2 47

class LSEPROM {
 public:
  LSEPROM();
  void lsGetMAC(byte *macAddress);
//  void lsPutMotorCalibrate(float PLF,float PRF,float PLB,float PRB);
//  void lsGetMotorCalibrated(float *PLF,float *PRF,float *PLB,float *PRB);
  void lsEPgetMotorCalibrated(int *MTLF,int *MTRF,int *MTLB,int *MTRB,int *mSLF,int *mSRF,int *mSLB,int *mSRB,
                            int *MSLF,int *MSRF,int *MSLB,int *MSRB);
  void lsEPputMotorCalibrate(int MTLF,int MTRF,int MTLB,int MTRB,int mSLF,int mSRF,int mSLB,int mSRB,
                           int MSLF,int MSRF,int MSLB,int MSRB);
  bool lsEPcalibrated();  
  void lsEPputChar(byte ad,char val);
  void lsEPgetChar(byte ad,char *val);
  void lsEPputInt(byte ad,int val);
  void lsEPgetInt(byte ad,int *val);
  void lsEPputLong(byte ad,long val);
  void lsEPgetLong(byte ad,long *val);
  void lsEPputFloat(byte ad,float val);
  void lsEPgetFloat(byte ad,float *val);
 private:
 void fill_buffer(void);
 byte readSPI(byte A);
 void writeSPI(byte pos,byte val);
};

//=========================================================================================  EEPROM ARDUINO MEGA

void EEPgetMotorCalibrated(int *MTLF,int *MTRF,int *MTLB,int *MTRB,int *mSLF,int *mSRF,int *mSLB,int *mSRB,
                           int *MSLF,int *MSRF,int *MSLB,int *MSRB);
void EEPputMotorCalibrate(int MTLF,int MTRF,int MTLB,int MTRB,int mSLF,int mSRF,int mSLB,int mSRB,
                          int MSLF,int MSRF,int MSLB,int MSRB);
bool EEPcalibrated();  

#endif

