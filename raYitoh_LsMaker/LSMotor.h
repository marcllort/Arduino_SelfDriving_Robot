
/*************************************************** 
  This is a library for the LSMaker Motor Control
  Designed specifically to work with the LSMaker robot
  ----> http://www.salleurl.edu
  These Motor Control TAD use PWM and Encoders IRQs
  
  Written by Joan Camps/Marcos Hervas for LaSalle-URL.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#ifndef LSMOTOR_H
#define LSMOTOR_H

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

// ========
// ENCODERS
// ========
// 1 volta de motor son 206.5 tics d'encoder
const float CTNT_TICS_VOLTA = 206.25;                    // tics d'encoder en fer una volta completa de la roda
                                                         // l'encoder és de 11PPR a nivell de motor però sembla que tenim una reductora de 18.75:1 (206.25 = 11 x 18.75)
// el perimetre de la roda es de  13.5088cm = (2*pi*r) = (2*pi*2.15cm)
const float CTNT_PERIM_RODA =  13.5088;                 // centimetres del perimetre de la roda
// tics per centimetre es: 15.2863 = 206.5/13.5088
const float CTNT_TICS_CM    =  15.2678;                 // tics de l'encoder per centimetre recorregut de roda

const int LE1 = 2;            // pin d'entrada de l'encoder 1 del motor esquerre (INT0) - pin 3
const int LE2 = 26;           // pin d'entrada de l'encoder 2 del motor esquerre - pin 27
const int RE1 = 3;            // pin d'entrada de l'encoder 1 del motor dret (INT1) - pin 2
const int RE2 = 27;           // pin d'entrada de l'encoder 2 del motor dret - pin 26


class LSMt {
 public:
  LSMt();
  void begin(void);
  void LsMtMotor(void);
  void LsMtMando(float LeftSpeed,float RightSpeed,int push);
  void LsMtFastAutocalibrate() ;
  void LsMtCalibrate();
  void LsMtView();
  
  float ENC_VelocitatLeft() ;
  float ENC_VelocitatRight() ;

  int LsMtHiHaOrdreActiva();
  void LsMtRecte(float initVeloc,float initSpace,float veloc,float space,float endVeloc,float endSpace) ;
  void LsMtDreta(float radi,float initVeloc,float initGraus,float veloc,float graus,float endVeloc,float endGraus);
  void LsMtEsquerra(float radi,float initVeloc,float initGraus,float veloc,float graus,float endVeloc,float endGraus) ;
  void MtParaMotor(char quin);

 private:
  void ENC_Init(void);
  static void ISRencoderLeft(void);
  static void ISRencoderRight(void) ;
  void ENC_Motor(void);

  void ENC_Reset() ;

  long ENC_EspaiLeft() ;
  long ENC_EspaiRight() ;

  float ENC_AcceleracioLeft() ;
  float ENC_AcceleracioRight();
  // interrupcio motor dret
  //ISR(TIMER4_COMPA_vect) ;
  // interrupcio motor esquerre
  //ISR(TIMER4_COMPB_vect);
  // interrupcio overflow
  //ISR(TIMER4_OVF_vect);

  long MtEspaiTics(float espai) ;
  int MtVelocitat(float velo,char roda);
  float MtAcceleracio(float vini,float vfinal,float espai) ;
  float MtVelocitatGir(char gir,float radi,char roda,float velo);
  void MtArrancaMotor(char quin,char com,float velo);
//  void MtParaMotor(char quin);
  float MtAjustaVelocitat(float Speed,float TeoSpace,float RealSpace,float TeoSpeed,float RealSpeed,float TeoAccel,float RealAccel,float MaxSpeed);
//  int LsMtHiHaOrdreActiva();
//  void LsMtRecte(float initVeloc,float initSpace,float veloc,float space,float endVeloc,float endSpace) ;
//  void LsMtDreta(float radi,float initVeloc,float initGraus,float veloc,float graus,float endVeloc,float endGraus);
//  void LsMtEsquerra(float radi,float initVeloc,float initGraus,float veloc,float graus,float endVeloc,float endGraus) ;

};

extern float PercLF, PercRF, PercLB, PercRB;
extern int MaxTicsLF, MaxTicsRF, MaxTicsLB, MaxTicsRB;
extern int MaxSpeedLF, MaxSpeedRF, MaxSpeedLB, MaxSpeedRB;
extern int MinSpeedLF, MinSpeedRF, MinSpeedLB, MinSpeedRB;




#endif
