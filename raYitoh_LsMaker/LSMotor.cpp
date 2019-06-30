
/*************************************************** 
  This is a library for the LSMaker Motor Control
  Designed specifically to work with the LSMaker robot
  ----> http://www.salleurl.edu
  These Motor Control TAD use PWM and Encoders IRQs
  
  Written by Joan Camps/Marcos Hervas for LaSalle-URL.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/


#include "LSMotor.h"

long ticsL=0,ticsR=0;
long auxL=0,auxR=0;
unsigned long t1,t2,inct;
float old_spL,old_spR,spL,spR;
float accL,accR;


LSMt::LSMt(void) {
}


void LSMt::ENC_Init(void) {
  pinMode(RE1,INPUT_PULLUP);
  pinMode(RE2,INPUT_PULLUP);
  pinMode(LE1,INPUT_PULLUP);
  pinMode(LE2, INPUT_PULLUP);
  auxL = 0;
  auxR = 0;
  ticsL = 0;
  ticsR = 0;
  old_spL = 0;
  spL = 0;
  old_spR = 0;
  spR = 0;
  accL = 0;
  accR = 0;
  attachInterrupt(digitalPinToInterrupt(LE1),ISRencoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(RE1),ISRencoderRight, RISING);
  t1 = millis();
}

void LSMt::ISRencoderLeft(void) {
  if (digitalRead(LE2)==HIGH) ticsL++;
  else ticsL--;
}

void LSMt::ISRencoderRight() {
  if (digitalRead(RE2)==HIGH) ticsR++;
  else ticsR--;
}

void LSMt::ENC_Motor() {
  t2 = millis();
  inct = t2-t1;
  if (inct>=10) {
    old_spL = spL;
    old_spR = spR;
    spL = (ticsL-auxL)*1000/(float)inct;
    spR = (ticsR-auxR)*1000/(float)inct;
    accL = (spL-old_spL)*1000/(float)inct;
    accR = (spR-old_spR)*1000/(float)inct;
    auxL = ticsL;
    auxR = ticsR;
    t1 = t2;
  }
}

void LSMt::ENC_Reset() {
  auxL = 0;
  auxR = 0;
  ticsL=0;
  ticsR=0;
  old_spL = 0;
  spL = 0;
  old_spR = 0;
  spR = 0;
  accL = 0;
  accR = 0;
  t1 = millis();
}

long LSMt::ENC_EspaiLeft() {   // tics
  return ticsL;
}

long LSMt::ENC_EspaiRight() {   // tics
  return ticsR;
}

float LSMt::ENC_VelocitatLeft() {   // tics/segon
  return spL;
}

float LSMt::ENC_VelocitatRight() {  // tics/segon
  return spR;
}

float LSMt::ENC_AcceleracioLeft() {   // tics/segon^2
  return accL;
}

float LSMt::ENC_AcceleracioRight() {  // tics/segon^2
  return accR;
}

void LSMt::LsMtView() {
  Serial.print(ticsL); Serial.print(" "); Serial.print(ticsR);
  Serial.print(" <S> ");
  Serial.print(spL); Serial.print(" "); Serial.println(spR);
}


// ======
// MOTORS
// ======

// Modificat en la release 2

// Right Wheel
const int LWenable = 22;
const int LWfront = 23;
const int LWpwm = 7;
// Left Wheel
const int RWenable = 24;
const int RWfront = 25;
const int RWpwm = 8;

// amplada del robot de centre de la roda a centre de l'altra roda
const float CT_AMPLE = 20.0; 

typedef struct {
  int   actiu;                             // 0-sense ordre  1-ordre donada  2-ordre en curs (accelerant)  3-ordre en curs(vel.constant)  4-ordre en curs (frenant)
  char  direc;                             // 'L' left, 'R' right, 'F' recte
  //   dades d'entrada
  int   accel;                             // 0-sense accel. 1-amb accel.
  float initvelo, initspace;               // dades per l'acceleracio
  float veloc, space, radi;                // dades per la velocitat constant
  int   frena;                             // 0-sense fren.  1-amb fren.
  float endveloc,endspace;                 // dades per la frenada
  // dades calculades per a cada roda
                                           // M.U.A: MOVIMENT UNIFORMEMENT ACCELERAT
  float initLveloc,initRveloc;             // M.U.A: velocitat inicial calculada a cada roda
  int   initLspeed,initRspeed;             // M.U.A: velocitat inicial segons els motors de cada roda
//  int   initLdir,initRdir;                 // M.U.A: direccio inicial segons els motors de cada roda
  float accL,accR;                         // M.U.A: calcul de l'acceleracio de cada motor
  long  initLtics,initRtics;               // M.U.A: tics de encoder de cada roda en l'acceleracio
                                           // M.U: MOVIMENT UNIFORME
  float Lveloc,Rveloc;                     // M.U: velocitat constant calculada per a cada roda
  int   Lspeed,Rspeed;                     // M.U: velocitats constants sefons el motor de cada roda
  int   Ldir,Rdir;                         // M.U: direccio segons els motors de cada roda
  float Ltics,Rtics;                       // M.U: tics de encoder de cada roda en la velocitat constant    // era long
                                           // M.U.F: MOVIMENT UNIFORMEMENT FRENAT
  float endLveloc,endRveloc;               // M.U.F: velocitat final calculada a cada roda
  int   endLspeed,endRspeed;               // M.U.F: velocitat final segons els motors de cada roda
//  int   endLdir,endRdir;                   // M.U.F: direccio final segons els motors de cada roda
  float freL,freR;                         // M.U.F: calcul de la frenada de cada motor
  long  endLtics,endRtics;                 // M.U.F: tics de encoder de cada roda en la frenada
  
// he passat tots els long a float!!!
  // acumulats teorics...
  int   teoTtics;                          // tics actuals temporals acumulats
  int   teoTticsInc;                       // tics actuals temporals en l'increment
  float teoLtics;                          // tics actuals encoder Left acumulats (espai)
  float teoLticsInc;                       // tics actuals encoder Left en l'increment (velocitat)
  float teoLticsIncInc;                    // variacio dels tics actuals encoder Left en l'increment (acceleracio)
  float teoRtics;                          // tics actuals encoder Right acumulats (espai)
  float teoRticsInc;                       // tics actuals encoder Right en l'increment (velocitat)
  float teoRticsIncInc;                    // variacio dels tics actuals encoder Right en l'increment (acceleracio)
  // acumulats reals...
  int   actTtics;                          // tics actuals temporals acumulats
  int   actTticsInc;                       // tics actuals temporals en l'increment
  float actLtics;                          // tics actuals encoder Left acumulats (espai)
  float actLticsInc;                       // tics actuals encoder Left en l'increment (velocitat)
  float actLticsIncInc;                    // variacio dels tics actuals encoder Left en l'increment (acceleracio)
  float actRtics;                          // tics actuals encoder Right acumulats (espai)
  float actRticsInc;                       // tics actuals encoder Right en l'increment (velocitat)
  float actRticsIncInc;                    // variacio dels tics actuals encoder Right en l'increment (acceleracio)
} MotorsStatus;

MotorsStatus MT;

int Lspeed, Rspeed;

int MaxTicsLF, MaxTicsRF, MaxTicsLB, MaxTicsRB;
int MaxSpeedLF, MaxSpeedRF, MaxSpeedLB, MaxSpeedRB;
int MinSpeedLF, MinSpeedRF, MinSpeedLB, MinSpeedRB;
float fact[33] = {0, 0.08538163, 0.112548512, 0.131953428, 0.138421734, 0.144890039, 0.151358344, 0.155239327, 0.160413972,
                  0.169469599, 0.175937904, 0.187580854, 0.19534282, 0.205692109, 0.214747736, 0.228978008, 0.243208279,
                  0.257438551, 0.26778784, 0.283311772, 0.301423027, 0.326002587, 0.344113842, 0.372574386, 0.401034929,
                  0.434670116, 0.472186287, 0.518758085, 0.557567917, 0.602846054, 0.690815006, 0.767141009, 1};
float MaxTicsF,MaxTicsB;
float PercLF, PercRF, PercLB, PercRB;
long ticsMando;

// velocitat maxima lineal (cm/s)
//const float CT_VMAX = 890/CTNT_TICS_CM;
//#define CT_VMAX (MaxSpeedTicsLF[32]/(float)CTNT_TICS_CM)

// velocitat maxima lineal (cm/s)
//const float CT_VMIN = -830/CTNT_TICS_CM;
//#define CT_VMIN (SpeedTicsLB[32]/(float)CTNT_TICS_CM)

#define FRONT LOW
#define BACK HIGH

// RUTINES DE SERVEI D'INTERRUPCIO
// -------------------------------

// interrupcio motor dret
ISR(TIMER4_COMPA_vect) {
  OCR4A=Rspeed;
  digitalWrite(RWpwm,LOW);
}
// interrupcio motor esquerre
ISR(TIMER4_COMPB_vect) {
  OCR4B=Lspeed;
  digitalWrite(LWpwm,LOW);
}
// interrupcio overflow
ISR(TIMER4_OVF_vect) {
  digitalWrite(LWpwm,HIGH);
  digitalWrite(RWpwm,HIGH);
}

// FUNCIONS AUXILIARS
// ------------------

// passa de cm a tics d'encoder corresponents
long LSMt::MtEspaiTics(float espai) {
  long tics;
  tics = espai * CTNT_TICS_CM;
  return tics;
}

// passa de velocitat (ticsenc/s) a expressio necessaria pel motor (0x0???)
int LSMt::MtVelocitat(float velo,char roda) {
  float tic1, tic2;
  int i, mot1, mot2, vel=0x0000;
  if (roda=='L') {
    if (velo>=0) {
      velo = velo * PercLF;
      i = velo*32/(float)MaxTicsF;
      tic1=i*MaxTicsF/32;
      tic2=(i+1)*MaxTicsF/32;
      mot1=1023*fact[i];
      mot2=1023*fact[i+1];
    } else {
      velo = -velo * PercLB;
      i = velo*32/(float)MaxTicsB;
      tic1=i*MaxTicsB/32;
      tic2=(i+1)*MaxTicsB/32;
      mot1=1023*fact[i];
      mot2=1023*fact[i+1];
    }
  }
  if (roda=='R') {
    if (velo>=0) {
      velo = velo * PercRF;
      i = velo*32/(float)MaxTicsF;
      tic1=i*MaxTicsF/32;
      tic2=(i+1)*MaxTicsF/32;
      mot1=1023*fact[i];
      mot2=1023*fact[i+1];
    } else {
      velo = -velo * PercRB;
      i = velo*32/(float)MaxTicsB;
      tic1=i*MaxTicsRB/32;
      tic2=(i+1)*MaxTicsRB/32;
      mot1=1023*fact[i];
      mot2=1023*fact[i+1];
    }
  }
  vel = map(velo,tic1,tic2,mot1,mot2);
  return vel;
}

// calcula l'acceleracio a partir de velocitat inicial, final i espai a recorrer; unitats: ticsenc/s^2
float LSMt::MtAcceleracio(float vini,float vfinal,float espai) {
  float acc;
//  acc = ((vfinal-vini)*(vfinal+vini)/2)/espai;
  acc = (vfinal*vfinal-vini*vini)/(2*espai);
  return acc;
}

// calcula la velocitat de cada roda en qualsevol gir
float LSMt::MtVelocitatGir(char gir,float radi,char roda,float velo) {
  float vg=0;
  if (gir=='L' || gir=='R') {
    if (fabs(radi-(CT_AMPLE/2.0))<0.1) {    // el centre de gir es una de les rodes
        if (gir=='L') {                         // girem a l'esquerra
          if (roda=='L') vg=0;                      // roda esquerra quieta
          else vg=2*velo;                           // roda dreta belluga
        } else {                                // girem a la dreta
          if (roda=='R') vg=0;                      // roda dreta quieta
          else vg=2*velo;                           // roda esquerra belluga
        }
    } else {
      if ((radi-(CT_AMPLE/2.0))>0) {        // el centre de gir esta situat fora del robot
        if (gir=='L') {                         // girem a l'esquerra
          if (roda=='L') vg=velo*radi/(radi-(CT_AMPLE/2));
          else vg=velo*radi/(radi+(CT_AMPLE/2));
        } else {                                // girem a la dreta
          if (roda=='R') vg=velo*radi/(radi-(CT_AMPLE/2));
          else vg=velo*radi/(radi+(CT_AMPLE/2));
        }
      } else {                              // el centre de gir est� situat entre les rodes del robot
        if (gir=='L') {                         // girem a l'esquerra
          if (roda=='L') vg=-velo*(CT_AMPLE/2-radi)/(CT_AMPLE/2);
          else vg=velo*(CT_AMPLE/2+radi)/(CT_AMPLE/2);
        } else {                                // girem a la dreta
          if (roda=='R') vg=-velo*(CT_AMPLE/2-radi)/(CT_AMPLE/2);
          else vg=velo*(CT_AMPLE/2+radi)/(CT_AMPLE/2);
        }
      }
    }
  }  
  return vg;
}

void LSMt::MtArrancaMotor(char quin,char com,float velo) {
  if (quin=='R') {
    if (com=='F') {
      digitalWrite(RWfront,FRONT);
      Rspeed=MtVelocitat(fabs(velo),quin);
    } else {
      digitalWrite(RWfront,BACK);
      Rspeed=MtVelocitat(-fabs(velo),quin);
    }
    OCR4A=Rspeed;
    digitalWrite(RWenable,HIGH);
  } else {
    if (com=='F') {
      digitalWrite(LWfront,FRONT);
      Lspeed=MtVelocitat(fabs(velo),quin);
    } else {
      digitalWrite(LWfront,BACK);
      Lspeed=MtVelocitat(-fabs(velo),quin);
    }
    OCR4B=Lspeed;
    digitalWrite(LWenable,HIGH);
  }
}

void LSMt::MtParaMotor(char quin) {
  if (quin=='R') {
    Rspeed = 0x0000;
    digitalWrite(RWenable,LOW);
  } else {
    Lspeed = 0x0000;
    digitalWrite(LWenable,LOW);
  }
}


// Ajust PID de la velocitat
// Speed (tics/s):        velocitat actual que es modifica (comanda usada)
// TeoSpace (tics):       espai teoric que s'hauria d'haver recorregut
// RealSpace(tics):       espai real que s'ha recorregut (llegit dels encoders)
// TeoSpeed (tics/s):     velocitat teorica que s'hauria de portar
// RealSpeed (tics/s):    velocitat real que es porta (llegit dels encoders)
// TeoAccel (tics/s^2):   acceleracio teorica que s'hauria de portar
// RealAccel (tics/s^2):  acceleracio real que es porta (llegit dels encoders)
// MaxSpeed (tics/s):     velocitat maxima a la que pot arribar el motor
  //
float LSMt::MtAjustaVelocitat(float Speed,float TeoSpace,float RealSpace,float TeoSpeed,float RealSpeed,float TeoAccel,float RealAccel,float MaxSpeed) {
  float DifSpace,DifSpeed,DifAccel;
  float aux = Speed;
/*
  DifSpace = TeoSpace - RealSpace;
  DifSpeed = TeoSpeed - RealSpeed;
  DifAccel = TeoAccel - RealAccel;

// 0 0 0.8 0 0 0
//  Speed += 0*DifSpace + 0*DifSpace*DifSpace + 1.2*DifSpeed + 0*DifSpeed*DifSpeed + 0*DifAccel + 0*DifAccel*DifAccel;

  if (DifSpeed>20) Speed+=40;
  else if (DifSpeed>15) Speed+=30;
  else if (DifSpeed>10) Speed+=20;
  else if (DifSpeed>5) Speed+=10;
    else if (DifSpeed>0) Speed+=2;
  else if (DifSpeed>-5) Speed-=2;
  else if (DifSpeed>-10) Speed-=10;
  else if (DifSpeed>-15) Speed-=20;
  else if (DifSpeed>-20) Speed-=30;
  else Speed-=40;

  if (Speed>MaxSpeed) Speed = MaxSpeed;
  if (Speed<0) Speed = 0;
  //Serial.print("[DifSpace: "); Serial.print(DifSpace); Serial.print("  DifSpeed: "); Serial.print(DifSpeed); Serial.print("]  "); Serial.print(aux); Serial.print(" --> "); Serial.print(Speed);
*/
  return Speed;
}






// INICIALITZACIO I MOTOR
// ----------------------

void LSMt::begin(void) {
  int i;
  
  ticsR = 0;
  ticsL = 0;
  // ENABLE
  pinMode(LWenable,OUTPUT);
  digitalWrite(LWenable,LOW); // sense senyal
  pinMode(RWenable,OUTPUT);
  digitalWrite(RWenable,LOW); // sense senyal
  // PWM
  pinMode(LWpwm,OUTPUT);
  digitalWrite(LWpwm,LOW); // sense senyal
  pinMode(RWpwm,OUTPUT);
  digitalWrite(RWpwm,LOW); // sense senyal
  // SENTIT
  pinMode(LWfront,OUTPUT);
  digitalWrite(LWfront,FRONT); // HIGH-endavant; LOW-endarrera
  pinMode(RWfront,OUTPUT);
  digitalWrite(RWfront,FRONT); // HIGH-endavant; LOW-endarrera
//  ENC_Init();
  // INTERRUPTS
  noInterrupts();
  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4  = 0;
  OCR4A = 0x0000;
  OCR4B = 0x0000;
  TCCR4A = 0b00000011;
//  TCCR3B = 0b00001101;
  TCCR4B = 0b00001100;
  TIMSK4 = 0b00000111;
  interrupts();               // All interrupts enabled
  ENC_Init();
  Lspeed = 0x0000;
  Rspeed = 0x0000;
  // limits
  MaxTicsLF = 850;
  MaxTicsRF = 850;
  MaxTicsLB = 850;
  MaxTicsRB = 850;
  MaxSpeedLF = 1023;
  MaxSpeedRF = 1023;
  MaxSpeedLB = 1023;
  MaxSpeedRB = 1023;
  MinSpeedLF = 0;
  MinSpeedRF = 0;
  MinSpeedLB = 0;
  MinSpeedRB = 0;
  PercLF = 1;
  PercRF = 1;
  PercLB = 1;
  PercRB = 1;
  MaxTicsF = 850;
  MaxTicsB = 850;
}


void LSMt::LsMtMotor(void) {
  static int state=1;
  unsigned long Ttics,Tticsref;
  static float novaL,novaR;

  ENC_Motor();
  if (MT.actiu==0) state=1;
  if (MT.actiu==-1) state=50;
  switch(state) {
    case 0: // obtenir timer
            state++;
            break;
    case 1: // discernir operacio nova
            if (MT.actiu==1) {
              ENC_Reset();
              if (MT.Rdir==1) digitalWrite(RWfront,FRONT);
              else digitalWrite(RWfront,BACK);
              if (MT.Ldir==1) digitalWrite(LWfront,FRONT);
              else digitalWrite(LWfront,BACK);
              digitalWrite(RWenable,HIGH);
              digitalWrite(LWenable,HIGH);
              if (MT.accel==1) {            // acceleracio
                state=10;
              } else {
                if (MT.space>0) state=20;   // velocitat constant
                else state=30;              // frenada
              }
              Tticsref=millis();
              // valors reals que es van acumulant
              MT.actTtics = 0;                      // tics acumulats del timer
              MT.actTticsInc = 0;                   // increment de tics del timer
              MT.actLtics = 0;                      // espai: tics actuals acumulats motor esquerre
              MT.actRtics = 0;                      // espai: tics actuals acumulats motor dret
              MT.actLticsInc = 0;                   // velocitat: increment instantani de tics del motor esquerre
              MT.actRticsInc = 0;                   // velocitat: increment instantani de tics del motor dret
              MT.actLticsIncInc = 0;                // acceleracio: increment instantani de velocitat del motor esquerre
              MT.actRticsIncInc = 0;                // acceleracio: increment instantani de velocitat del motor dret
            }
            break;
    case 10: // acceleracio
            MT.actiu=2;
           // valors teorics que es van acumulant
            MT.teoTtics = 0;                                   // milisegons que hauria de durar (milisegons)
            MT.teoRtics = 0;                                   // tics acumulats d'encoder que hauria de fer el motor dret (espai)
            MT.teoLtics = 0;                                   // tics acumulats d'encoder que hauria de fer el motor esquerre (espai)
            // valors reals que es van acumulant
            MT.actTtics = 0;
            MT.actRtics = 0;
            MT.actLtics = 0;
            // valors teorics fixes
            MT.teoTticsInc = 25;                                          // milisegons d'un increment de calcul (milisegons)
            MT.teoLticsIncInc = MT.accL;                         // tics d'encoder d'un increment que hauria de sumar la velocitat del motor esquerre (acceleracio)
            MT.teoRticsIncInc = MT.accR;                         // tocs d'encoder d'un increment que hauria de sumar la velocitat del motor dret (acceleracio)
//            MT.teoRticsInc = MT.initRveloc*MT.teoTticsInc/(float)1000;    // tics d'encoder d'un increment que hauria de fer el motor dret (velocitat)
//            MT.teoLticsInc = MT.initLveloc*MT.teoTticsInc/(float)1000;    // tics d'encoder d'un increment que hauria de fer el motor esquerre (velocitat)
            // comandes motors
            novaR = MT.initRveloc;
            novaL = MT.initLveloc;
            // arranquem
            Rspeed = MtVelocitat(MT.initRveloc,'R');
            Lspeed = MtVelocitat(MT.initLveloc,'L');
            OCR4A=Rspeed;
            OCR4B=Lspeed;
            state++;
            break;
    case 11:
            if ((MT.actLtics>=MT.Ltics) && (MT.actRtics>=MT.Rtics)) {
              if (MT.space>0) state=20;
              else if (MT.frena==1) state=30;
              else state=40;
            } else {
              Ttics=millis()-Tticsref;
              if (Ttics>=MT.teoTticsInc) {
                // tics temporals
                MT.actTticsInc = Ttics; 
                MT.teoTtics += MT.teoTticsInc;
                MT.actTtics += Ttics;
                Tticsref=millis();
                // tics encoder - acceleracio (tics/50ms) (parcial)
                MT.actRticsIncInc = -MT.actRticsInc;
                MT.actLticsIncInc = -MT.actLticsInc;
                // tics encoder - velocitat (tics/50ms)
                MT.actRticsInc = abs(ticsR) - MT.actRtics;
                MT.actLticsInc = abs(ticsL) - MT.actLtics;
                // tics encoder - acceleracio (tics/50ms)
                MT.actRticsIncInc += MT.actRticsInc;
                MT.actLticsIncInc += MT.actLticsInc;
                // espai calculat (parcial)
                MT.teoRtics += MT.teoRticsInc * MT.teoTticsInc/1000.0;
                MT.teoLtics += MT.teoLticsInc * MT.teoTticsInc/1000.0;
                // velocitat calculada
                MT.teoRticsInc += MT.teoRticsIncInc * MT.teoTticsInc / 1000.0;                  // tics d'encoder d'un increment que hauria de fer el motor dret (velocitat)
                MT.teoLticsInc += MT.teoLticsIncInc * MT.teoTticsInc / 1000.0;                  // tics d'encoder d'un increment que hauria de fer el motor esquerre (velocitat)
                // tics encoder - espai
                MT.actRtics = abs(ticsR);
                MT.actLtics = abs(ticsL);
                // espai calculat
                MT.teoRtics += MT.teoRticsIncInc * MT.teoTticsInc * MT.teoTticsInc / 1000000.0;    // tics d'encoder del motor dret (espai)
                MT.teoLtics += MT.teoLticsIncInc * MT.teoTticsInc * MT.teoTticsInc / 1000000.0;    // tics d'encoder del motor esquerre (espai)
                // reajust de velocitat
//Serial.print("L-> ");
                if (MT.Ldir==1) novaL = MtAjustaVelocitat(novaL,MT.teoLtics,MT.actLtics,MT.teoLticsInc,MT.actLticsInc,MT.teoLticsIncInc,MT.actLticsIncInc,MaxTicsF);
                else novaL = MtAjustaVelocitat(novaL,MT.teoLtics,MT.actLtics,MT.teoLticsInc,MT.actLticsInc,MT.teoLticsIncInc,MT.actLticsIncInc,MaxTicsB);
//Serial.print("   R-> ");
                if (MT.Rdir==1) novaR = MtAjustaVelocitat(novaR,MT.teoRtics,MT.actRtics,MT.teoRticsInc,MT.actRticsInc,MT.teoRticsIncInc,MT.actRticsIncInc,MaxTicsF);
                else novaR = MtAjustaVelocitat(novaR,MT.teoRtics,MT.actRtics,MT.teoRticsInc,MT.actRticsInc,MT.teoRticsIncInc,MT.actRticsIncInc,MaxTicsB);
//Serial.println("");
                Rspeed = MtVelocitat(novaR,'R');
                Lspeed = MtVelocitat(novaL,'L');
                OCR4A=Rspeed;
                OCR4B=Lspeed;
              }
            }
            break;
    case 20: // velocitat constant
            MT.actiu=3;
            // valors teorics que es van acumulant
            MT.teoTtics = 0;                                   // milisegons que hauria de durar (milisegons)
            MT.teoRtics = 0;                                   // tics acumulats d'encoder que hauria de fer el motor dret (espai)
            MT.teoLtics = 0;                                   // tics acumulats d'encoder que hauria de fer el motor esquerre (espai)
            // valors reals que es van acumulant
            MT.actTtics = 0;
            MT.actRtics = 0;
            MT.actLtics = 0;
            // valors teorics fixes
            MT.teoTticsInc = 25;                               // milisegons d'un increment de calcul (milisegons)
            MT.teoRticsInc = MT.Rveloc*MT.teoTticsInc/(float)1000;    // tics d'encoder d'un increment que hauria de fer el motor dret (velocitat)
            MT.teoLticsInc = MT.Lveloc*MT.teoTticsInc/(float)1000;    // tics d'encoder d'un increment que hauria de fer el motor esquerre (velocitat)
            // comandes motors
            novaR = MT.Rveloc;
            novaL = MT.Lveloc;
            // arranquem
            Rspeed = MtVelocitat(MT.Rveloc,'R');
            Lspeed = MtVelocitat(MT.Lveloc,'L');
            OCR4A=Rspeed;
            OCR4B=Lspeed;
/*
Serial.print("LTicsTeoInc: "); Serial.println(MT.teoLticsInc);
Serial.print("RTicsTeoInc: "); Serial.println(MT.teoRticsInc);
Serial.print("Lveloc: "); Serial.println(MT.Lveloc);
Serial.print("Rveloc: "); Serial.println(MT.Rveloc);
Serial.print("Lpeed: "); Serial.println(MT.Lspeed);
Serial.print("Rpeed: "); Serial.println(MT.Rspeed);
*/
            state++;
            break;
/*
    case 21:
            if ((MT.actLtics>=MT.Ltics) && (MT.actRtics>=MT.Rtics)) {
              if (MT.frena==1) state=30;
              else state=40;
            } else {
              Ttics=millis()-Tticsref;
              if (Ttics>=MT.teoTticsInc) {
                // tics temporals
                MT.actTticsInc = Ttics; 
                MT.teoTtics += MT.teoTticsInc;
                MT.actTtics += Ttics;
                Tticsref=millis();

                // tics encoder - velocitat (tics/50ms)
                MT.actRticsInc = abs(ticsR) - MT.actRtics;
                MT.actLticsInc = abs(ticsL) - MT.actLtics;

                // tics encoder - espai
                MT.actRtics = abs(ticsR);
                MT.actLtics = abs(ticsL);
                MT.teoRtics += MT.teoRticsInc;
                MT.teoLtics += MT.teoLticsInc;

                // reajust de velocitat
//Serial.print("L-> ");
                if (MT.Ldir==1) novaL = MtAjustaVelocitat(novaL,MT.teoLtics,MT.actLtics,MT.teoLticsInc,MT.actLticsInc,0,0,MaxTicsF);
                else novaL = MtAjustaVelocitat(novaL,MT.teoLtics,MT.actLtics,MT.teoLticsInc,MT.actLticsInc,0,0,MaxTicsB);
//Serial.print("   R-> ");
                if (MT.Rdir==1) novaR = MtAjustaVelocitat(novaR,MT.teoRtics,MT.actRtics,MT.teoRticsInc,MT.actRticsInc,0,0,MaxTicsF);
                else novaR = MtAjustaVelocitat(novaR,MT.teoRtics,MT.actRtics,MT.teoRticsInc,MT.actRticsInc,0,0,MaxTicsB);
//Serial.println("");

                Rspeed = MtVelocitat(novaR,'R');
                Lspeed = MtVelocitat(novaL,'L');
                OCR4A=Rspeed;
                OCR4B=Lspeed;
              }
            }
            break;
*/

    case 21:
            if ((MT.actLtics>=MT.Ltics) && (MT.actRtics>=MT.Rtics)) {
              if (MT.frena==1) state=30;
              else state=40;
            } else {
              Ttics=millis()-Tticsref;
              if (Ttics>=MT.teoTticsInc) {
                // tics temporals
                MT.actTticsInc = Ttics; 
                MT.teoTtics += MT.teoTticsInc;
                MT.actTtics += Ttics;
                Tticsref=millis();
                // tics encoder - velocitat (tics/50ms)
                MT.actRticsInc = abs(ticsR) - MT.actRtics;
                MT.actLticsInc = abs(ticsL) - MT.actLtics;
                // tics encoder - espai
                MT.actRtics = abs(ticsR);
                MT.actLtics = abs(ticsL);
                MT.teoRtics += MT.teoRticsInc;
                MT.teoLtics += MT.teoLticsInc;
                // reajust de velocitat
//Serial.print("L-> ");
                if (MT.Ldir==1) novaL = MtAjustaVelocitat(novaL,MT.teoLtics,MT.actLtics,MT.teoLticsInc,MT.actLticsInc,0,0,MaxTicsF);
                else novaL = MtAjustaVelocitat(novaL,MT.teoLtics,MT.actLtics,MT.teoLticsInc,MT.actLticsInc,0,0,MaxTicsB);
//Serial.print("   R-> ");
                if (MT.Rdir==1) novaR = MtAjustaVelocitat(novaR,MT.teoRtics,MT.actRtics,MT.teoRticsInc,MT.actRticsInc,0,0,MaxTicsF);
                else novaR = MtAjustaVelocitat(novaR,MT.teoRtics,MT.actRtics,MT.teoRticsInc,MT.actRticsInc,0,0,MaxTicsB);
//Serial.println("");
                Rspeed = MtVelocitat(novaR,'R');
                Lspeed = MtVelocitat(novaL,'L');
                OCR4A=Rspeed;
                OCR4B=Lspeed;
              }
            }
            break;


    case 30: // frenada
            MT.actiu=4;
           // valors teorics que es van acumulant
            MT.teoTtics = 0;                                   // milisegons que hauria de durar (milisegons)
            MT.teoRtics = 0;                                   // tics acumulats d'encoder que hauria de fer el motor dret (espai)
            MT.teoLtics = 0;                                   // tics acumulats d'encoder que hauria de fer el motor esquerre (espai)
            // valors reals que es van acumulant
            MT.actTtics = 0;
            MT.actRtics = 0;
            MT.actLtics = 0;
            // valors teorics fixes
            MT.teoTticsInc = 25;                                          // milisegons d'un increment de calcul (milisegons)
            MT.teoLticsIncInc = MT.freL;                         // tics d'encoder d'un increment que hauria de sumar la velocitat del motor esquerre (acceleracio)
            MT.teoRticsIncInc = MT.freR;                         // tocs d'encoder d'un increment que hauria de sumar la velocitat del motor dret (acceleracio)
//            MT.teoRticsInc = MT.Rveloc*MT.teoTticsInc/(float)1000;    // tics d'encoder d'un increment que hauria de fer el motor dret (velocitat)
//            MT.teoLticsInc = MT.Lveloc*MT.teoTticsInc/(float)1000;    // tics d'encoder d'un increment que hauria de fer el motor esquerre (velocitat)
            // comandes motors
            novaR = MT.Rveloc;
            novaL = MT.Lveloc;
            // arranquem
            Rspeed = MtVelocitat(MT.Rveloc,'R');
            Lspeed = MtVelocitat(MT.Lveloc,'L');
            OCR4A=Rspeed;
            OCR4B=Lspeed;
            state++;
            break;
    case 31:
            if ((MT.actLtics>=MT.Ltics) && (MT.actRtics>=MT.Rtics)) {
              state=40;
            } else {
              Ttics=millis()-Tticsref;
              if (Ttics>=MT.teoTticsInc) {
                // tics temporals
                MT.actTticsInc = Ttics; 
                MT.teoTtics += MT.teoTticsInc;
                MT.actTtics += Ttics;
                Tticsref=millis();
                // tics encoder - frenada (tics/50ms) (parcial)
                MT.actRticsIncInc = -MT.actRticsInc;
                MT.actLticsIncInc = -MT.actLticsInc;
                // tics encoder - velocitat (tics/50ms)
                MT.actRticsInc = abs(ticsR) - MT.actRtics;
                MT.actLticsInc = abs(ticsL) - MT.actLtics;
                // tics encoder - acceleracio (tics/50ms)
                MT.actRticsIncInc += MT.actRticsInc;
                MT.actLticsIncInc += MT.actLticsInc;
                // espai calculat (parcial)
                MT.teoRtics += MT.teoRticsInc * MT.teoTticsInc/1000.0;
                MT.teoLtics += MT.teoLticsInc * MT.teoTticsInc/1000.0;
                // velocitat calculada
                MT.teoRticsInc += MT.teoRticsIncInc * MT.teoTticsInc / 1000.0;                  // tics d'encoder d'un increment que hauria de fer el motor dret (velocitat)
                MT.teoLticsInc += MT.teoLticsIncInc * MT.teoTticsInc / 1000.0;                  // tics d'encoder d'un increment que hauria de fer el motor esquerre (velocitat)
                // tics encoder - espai
                MT.actRtics = abs(ticsR);
                MT.actLtics = abs(ticsL);
                // espai calculat
                MT.teoRtics += MT.teoRticsIncInc * MT.teoTticsInc * MT.teoTticsInc / 1000000.0;    // tics d'encoder del motor dret (espai)
                MT.teoLtics += MT.teoLticsIncInc * MT.teoTticsInc * MT.teoTticsInc / 1000000.0;    // tics d'encoder del motor esquerre (espai)
                // reajust de velocitat
//Serial.print("L-> ");
                if (MT.Ldir==1) novaL = MtAjustaVelocitat(novaL,MT.teoLtics,MT.actLtics,MT.teoLticsInc,MT.actLticsInc,MT.teoLticsIncInc,MT.actLticsIncInc,MaxTicsF);
                else novaL = MtAjustaVelocitat(novaL,MT.teoLtics,MT.actLtics,MT.teoLticsInc,MT.actLticsInc,MT.teoLticsIncInc,MT.actLticsIncInc,MaxTicsB);
//Serial.print("   R-> ");
                if (MT.Rdir==1) novaR = MtAjustaVelocitat(novaR,MT.teoRtics,MT.actRtics,MT.teoRticsInc,MT.actRticsInc,MT.teoRticsIncInc,MT.actRticsIncInc,MaxTicsF);
                else novaR = MtAjustaVelocitat(novaR,MT.teoRtics,MT.actRtics,MT.teoRticsInc,MT.actRticsInc,MT.teoRticsIncInc,MT.actRticsIncInc,MaxTicsB);
//Serial.println("");
                Rspeed = MtVelocitat(novaR,'R');
                Lspeed = MtVelocitat(novaL,'L');
                OCR4A=Rspeed;
                OCR4B=Lspeed;
              }
            }
            break;
    case 40: digitalWrite(LWenable,LOW);
             digitalWrite(RWenable,LOW);
             MT.actiu=0;
             state = 1;
             break;
    case 50:  // quan es funciona amb el mando... assegurem 500 milis de la darrera instrucció en cas que hi hagi desconnexió del comandament
              if (millis()-ticsMando>=500) {    // un cop passat aquest temps es paren els motors.
                digitalWrite(LWenable,LOW);
                digitalWrite(RWenable,LOW);
                MT.actiu=0;
                state = 1;
                break;
              }
              break;
  }
}


// FUNCIONS PUBLIQUES
// ------------------

int LSMt::LsMtHiHaOrdreActiva(void) {
  return MT.actiu;
}


extern int MaxTicsLF, MaxTicsRF, MaxTicsLB, MaxTicsRB;
extern int MaxSpeedLF, MaxSpeedRF, MaxSpeedLB, MaxSpeedRB;
extern int MinSpeedLF, MinSpeedRF, MinSpeedLB, MinSpeedRB;



/*
// per si volguessim usar els encoders per a realimentar el control dels motors quan s'usa el mando (mobil)
          static float Tr, Tl, OTr, OTl;

          OTl = Motoreta.ENC_VelocitatLeft() ;
          OTr = Motoreta.ENC_VelocitatRight() ;
*/

/*
          Tl = Motoreta.ENC_VelocitatLeft() ;
          Tr = Motoreta.ENC_VelocitatRight() ;
          if (Tl > 0 && OTl > 0) PercLF = Tl / OTl;
          if (Tl < 0 && OTl < 0) PercLB = Tl / OTl;
          if (Tr > 0 && OTr > 0) PercRF = Tr / OTr;
          if (Tr < 0 && OTr < 0) PercRB = Tr / OTr;
*/


// CONTROL del MANDO
void LSMt::LsMtMando(float LeftSpeed,float RightSpeed,int push) {
  if (push==1) {
    MT.actiu = -1;
    digitalWrite(RWenable,HIGH);
    digitalWrite(LWenable,HIGH);
    if (RightSpeed>=0) {
      digitalWrite(RWfront,FRONT);
      Rspeed = (int)((float)MinSpeedRF+(float)(RightSpeed*(MaxSpeedRF-MinSpeedRF))/100.0);
      if (Rspeed>MaxSpeedRF) Rspeed = MaxSpeedRF;
      //      RightSpeed = (int)((float)MinSpeedRF+(float)(RightSpeed*(MaxSpeedRF-MinSpeedRF))/100.0);
      //      if (RightSpeed>MaxSpeedRF) RightSpeed = MaxSpeedRF;
      //      Rspeed = MtVelocitat(RightSpeed*MaxTicsF/100.0,'R');
    } else {
      digitalWrite(RWfront,BACK);
      Rspeed = (int)((float)MinSpeedRB+(float)(RightSpeed*(MaxSpeedRB-MinSpeedRB))/100.0);
      if (Rspeed>MaxSpeedRB) Rspeed = MaxSpeedRB;
      //      RightSpeed = (int)((float)MinSpeedRB+(float)(RightSpeed*(MaxSpeedRB-MinSpeedRB))/100.0);
      //      if (RightSpeed>MaxSpeedRB) RightSpeed = MaxSpeedRB;
      //      Rspeed = MtVelocitat(RightSpeed*MaxTicsB/100.0,'R');
    }
    if (LeftSpeed>=0) {
      digitalWrite(LWfront,FRONT);
      Lspeed = (int)((float)MinSpeedLF+(float)(LeftSpeed*(MaxSpeedLF-MinSpeedLF))/100.0);
      if (Lspeed>MaxSpeedLF) Lspeed = MaxSpeedLF;
      //      LeftSpeed = (int)((float)MinSpeedLF+(float)(LeftSpeed*(MaxSpeedLF-MinSpeedLF))/100.0);
      //      if (LeftSpeed>MaxSpeedLF) LeftSpeed = MaxSpeedLF;
      //      Lspeed = MtVelocitat(LeftSpeed*MaxTicsF/100.0,'L');
    } else {
      digitalWrite(LWfront,BACK);
      Lspeed = (int)((float)MinSpeedLB+(float)(LeftSpeed*(MaxSpeedLB-MinSpeedLB))/100.0);
      if (Lspeed>MaxSpeedLB) Lspeed = MaxSpeedLB;
      //      LeftSpeed = (int)((float)MinSpeedLB+(float)(LeftSpeed*(MaxSpeedLB-MinSpeedLB))/100.0);
      //      if (LeftSpeed>MaxSpeedLB) LeftSpeed = MaxSpeedLB;
      //      Lspeed = MtVelocitat(LeftSpeed*MaxTicsB/100.0,'L');
    }
      //Lspeed=fabs(LeftSpeed);
      //Rspeed=fabs(RightSpeed);
    OCR4A=Rspeed;
    OCR4B=Lspeed;
  } else {
    digitalWrite(RWenable,LOW);
    digitalWrite(LWenable,LOW);
    MT.actiu=0;
  }
  ticsMando=millis();
}




// RECTE
//    unitats:       cm/s            cm              cm/s        cm          cm/s           cm
void LSMt::LsMtRecte(float initVeloc,float initSpace,float veloc,float space,float endVeloc,float endSpace) {
  if(MT.actiu==0) {
    // dades originals
    MT.actiu = 1;
    MT.direc = 'F';
    if ((int)(MT.initspace)==0) {
      MT.accel = 0;
    } else {
      MT.accel=1;    
      MT.initvelo = initVeloc;
      MT.initspace = initSpace;
    }
    MT.veloc = veloc;
    MT.space = space;
    MT.radi = 100000;
    if ((int)(MT.endspace)==0) {
      MT.frena = 0;
    } else {
      MT.frena=1;    
      MT.endveloc = endVeloc;
      MT.endspace = endSpace;
    }
    // calcular els altres parametres que es deriven dels inicials
    if (MT.space>0) {
      if (MT.veloc>=0) { MT.Ldir = 1; MT.Rdir = 1; }
      else { MT.Ldir = -1; MT.Rdir = -1; }
    } else {
      if (MT.accel==1) {
        if (MT.initvelo>=0) { MT.Ldir = 1; MT.Rdir = 1; }
        else { MT.Ldir = -1; MT.Rdir = -1; }
      } else {
        if (MT.endveloc>=0) { MT.Ldir = 1; MT.Rdir = 1; }
        else { MT.Ldir = -1; MT.Rdir = -1; }
      }
    }
    if (MT.space>0) {
      MT.Lveloc = MtEspaiTics(MT.veloc);                                                 // ticsenc/s (+/-)      <--   cm/s (+/-)
      MT.Rveloc = MtEspaiTics(MT.veloc);                                                 // ticsenc/s (+/-)      <--   cm/s (+/-)
      MT.Lspeed = MtVelocitat(MT.Lveloc,'L');                                            // 0x0??? (+)         <--   (ticsenc/s) (+/-)
      MT.Rspeed = MtVelocitat(MT.Rveloc,'R');                                            // 0x0??? (+)         <--   (ticsenc/s) (+/-)
      MT.Ltics = MtEspaiTics(MT.space);                                                  // tics (+)           <--   (cm) (+)
      MT.Rtics = MtEspaiTics(MT.space);                                                  // tics (+)           <--   (cm) (+)
//Serial.print("[CALCULATS]   -->  [Lspa: "); Serial.print(MT.Ltics); Serial.print("  Lspe: "); Serial.print(MT.Lveloc); //Serial.print("  Lmot: "); Serial.print(MT.Lspeed); 
//Serial.print("]     [Rspa: "); Serial.print(MT.Rtics); Serial.print("  Rspe: "); Serial.println(MT.Rveloc); //Serial.print("  Rmot: "); Serial.print(MT.Rspeed); Serial.println("]"); 
    }
    if (MT.accel==1) {
      MT.initLveloc = MtEspaiTics(MT.initvelo);                                          // ticsenc/s      <--   cm/s
      MT.initRveloc = MtEspaiTics(MT.initvelo);                                          // ticsenc/s      <--   cm/s
      MT.initLspeed = MtVelocitat(MT.initLveloc,'L');                                    // 0x0???         <--   (ticsenc/s)
      MT.initRspeed = MtVelocitat(MT.initRveloc,'R');                                    // 0x0???         <--   (ticsenc/s)
      MT.initLtics = MtEspaiTics(MT.initspace);                                          // tics           <--   (cm)
      MT.initRtics = MtEspaiTics(MT.initspace);                                          // tics           <--   (cm)
      MT.accL = MtAcceleracio(MT.initLveloc,MT.Lveloc,MT.initLtics);                     // ticsenc/s^2    <--   (ticsenc/s;ticsenc/s;ticsenc)
      MT.accR = MtAcceleracio(MT.initRveloc,MT.Rveloc,MT.initRtics);                     // ticsenc/s^2    <--   (ticsenc/s;ticsenc/s;ticsenc)
    }
    if (MT.frena==1) {
      MT.endLveloc = MtEspaiTics(MT.endveloc);                                           // ticsenc/s      <--   cm/s
      MT.endRveloc = MtEspaiTics(MT.endveloc);                                           // ticsenc/s      <--   cm/s
      MT.endLspeed = MtVelocitat(MT.endLveloc,'L');                                     // 0x0???         <--   (ticsenc/s)
      MT.endRspeed = MtVelocitat(MT.endRveloc,'R');                                     // 0x0???         <--   (ticsenc/s)
      MT.endLtics = MtEspaiTics(MT.endspace);                                            // tics           <--   (cm)
      MT.endRtics = MtEspaiTics(MT.endspace);                                            // tics           <--   (cm)
      MT.freL = MtAcceleracio(MT.Lveloc,MT.endLveloc,MT.endLtics);                       // ticsenc/s^2    <--   (ticsenc/s;ticsenc/s;ticsenc)
      MT.freR = MtAcceleracio(MT.Rveloc,MT.endRveloc,MT.endRtics);                       // ticsenc/s^2    <--   (ticsenc/s;ticsenc/s;ticsenc)
    }
  }
}


// GIRA DRETA
// si initGraus �s 0 significa que no hi ha acceleraci�
// endGraus �s 0 indica que no hi ha frenada
// Graus sempre positius
// velocitat positiva o negativa segons sentit de la marxa
void LSMt::LsMtDreta(float radi,float initVeloc,float initGraus,float veloc,float graus,float endVeloc,float endGraus) {
  if(MT.actiu==0) {
    // dades originals
    MT.actiu = 1;
    MT.direc = 'R';
    if ((int)(MT.initspace)==0) {
      MT.accel = 0;
    } else {
      MT.accel=1;    
      MT.initvelo = initVeloc;
      MT.initspace = initGraus;
    }
    MT.veloc = veloc;
    MT.space = graus;
    MT.radi = radi;
    if ((int)(MT.endspace)==0) {
      MT.frena = 0;
    } else {
      MT.frena=1;    
      MT.endveloc = endVeloc;
      MT.endspace = endGraus;
    }
    // calcular els altres parametres que es deriven dels inicials
    if (MT.space>0) {
      MT.Lveloc = MtVelocitatGir('R',MT.radi,'L',MT.veloc);                              // ticsenc/s      <--   (graus;cm/s)
      MT.Rveloc = MtVelocitatGir('R',MT.radi,'R',MT.veloc);                              // ticsenc/s      <--   (graus;cm/s)
      if (MT.Lveloc>=0) MT.Ldir = 1; else MT.Ldir = -1;
      if (MT.Rveloc>=0) MT.Rdir = 1; else MT.Rdir = -1;
    } else {
      if (MT.accel==1) {
        MT.initLveloc = MtVelocitatGir('R',MT.radi,'L',MT.initvelo);                       // ticsenc/s      <--   (graus;cm/s)
        MT.initRveloc = MtVelocitatGir('R',MT.radi,'R',MT.initvelo);                       // ticsenc/s      <--   (graus;cm/s)
        if (MT.initLveloc>=0) MT.Ldir = 1; else MT.Ldir = -1;
        if (MT.initRveloc>=0) MT.Rdir = 1; else MT.Rdir = -1;
      } else {
        MT.endLveloc = MtVelocitatGir('R',MT.radi,'L',MT.endveloc);                        // ticsenc/s      <--   (graus;cm/s)
        MT.endRveloc = MtVelocitatGir('R',MT.radi,'R',MT.endveloc);                        // ticsenc/s      <--   (graus;cm/s)
        if (MT.endLveloc>=0) MT.Ldir = 1; else MT.Ldir = -1;
        if (MT.endRveloc>=0) MT.Rdir = 1; else MT.Rdir = -1;
      }
    }
    if (MT.space>0) {
      MT.Lspeed = MtVelocitat(MT.Lveloc,'L');                                            // 0x0???         <--   (ticsenc/s)
      MT.Rspeed = MtVelocitat(MT.Rveloc,'R');                                            // 0x0???         <--   (ticsenc/s)
      MT.Ltics = MtEspaiTics(MT.space*MT.radi*PI/180.0);                                 // tics           <--   (cm;graus)
      MT.Rtics = MtEspaiTics(MT.space*MT.radi*PI/180.0);                                 // tics           <--   (cm;graus)  
    }
    if (MT.accel==1) {
      MT.initLspeed = MtVelocitat(MT.initLveloc,'L');                                    // 0x0???         <--   (ticsenc/s)
      MT.initRspeed = MtVelocitat(MT.initRveloc,'R');                                    // 0x0???         <--   (ticsenc/s)
      MT.initLtics = MtEspaiTics(MT.initspace*MT.radi*PI/180.0*MT.initLveloc/MT.initvelo);      // tics           <--   (cm)   // ???
      MT.initRtics = MtEspaiTics(MT.initspace*MT.radi*PI/180.0*MT.initRveloc/MT.initvelo);      // tics           <--   (cm)   // ???
      MT.accL = MtAcceleracio(MT.initLveloc,MT.Lveloc,MT.initLtics);                     // ticsenc/s^2    <--   (ticsenc/s;ticsenc/s;ticsenc)
      MT.accR = MtAcceleracio(MT.initRveloc,MT.Rveloc,MT.initRtics);                     // ticsenc/s^2    <--   (ticsenc/s;ticsenc/s;ticsenc)
    }
    if (MT.frena==1) {
      MT.endLspeed = MtVelocitat(MT.endLveloc,'L');                                      // 0x0???         <--   (ticsenc/s)
      MT.endRspeed = MtVelocitat(MT.endRveloc,'R');                                      // 0x0???         <--   (ticsenc/s)
      MT.endLtics = MtEspaiTics(MT.endspace*MT.radi*PI/180.0*MT.endLveloc/MT.endveloc);       // tics           <--   (cm)   // ???
      MT.endRtics = MtEspaiTics(MT.endspace*MT.radi*PI/180.0*MT.endRveloc/MT.endveloc);       // tics           <--   (cm)   // ???
      MT.freL = MtAcceleracio(MT.Lveloc,MT.endLveloc,MT.endLtics);                       // ticsenc/s^2    <--   (ticsenc/s;ticsenc/s;ticsenc)
      MT.freR = MtAcceleracio(MT.Rveloc,MT.endRveloc,MT.endRtics);                       // ticsenc/s^2    <--   (ticsenc/s;ticsenc/s;ticsenc)
    }
  }
}


// GIRA ESQUERRA
// si initGraus �s 0 significa que no hi ha acceleraci�
// endGraus �s 0 indica que no hi ha frenada
// Graus sempre positius
// velocitat positiva o negativa segons sentit de la marxa
void LSMt::LsMtEsquerra(float radi,float initVeloc,float initGraus,float veloc,float graus,float endVeloc,float endGraus) {
  if(MT.actiu==0) {
    // dades originals
    MT.actiu = 1;
    MT.direc = 'L';
    if ((int)(MT.initspace)==0) {
      MT.accel = 0;
    } else {
      MT.accel=1;    
      MT.initvelo = initVeloc;
      MT.initspace = -initGraus;
    }
    MT.veloc = veloc;
    MT.space = -graus;
    MT.radi = radi;
    if ((int)(MT.endspace)==0) {
      MT.frena = 0;
    } else {
      MT.frena=1;    
      MT.endveloc = endVeloc;
      MT.endspace = -endGraus;
    }
    // calcular els altres parametres que es deriven dels inicials
    if (MT.space>0) {
      MT.Lveloc = MtVelocitatGir('L',MT.radi,'L',MT.veloc);                              // ticsenc/s      <--   (graus;cm/s)
      MT.Rveloc = MtVelocitatGir('L',MT.radi,'R',MT.veloc);                              // ticsenc/s      <--   (graus;cm/s)
      if (MT.Lveloc>=0) MT.Ldir = 1; else MT.Ldir = -1;
      if (MT.Rveloc>=0) MT.Rdir = 1; else MT.Rdir = -1;
    } else {
      if (MT.accel==1) {
        MT.initLveloc = MtVelocitatGir('L',MT.radi,'L',MT.initvelo);                       // ticsenc/s      <--   (graus;cm/s)
        MT.initRveloc = MtVelocitatGir('L',MT.radi,'R',MT.initvelo);                       // ticsenc/s      <--   (graus;cm/s)
        if (MT.initLveloc>=0) MT.Ldir = 1; else MT.Ldir = -1;
        if (MT.initRveloc>=0) MT.Rdir = 1; else MT.Rdir = -1;
      } else {
        MT.endLveloc = MtVelocitatGir('L',MT.radi,'L',MT.endveloc);                        // ticsenc/s      <--   (graus;cm/s)
        MT.endRveloc = MtVelocitatGir('L',MT.radi,'R',MT.endveloc);                        // ticsenc/s      <--   (graus;cm/s)
        if (MT.endLveloc>=0) MT.Ldir = 1; else MT.Ldir = -1;
        if (MT.endRveloc>=0) MT.Rdir = 1; else MT.Rdir = -1;
      }
    }
    if (MT.space>0) {
      MT.Lspeed = MtVelocitat(MT.Lveloc,'L');                                            // 0x0???         <--   (ticsenc/s)
      MT.Rspeed = MtVelocitat(MT.Rveloc,'R');                                            // 0x0???         <--   (ticsenc/s)
      MT.Ltics = MtEspaiTics(MT.space*MT.radi*PI/180.0);                                 // tics           <--   (cm;graus)
      MT.Rtics = MtEspaiTics(MT.space*MT.radi*PI/180.0);                                 // tics           <--   (cm;graus)    
    }
    if (MT.accel==1) {
      MT.initLspeed = MtVelocitat(MT.initLveloc,'L');                                    // 0x0???         <--   (ticsenc/s)
      MT.initRspeed = MtVelocitat(MT.initRveloc,'R');                                    // 0x0???         <--   (ticsenc/s)
      MT.initLtics = MtEspaiTics(MT.initspace*MT.radi*PI/180.0*MT.initLveloc/MT.initvelo);      // tics           <--   (cm)   // ???
      MT.initRtics = MtEspaiTics(MT.initspace*MT.radi*PI/180.0*MT.initRveloc/MT.initvelo);      // tics           <--   (cm)   // ???
      MT.accL = MtAcceleracio(MT.initLveloc,MT.Lveloc,MT.initLtics);                     // ticsenc/s^2    <--   (ticsenc/s;ticsenc/s;ticsenc)
      MT.accR = MtAcceleracio(MT.initRveloc,MT.Rveloc,MT.initRtics);                     // ticsenc/s^2    <--   (ticsenc/s;ticsenc/s;ticsenc)
    }
    if (MT.frena==1) {
      MT.endLspeed = MtVelocitat(MT.endLveloc,'L');                                      // 0x0???         <--   (ticsenc/s)
      MT.endRspeed = MtVelocitat(MT.endRveloc,'R');                                      // 0x0???         <--   (ticsenc/s)
      MT.endLtics = MtEspaiTics(MT.endspace*MT.radi*PI/180.0*MT.endLveloc/MT.endveloc);       // tics           <--   (cm)   // ???
      MT.endRtics = MtEspaiTics(MT.endspace*MT.radi*PI/180.0*MT.endRveloc/MT.endveloc);       // tics           <--   (cm)   // ???
      MT.freL = MtAcceleracio(MT.Lveloc,MT.endLveloc,MT.endLtics);                       // ticsenc/s^2    <--   (ticsenc/s;ticsenc/s;ticsenc)
      MT.freR = MtAcceleracio(MT.Rveloc,MT.endRveloc,MT.endRtics);                       // ticsenc/s^2    <--   (ticsenc/s;ticsenc/s;ticsenc)
    }
  }
}


// amb aquest procediment els motors s'autocalibren de forma molt rapida ajustant els valors de PercXY
// perque funcioni cal haver inicialitzat abans els encoders i els motors
void LSMt::LsMtFastAutocalibrate(void) {
  int spL,spR;
  // inits
  PercLF = 1;
  PercRF = 1;
  PercLB = 1;
  PercRB = 1;
  MaxTicsF = 850;
  MaxTicsB = 850;

  // forward maximum speed
  digitalWrite(LWfront,FRONT);
  digitalWrite(RWfront,FRONT);
  Lspeed=0x03FF;
  OCR4B=Lspeed;
  Rspeed=0x03FF;
  OCR4A=Rspeed;
  digitalWrite(LWenable,HIGH);
  digitalWrite(RWenable,HIGH);
  delay(1000);
  ENC_Reset();
  delay(500);
  ENC_Motor();
  spL = ENC_VelocitatLeft();
  spR = ENC_VelocitatRight();
  Serial.print("FOR. "); Serial.print(spL); Serial.print(" - "); Serial.println(spR);
  if (spL<spR) MaxTicsF = spL;
  else MaxTicsF = spR;
  PercLF = MaxTicsF/(float)spL;
  PercRF = MaxTicsF/(float)spR;
  Serial.print("FOR. "); Serial.print(MaxTicsF); Serial.print(" -> "); Serial.print(PercLF); Serial.print(":"); Serial.println(PercRF);
  digitalWrite(LWenable,LOW);
  digitalWrite(RWenable,LOW);
  delay(500);

  // backward maximum speed
  digitalWrite(LWfront,BACK);
  digitalWrite(RWfront,BACK);
  Lspeed=0x03FF;
  OCR4B=Lspeed;
  Rspeed=0x03FF;
  OCR4A=Rspeed;
  digitalWrite(LWenable,HIGH);
  digitalWrite(RWenable,HIGH);
  delay(1000);
  ENC_Reset();
  delay(500);
  ENC_Motor();
  spL = -ENC_VelocitatLeft();
  spR = -ENC_VelocitatRight();
  Serial.print("BAC. "); Serial.print(spL); Serial.print(" - "); Serial.println(spR);
  if (spL<spR) MaxTicsB = spL;
  else MaxTicsB = spR;
  PercLB = MaxTicsB/(float)spL;
  PercRB = MaxTicsB/(float)spR;
  Serial.print("BAC. "); Serial.print(MaxTicsB); Serial.print(" -> "); Serial.print(PercLB); Serial.print(":"); Serial.println(PercRB);
  digitalWrite(LWenable,LOW);
  digitalWrite(RWenable,LOW);
  delay(500);
}




void LSMt::LsMtCalibrate() {
  float spd,oldspd;

  // forward left
  digitalWrite(LWfront,FRONT);
  Lspeed=0x0000;
  OCR4B=Lspeed;
  digitalWrite(LWenable,HIGH);
  ENC_Reset();
  ENC_Motor(); //LsMtMotor();
  do {
    Lspeed++;
    OCR4B=Lspeed;
    ENC_Motor(); // LsMtMotor();
    delay(50);
    ENC_Motor(); // LsMtMotor();
    delay(50);
    ENC_Motor(); // LsMtMotor();
    spd = ENC_VelocitatLeft();
//Serial.print(Lspeed,HEX); Serial.print(" - "); Serial.println(spd,5);
  } while(spd<1.0);
//Serial.print(Lspeed,HEX); Serial.print(" - "); Serial.println(spd,5);
  MinSpeedLF = Lspeed;
  Lspeed=0x03ff;
  OCR4B=Lspeed;
  ENC_Motor(); // LsMtMotor();
  delay(500);
  ENC_Motor(); // LsMtMotor();
  delay(500);
  ENC_Motor(); // LsMtMotor();
  spd = ENC_VelocitatLeft();    
//Serial.print(Lspeed,HEX); Serial.print(" - "); Serial.println(spd,5);
  MaxSpeedLF = Lspeed;
  MaxTicsLF = spd;
  digitalWrite(LWenable,LOW);

  // forward right
  digitalWrite(RWfront,FRONT);
  Rspeed=0x0000;
  OCR4A=Rspeed;
  digitalWrite(RWenable,HIGH);
  ENC_Reset();
  ENC_Motor(); // LsMtMotor();
  do {
    Rspeed++;
    OCR4A=Rspeed;
    ENC_Motor(); // LsMtMotor();
    delay(50);
    ENC_Motor(); // LsMtMotor();
    delay(50);
    ENC_Motor(); // LsMtMotor();
    spd = ENC_VelocitatRight();
//Serial.print(Lspeed,HEX); Serial.print(" - "); Serial.println(spd,5);
  } while(spd<1.0);
//Serial.print(Rspeed,HEX); Serial.print(" - "); Serial.println(spd,5);
  MinSpeedRF = Rspeed;
  Rspeed=0x03ff;
  OCR4A=Rspeed;
  ENC_Motor(); // LsMtMotor();
  delay(500);
  ENC_Motor(); // LsMtMotor();
  delay(500);
  ENC_Motor(); // LsMtMotor();
  spd = ENC_VelocitatRight();    
//Serial.print(Rspeed,HEX); Serial.print(" - "); Serial.println(spd,5);
  MaxSpeedRF = Rspeed;
  MaxTicsRF = spd;
  digitalWrite(RWenable,LOW);

  // backward left
  digitalWrite(LWfront,BACK);
  Lspeed=0x0000;
  OCR4B=Lspeed;
  digitalWrite(LWenable,HIGH);
  ENC_Reset();
  ENC_Motor(); // LsMtMotor();
  do {
    Lspeed++;
    OCR4B=Lspeed;
    ENC_Motor(); // LsMtMotor();
    delay(50);
    ENC_Motor(); // LsMtMotor();
    delay(50);
    ENC_Motor(); // LsMtMotor();
    spd = ENC_VelocitatLeft();
//Serial.print(Lspeed,HEX); Serial.print(" - "); Serial.println(spd,5);
  } while(spd>-1.0);
//Serial.print(Lspeed,HEX); Serial.print(" - "); Serial.println(spd,5);
  MinSpeedLB = Lspeed;
  Lspeed=0x03ff;
  OCR4B=Lspeed;
  ENC_Motor(); // LsMtMotor();
  delay(500);
  ENC_Motor(); // LsMtMotor();
  delay(500);
  ENC_Motor(); // LsMtMotor();
  spd = ENC_VelocitatLeft();    
//Serial.print(Lspeed,HEX); Serial.print(" - "); Serial.println(spd,5);
  MaxSpeedLB = Lspeed;
  MaxTicsLB = spd;
  digitalWrite(LWenable,LOW);

  // backward right
  digitalWrite(RWfront,BACK);
  Rspeed=0x0000;
  OCR4A=Rspeed;
  digitalWrite(RWenable,HIGH);
  ENC_Reset();
  ENC_Motor(); // LsMtMotor();
  do {
    Rspeed++;
    OCR4A=Rspeed;
    ENC_Motor(); // LsMtMotor();
    delay(50);
    ENC_Motor(); // LsMtMotor();
    delay(50);
    ENC_Motor(); // LsMtMotor();
    spd = ENC_VelocitatRight();
//Serial.print(Lspeed,HEX); Serial.print(" - "); Serial.println(spd,5);
  } while(spd>-1.0);
//Serial.print(Rspeed,HEX); Serial.print(" - "); Serial.println(spd,5);
  MinSpeedRB = Rspeed;
  Rspeed=0x03ff;
  OCR4A=Rspeed;
  ENC_Motor(); // LsMtMotor();
  delay(500);
  ENC_Motor(); // LsMtMotor();
  delay(500);
  ENC_Motor(); // LsMtMotor();
  spd = ENC_VelocitatRight();    
//Serial.print(Rspeed,HEX); Serial.print(" - "); Serial.println(spd,5);
  MaxSpeedRB = Rspeed;
  MaxTicsRB = spd;
  digitalWrite(RWenable,LOW);

  // recalculem els màxims per anar sempre equilibrats
  if (MaxTicsLF!=MaxTicsRF) {     // balancejant forward
    if (MaxTicsLF<MaxTicsRF) {  // rebaixar right forward max
      MaxSpeedRF = (int)((float)MinSpeedRF + (float)MaxTicsLF * (MaxSpeedRF-MinSpeedRF)/(float)(MaxTicsRF-1));
      MaxTicsRF==MaxTicsLF;
    } else {                        // rebaixar left forward max
      MaxSpeedLF = (int)((float)MinSpeedLF + (float)MaxTicsRF * (MaxSpeedLF-MinSpeedLF)/(float)(MaxTicsLF-1));
      MaxTicsLF==MaxTicsRF;
    }
  }
  if (MaxTicsLB!=MaxTicsRB) {     // balancejant backward
    if (MaxTicsLB>MaxTicsRB) {  // rebaixar right backward max
      MaxSpeedRB = (int)((float)MinSpeedRB + (float)MaxTicsLB * (MaxSpeedRB-MinSpeedRB)/(float)(MaxTicsRB-1));
      MaxTicsRB==MaxTicsLB;
    } else {                        // rebaixar left backward max
      MaxSpeedLB = (int)((float)MinSpeedLB + (float)MaxTicsRB * (MaxSpeedLB-MinSpeedLB)/(float)(MaxTicsLB-1));
      MaxTicsLB==MaxTicsRB;
    }
  }
}


