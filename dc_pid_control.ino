#include <Wire.h>
#include <PGMWrap.h>
#include <PID_v1.h>

#include "fatal.h"
#include "as5601.h"
#include "motor.h"

/* use ports usually used for hardware I2C */
#define SDA_PORT PORTB
#define SDA_PIN 3
#define SCL_PORT PORTB
#define SCL_PIN 4

#define I2C_TIMEOUT 100
#define I2C_NOINTERRUPT 0
#define I2C_FASTMODE 1
#define FAC 1
//#define I2C_CPUFREQ (F_CPU/FAC)
#include <SoftWire.h>
SoftWire Wire2 = SoftWire();

/* Serial configuration */
#define BufferSize 256
#define BaudRate 19200

volatile unsigned long ovf_count=0;

AS5601<TwoWire> encoder_AZ(Wire);
AS5601<SoftWire> encoder_EL(Wire2);

/* H-bridge defines */
#define PWM1M1 5
#define PWM2M1 6
#define PWM1M2 9
#define PWM2M2 10
#define M1FB A0
#define M2FB A1
#define nSF 7
/* PIN for Enable or Disable Motors */
#define EN 8
/* Homing switch */
#define HOME_AZ 3
#define HOME_EL 4
/* RS485 enable TX */
#define TX_EN 2

#define MAXSPEED_AZ 140
#define MAXSPEED_EL 160
#define OFFSET_SPEED_AZ 18
#define OFFSET_SPEED_EL 120

#define DEADZONE_AZ 0.2
#define DEADZONE_EL 0.3

motor motor_AZ(PWM1M1, PWM2M1, MAXSPEED_AZ, OFFSET_SPEED_AZ);
motor motor_EL(PWM1M2, PWM2M2, MAXSPEED_EL, OFFSET_SPEED_EL);

#define AZ_KP          5.5
#define AZ_KI          0.0
#define AZ_KD          0.1

#define EL_KP_HOMEING   4
#define EL_KI_HOMEING   0
#define EL_KD_HOMEING   0.1

#define EL_KP          6
#define EL_KI          0
#define EL_KD          0.1

#define EL_KP_LOCAL          2.3
#define EL_KI_LOCAL          1
#define EL_KD_LOCAL          0.2

#define SAMPLE_TIME    10

volatile double setpointAZ, inputAZ, outputAZ;
volatile double setpointEL, inputEL, outputEL;
double AZ_Kp = AZ_KP, AZ_Ki = AZ_KI, AZ_Kd = AZ_KD;
double EL_Kp = EL_KP, EL_Ki = EL_KI, EL_Kd = EL_KD;
bool adaptiveTuning = true;
#define PON P_ON_E

PID pidAZ(&inputAZ, &outputAZ, &setpointAZ, AZ_Kp, AZ_Ki, AZ_Kd, PON,DIRECT);
PID pidEL(&inputEL, &outputEL, &setpointEL, EL_Kp, EL_Ki, EL_Kd, PON,DIRECT);

volatile bool initialized = false;
volatile bool pingEncoders = false;

bool debug = false;

int startTime;

void setup() {
  double limitAZ, limitEL;
  
  Serial.begin(BaudRate);
  Serial.print("1. Init\n");

  /* H-bridge */
  pinMode(OUTPUT, PWM1M1);
  pinMode(OUTPUT, PWM2M1);
  pinMode(OUTPUT, PWM1M2);
  pinMode(OUTPUT, PWM2M2);
  /* Feedback and sense */
  pinMode(M1FB,INPUT);
  pinMode(M2FB,INPUT);
  pinMode(nSF,INPUT);
  /* Enable Motors */
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);
  /* Homing switch */
  pinMode(HOME_AZ, INPUT_PULLUP);
  pinMode(HOME_EL, INPUT_PULLUP);

  // put your setup code here, to run once:
  encoder_AZ.Begin();
  //encoder_AZ.set_zero();
  encoder_AZ.get_pos(&inputAZ);
  setpointAZ = inputAZ;
  outputAZ = 0;
  
  encoder_EL.Begin();
  //encoder_EL.set_zero();
  encoder_EL.get_pos(&inputEL);
  setpointEL = inputEL;
  outputEL = 0;

  Serial.print("2. Init encoders\n");

  setPWMfreq();
  enableTimedInt();

  Serial.print("3. Init timers\n");

  pidAZ.SetSampleTime(SAMPLE_TIME);
  limitAZ = MAXSPEED_AZ-OFFSET_SPEED_AZ;
  pidAZ.SetOutputLimits(-limitAZ,limitAZ);
  pidAZ.SetMode(AUTOMATIC);

  pidEL.SetSampleTime(SAMPLE_TIME);
  limitEL = MAXSPEED_EL-OFFSET_SPEED_EL;
  pidEL.SetOutputLimits(-limitEL,limitEL);
  pidEL.SetMode(AUTOMATIC);

  Serial.print("4. Init PIDs\n");

  startTime = millis();
  
  initialized = true;
  Serial.print("Setup\n");

  Homing(true);
}

void enableTimedInt()
{
  // initialize counter
  TCNT2 = 5; // 250*4e-6 = 1e-3  4e-6=16e6/64
  TIMSK2 |= (1 << TOIE2); //overflow interrupt enable

}

void disableTimedInt()
{
  // initialize counter
  TCNT2 = 5; // 250*4e-6 = 1e-3
  TIMSK2 &= ~(1 << TOIE2); //overflow interrupt disable

}
ISR(TIMER2_OVF_vect)
{
  if (!initialized) return;
  
  ovf_count++;               //Increments the overflow counter
  if (ovf_count < SAMPLE_TIME) return;  // reduce PID frequency
  
  ovf_count = 0;       //Resets the overflow counter

  pidAZ.Compute();
  motor_AZ.move(outputAZ);
  pidEL.Compute();
  motor_EL.move(outputEL);
  pingEncoders = true;
  
}

void setPWMfreq()
{
//---------------------------------------------- Set PWM frequency for D5 & D6 -------------------------------
 
//TCCR0B = TCCR0B & B11111000 | B00000001;    // set timer 0 divisor to     1 for PWM frequency of 62500.00 Hz
//TCCR0B = TCCR0B & B11111000 | B00000010;    // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
//TCCR0B = TCCR0B & B11111000 | B00000011;    // set timer 0 divisor to    64 for PWM frequency of   976.56 Hz (The DEFAULT)
//TCCR0B = TCCR0B & B11111000 | B00000100;    // set timer 0 divisor to   256 for PWM frequency of   244.14 Hz
//TCCR0B = TCCR0B & B11111000 | B00000101;    // set timer 0 divisor to  1024 for PWM frequency of    61.04 Hz
 
 
//---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------
 
TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
//TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
//TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)
//TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz  

//---------------------------------------------- Set PWM frequency for D3 & D11 ------------------------------
 
//TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
//TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
//TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)
//TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
//TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz
 
}

void loop() {
  double posAZ,posEL;
  unsigned char statusAZ, statusEL;
  unsigned short confAZ, confEL, magAZ, magEL, agcAZ, agcEL;
  double *set_point;
  static bool EL_local = false;
  static unsigned int dbgcount = 0;

//  if ((millis() - startTime) >= 1000) {
//    motor_AZ.move(-50);
//    setpointAZ = 10;
//  }

    /* Read commands from serial */
  set_point = cmd_proc();
  setpointAZ = set_point[0];
  setpointEL = set_point[1];

  if (adaptiveTuning && !EL_local && abs(setpointEL - inputEL)<2) {
    pidEL.SetTunings(EL_KP_LOCAL, EL_KI_LOCAL, EL_KD_LOCAL, PON);
    EL_local = true;
    if (debug) Serial.println("local EL tuning");
  } else if (adaptiveTuning && EL_local && abs(setpointEL - inputEL)>=2) {
    if (debug) Serial.println("global EL tuning");
    pidEL.SetTunings(EL_KP, EL_KI, EL_KD, PON);
    EL_local = false;    
  }
  // put your main code here, to run repeatedly:
//  while(semTake(0));
//  statusAZ = encoder_AZ.get_pos(&posAZ);
//  semGive();
//  
//  while(semTake(0));
//  confAZ = encoder_AZ.get_conf();
//  semGive();
//
//  while(semTake(0));
//  magAZ = encoder_AZ.get_magnitude();
//  semGive();
//
//  while(semTake(0));
//  agcAZ = encoder_AZ.get_agc();
//  semGive();
//
//  while(semTake(0));
//  statusEL = encoder_EL.get_pos(&posEL);
//  semGive();
//  
//  while(semTake(0));
//  confEL = encoder_EL.get_conf();
//  semGive();
//
//  while(semTake(0));
//  magEL = encoder_EL.get_magnitude();
//  semGive();
//
//  while(semTake(0));
//  agcEL = encoder_EL.get_agc();
//  semGive();

  if (pingEncoders)
  {
    pingEncoders = false;
    encoder_AZ.get_pos(&inputAZ);
    encoder_EL.get_pos(&inputEL);
  }
  
  //Serial.print("Status:");Serial.print(statusAZ,HEX);Serial.print(" posAZ:");Serial.print(posAZ);Serial.print(" CONF: ");Serial.print(confAZ);Serial.print(" MAG: ");Serial.print(magAZ);Serial.print(" AGC: ");Serial.println(agcAZ);
  //Serial.print("Status:");Serial.print(statusEL,HEX);Serial.print(" posEL:");Serial.print(posEL);Serial.print(" CONF: ");Serial.print(confEL);Serial.print(" MAG: ");Serial.print(magEL);Serial.print(" AGC: ");Serial.println(agcEL);
  if (debug) {
    dbgcount++;
    double myEL=outputEL, myAZ=outputAZ, myinEL = inputEL;
    if ((dbgcount > 1000) && ((myEL>1) || (myAZ>1))) {
    Serial.print(millis());
    Serial.print(" setpointAZ: ");Serial.print(setpointAZ);
    Serial.print(" inputAZ: ");Serial.print(inputAZ);
    Serial.print(" outputAZ:");Serial.print(myAZ);
    
    Serial.print(" setpointEL: ");Serial.print(setpointEL);
    Serial.print(" inputEL: ");Serial.print(myinEL);
    Serial.print(" outputEL:");Serial.println(myEL);
    dbgcount=0;
    }
  }
  
  //Serial.println(ovf_count);
}
