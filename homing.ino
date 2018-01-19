/* Change to LOW according to Home sensor */
#define DEFAULT_HOME_STATE LOW

/* Maximum Angle for homing scanning */
#define SEEK_MOVE 10
#define SEEK_AZ_ANGLE -360
#define SEEK_EL_ANGLE -180

/* Homing switch */
#define HOME_AZ 3
#define HOME_EL 4

#define DEBUG true

/* Homing Function */
void Homing(bool Init)
{
  int value_Home_AZ = DEFAULT_HOME_STATE;
  int value_Home_EL = DEFAULT_HOME_STATE;
  boolean isHome_AZ = false;
  boolean isHome_EL = false;
  boolean seek_AZ = false;
  boolean seek_EL = false;
  double startAZ, startEL;
  double curr_angle[2];
  double set_point[2];

  pidEL.SetTunings(EL_KP_HOMEING, EL_KI_HOMEING, EL_KD_HOMEING, PON);

  encoder_AZ.get_pos(&startAZ);
  encoder_EL.get_pos(&startEL);
      
  if (Init == true)
  {
    set_point[0] = startAZ + SEEK_MOVE;
    set_point[1] = startEL + SEEK_MOVE;
  }
  else
  {
    set_point[0] = SEEK_MOVE;
    set_point[1] = SEEK_MOVE;
  }
  
  setpointAZ = set_point[0];
  setpointEL = set_point[1];

  while(!seek_AZ || !seek_EL) {
    encoder_AZ.get_pos(&inputAZ);
    encoder_EL.get_pos(&inputEL);
    Serial.print("current AZ: "); Serial.print(setpointAZ); Serial.print("/"); Serial.print(inputAZ);Serial.print(" outputAZ:");Serial.print(outputAZ);
    Serial.print(" current EL: "); Serial.print(setpointEL); Serial.print("/"); Serial.print(inputEL);Serial.print(" outputEL:");Serial.println(outputEL);
    if ((inputAZ - startAZ) >= (abs(SEEK_MOVE)-DEADZONE_AZ) && !isHome_AZ && Init) {
      seek_AZ = true;
    }
    if ((inputEL - startEL) >= (abs(SEEK_MOVE)-DEADZONE_EL) && !isHome_EL && Init) {
      seek_EL = true;
    }
  }

  setpointAZ = startAZ+SEEK_AZ_ANGLE;
  setpointEL = startEL+SEEK_EL_ANGLE;

  while (isHome_AZ == false || isHome_EL == false)
  {
    encoder_AZ.get_pos(&inputAZ);
    value_Home_AZ = digitalRead(HOME_AZ);
    //Serial.print("switches: "); Serial.print(value_Home_AZ);Serial.print(value_Home_EL);
    /* Change to LOW according to Home sensor */
    if (value_Home_AZ == DEFAULT_HOME_STATE && isHome_AZ == false)
    {
      isHome_AZ = true;
      pidAZ.SetMode(MANUAL);
      outputAZ=0;
      motor_AZ.stop();
      if (Init)
        encoder_AZ.set_zero();
      encoder_AZ.get_pos(&inputAZ);
      setpointAZ = 0;
      pidAZ.SetTunings(AZ_Kp, AZ_Ki, AZ_Kd, PON);
      pidAZ.SetMode(AUTOMATIC);
      Serial.print("homing AZ: "); Serial.print(setpointAZ); Serial.print("/"); Serial.print(inputAZ);Serial.print(" outputAZ:");Serial.println(outputAZ);
    }
    encoder_EL.get_pos(&inputEL);
    value_Home_EL = digitalRead(HOME_EL);
    if (value_Home_EL == DEFAULT_HOME_STATE && isHome_EL == false)
    {
      isHome_EL = true;
      pidEL.SetMode(MANUAL);      
      outputEL = 0;
      motor_EL.stop();
      if (Init)
        encoder_EL.set_zero();;
      encoder_EL.get_pos(&inputEL);
      setpointEL = 0;
      pidEL.SetTunings(EL_Kp, EL_Ki, EL_Kd, PON);
      pidEL.SetMode(AUTOMATIC);
      Serial.print("homing EL: "); Serial.print(setpointEL); Serial.print("/"); Serial.print(inputEL);Serial.print(" outputEL:");Serial.println(outputEL);
    }
    encoder_AZ.get_pos(&inputAZ);
    encoder_EL.get_pos(&inputEL);
    if (debug) {
      Serial.print("current AZ: "); Serial.print(setpointAZ); Serial.print("/"); Serial.print(inputAZ);Serial.print(" outputAZ:");Serial.print(outputAZ);
      Serial.print(" current EL: "); Serial.print(setpointEL); Serial.print("/"); Serial.print(inputEL);Serial.print(" outputEL:");Serial.println(outputEL);
//    Serial.print(getM1CurrentMilliamps());Serial.print(" ");
//    Serial.print(getM2CurrentMilliamps());Serial.print("\n");
    }      
    if ((abs(inputAZ - startAZ) >= abs(SEEK_AZ_ANGLE-DEADZONE_AZ) && !isHome_AZ) || (abs(inputEL - startEL) >= abs(SEEK_EL_ANGLE-DEADZONE_EL) && !isHome_EL))
    {
      fatal(FATAL_HOMING);
    }
  }
}
