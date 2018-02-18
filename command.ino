//#define DEBUG 

/* EasyComm 2 Protocol */
double * cmd_proc()
{
  static double set_point[] = {0, 0};
  /* Serial */
  static char buffer[BufferSize];
  static int BufferCnt = 0;
  char incomingByte;
  char *Data = buffer;
  char *rawData;
  char data[100];
  double pos[2];

  /* Read from serial */
  while (myRS485Serial.available() > 0)
  {
    incomingByte = myRS485Serial.read();
#ifdef DEBUG
    myRS485Serial.print(incomingByte);
#endif
    /* New data */
    if (incomingByte == '\n' || incomingByte == '\r')
    {
      buffer[BufferCnt] = 0;
#ifdef DEBUG
      myRS485Serial.print("command: ");myRS485Serial.println(buffer);
#endif
      delay(100); // delay before sending just in case
      if (buffer[0] == 'A' && buffer[1] == 'Z')
      {
        if (buffer[2] == ' ' && buffer[3] == 'E' && buffer[4] == 'L')
        {
          /* Get position */
          myRS485Serial.print("AZ");
          myRS485Serial.print(inputAZ, 1);
          myRS485Serial.print(" ");
          myRS485Serial.print("EL");
          myRS485Serial.println(inputEL, 1);
        }
        else
        {
          /* Get the absolute value of angle */
          rawData = strtok_r(Data, " " , &Data);
          strncpy(data, rawData + 2, 10);
          if (isNumber(data))
          {
            set_point[0] = atof(data);
            if (set_point[0] > MAX_AZ_ANGLE)
              set_point[0] = MAX_AZ_ANGLE;
            else if (set_point[0] < MIN_AZ_ANGLE)
              set_point[0] = MIN_AZ_ANGLE;
          }
          /* Get the absolute value of angle */
          rawData = strtok_r(NULL, " " , &Data);
          if (rawData[0] == 'E' && rawData[1] == 'L')
          {
            strncpy(data, rawData + 2, 10);
            if (isNumber(data))
            {
              set_point[1] = atof(data);
              if (set_point[1] > MAX_EL_ANGLE)
                set_point[1] = MAX_EL_ANGLE;
              else if (set_point[1] < MIN_EL_ANGLE)
                set_point[1] = MIN_EL_ANGLE;
            }
          }
          if (debug) {
            myRS485Serial.print(set_point[0]);
            myRS485Serial.print(" ");
            myRS485Serial.print(set_point[1]);
            myRS485Serial.print(" ");
          }
        }
      }
      /* Stop Moving */
      else if (buffer[0] == 'S' && buffer[1] == 'A' && buffer[2] == ' ' && buffer[3] == 'S' && buffer[4] == 'E')
      {
        /* Get position */
        myRS485Serial.print("AZ");
        myRS485Serial.print(inputAZ, 1);
        myRS485Serial.print(" ");
        myRS485Serial.print("EL");
        myRS485Serial.println(inputEL, 1);
        set_point[0] = inputAZ;
        set_point[1] = inputEL;
      }
      /* Reset the rotator */
      else if (buffer[0] == 'R' && buffer[1] == 'E' && buffer[2] == 'S' && buffer[3] == 'E' && buffer[4] == 'T')
      {
        /* Get position */
        myRS485Serial.print("AZ");
        myRS485Serial.print(inputAZ, 1);
        myRS485Serial.print(" ");
        myRS485Serial.print("EL");
        myRS485Serial.println(inputEL, 1);
        /* Move to initial position */
        Homing(false);
        set_point[0] = 0;
        set_point[1] = 0;
      }
      else if (buffer[0] == 'K' && buffer[1] == 'A') {
        /* Get the absolute value of angle */
        rawData = strtok_r(Data, " " , &Data);
        strncpy(data, rawData + 2, 10);
        if (isNumber(data))
        {
            AZ_Kp = atof(data);
        }
        /* Get the absolute value of angle */
        rawData = strtok_r(NULL, " " , &Data);
        if (rawData[0] == 'I' && rawData[1] == 'A')
        {
          strncpy(data, rawData + 2, 10);
          if (isNumber(data))
          {
            AZ_Ki = atof(data);
          }
        }
        /* Get the absolute value of angle */
        rawData = strtok_r(NULL, " " , &Data);
        if (rawData[0] == 'D' && rawData[1] == 'A')
        {
          strncpy(data, rawData + 2, 10);
          if (isNumber(data))
          {
            AZ_Kd = atof(data);
          }
        }
        myRS485Serial.print("AZ_Kp: ");myRS485Serial.print(AZ_Kp);
        myRS485Serial.print(" AZ_Ki: ");myRS485Serial.print(AZ_Ki);
        myRS485Serial.print(" AZ_Kd: ");myRS485Serial.println(AZ_Kd);
        pidAZ.SetTunings(AZ_Kp, AZ_Ki, AZ_Kd);
      }
      else if (buffer[0] == 'K' && buffer[1] == 'E') {
        /* Get the absolute value of angle */
        rawData = strtok_r(Data, " " , &Data);
        strncpy(data, rawData + 2, 10);
        if (isNumber(data))
        {
            EL_Kp = atof(data);
        }
        /* Get the absolute value of angle */
        rawData = strtok_r(NULL, " " , &Data);
        if (rawData[0] == 'I' && rawData[1] == 'E')
        {
          strncpy(data, rawData + 2, 10);
          if (isNumber(data))
          {
            EL_Ki = atof(data);
          }
        }
        /* Get the absolute value of angle */
        rawData = strtok_r(NULL, " " , &Data);
        if (rawData[0] == 'D' && rawData[1] == 'E')
        {
          strncpy(data, rawData + 2, 10);
          if (isNumber(data))
          {
            EL_Kd = atof(data);
          }
        }
        myRS485Serial.print("EL_Kp: ");myRS485Serial.print(EL_Kp);
        myRS485Serial.print(" EL_Ki: ");myRS485Serial.print(EL_Ki);
        myRS485Serial.print(" EL_Kd: ");myRS485Serial.println(EL_Kd);
        pidEL.SetTunings(EL_Kp, EL_Ki, EL_Kd);
      }
      else if (buffer[0] == 'D' && buffer[1] == 'B')
      {
        debug = !debug;
      }
      else if (buffer[0] == 'A' && buffer[1] == 'T')
      {
        adaptiveTuning = !adaptiveTuning;
        myRS485Serial.print("Adaptive is turned ");myRS485Serial.println((adaptiveTuning)?"on":"off");
      }
      else if (buffer[0] == 'S' && buffer[1] == 'T')
      {
        myRS485Serial.print("Status\n");
        myRS485Serial.print("AZ_Kp: ");myRS485Serial.print(pidAZ.GetKp());
        myRS485Serial.print(" AZ_Ki: ");myRS485Serial.print(pidAZ.GetKi());
        myRS485Serial.print(" AZ_Kd: ");myRS485Serial.println(pidAZ.GetKd());
        myRS485Serial.print("EL_Kp: ");myRS485Serial.print(pidEL.GetKp());
        myRS485Serial.print(" EL_Ki: ");myRS485Serial.print(pidEL.GetKi());
        myRS485Serial.print(" EL_Kd: ");myRS485Serial.println(pidEL.GetKd());
        myRS485Serial.print(millis());
        myRS485Serial.print(" setpointAZ: ");myRS485Serial.print(setpointAZ);
        myRS485Serial.print(" inputAZ: ");myRS485Serial.print(inputAZ);
        myRS485Serial.print(" outputAZ:");myRS485Serial.print(outputAZ);

        myRS485Serial.print(" setpointEL: ");myRS485Serial.print(setpointEL);
        myRS485Serial.print(" inputEL: ");myRS485Serial.print(inputEL);
        myRS485Serial.print(" outputEL:");myRS485Serial.println(outputEL);
        myRS485Serial.print("adaptive: ");myRS485Serial.println((adaptiveTuning?"on":"off"));
      }
      else if (buffer[0] == 'F' && buffer[1] == 'A')
      {
        fatal(FATAL_USER);
      }

      BufferCnt = 0;
      /* Reset the disable motor time */
      //t_DIS = 0;
    }
    /* Fill the buffer with incoming data */
    else {
      buffer[BufferCnt] = incomingByte;
      BufferCnt++;
      if (BufferCnt>=BufferSize) BufferCnt=0;
    }
  }
  return set_point;
}


/* check if is argument in number */
boolean isNumber(char *input)
{
  for (int i = 0; input[i] != '\0'; i++)
  {
    if (isalpha(input[i]))
      return false;
  }
  return true;
}

