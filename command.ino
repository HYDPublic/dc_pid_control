#define DEBUG true

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
  while (Serial.available() > 0)
  {
    incomingByte = Serial.read();
#ifdef DEBUG
    Serial.print(incomingByte);
#endif
    /* New data */
    if (incomingByte == '\n' || incomingByte == '\r')
    {
      buffer[BufferCnt] = 0;
#ifdef DEBUG
      Serial.print("command: ");Serial.println(buffer);
#endif
      delay(100); // delay before sending just in case
      if (buffer[0] == 'A' && buffer[1] == 'Z')
      {
        if (buffer[2] == ' ' && buffer[3] == 'E' && buffer[4] == 'L')
        {
          /* Get position */
          digitalWrite(TX_EN, HIGH);
          delay(5);
          Serial.print("AZ");
          Serial.print(inputAZ, 1);
          Serial.print(" ");
          Serial.print("EL");
          Serial.println(inputEL, 1);
          delay(5);
          digitalWrite(TX_EN, LOW);
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
          digitalWrite(TX_EN, HIGH);
          delay(5);
          Serial.print(set_point[0]);
          Serial.print(" ");
          Serial.print(set_point[1]);
          Serial.print(" ");
          delay(5);
          digitalWrite(TX_EN, LOW);
        }
      }
      /* Stop Moving */
      else if (buffer[0] == 'S' && buffer[1] == 'A' && buffer[2] == ' ' && buffer[3] == 'S' && buffer[4] == 'E')
      {
        /* Get position */
        digitalWrite(TX_EN, HIGH);
        delay(5);
        Serial.print("AZ");
        Serial.print(inputAZ, 1);
        Serial.print(" ");
        Serial.print("EL");
        Serial.println(inputEL, 1);
        delay(5);
        digitalWrite(TX_EN, LOW);
        set_point[0] = inputAZ;
        set_point[1] = inputEL;
      }
      /* Reset the rotator */
      else if (buffer[0] == 'R' && buffer[1] == 'E' && buffer[2] == 'S' && buffer[3] == 'E' && buffer[4] == 'T')
      {
        /* Get position */
        digitalWrite(TX_EN, HIGH);
        delay(5);
        Serial.print("AZ");
        Serial.print(inputAZ, 1);
        Serial.print(" ");
        Serial.print("EL");
        Serial.println(inputEL, 1);
        delay(5);
        digitalWrite(TX_EN, LOW);
        /* Move to initial position */
        //Homing(-MAX_AZ_ANGLE, -MAX_EL_ANGLE, false);
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
        Serial.print("AZ_Kp: ");Serial.print(AZ_Kp);
        Serial.print(" AZ_Ki: ");Serial.print(AZ_Ki);
        Serial.print(" AZ_Kd: ");Serial.println(AZ_Kd);
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
        Serial.print("EL_Kp: ");Serial.print(EL_Kp);
        Serial.print(" EL_Ki: ");Serial.print(EL_Ki);
        Serial.print(" EL_Kd: ");Serial.println(EL_Kd);
        pidEL.SetTunings(EL_Kp, EL_Ki, EL_Kd);
      }
      else if (buffer[0] == 'D' && buffer[1] == 'E')
      {
        debug = !debug;
      }
      else if (buffer[0] == 'S' && buffer[1] == 'T')
      {
        Serial.print("Status\n");
        Serial.print("AZ_Kp: ");Serial.print(AZ_Kp);
        Serial.print(" AZ_Ki: ");Serial.print(AZ_Ki);
        Serial.print(" AZ_Kd: ");Serial.println(AZ_Kd);
        Serial.print("EL_Kp: ");Serial.print(EL_Kp);
        Serial.print(" EL_Ki: ");Serial.print(EL_Ki);
        Serial.print(" EL_Kd: ");Serial.println(EL_Kd);
        stat1();
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
  delay(5);
  digitalWrite(TX_EN, LOW );
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

