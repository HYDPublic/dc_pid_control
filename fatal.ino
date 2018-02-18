
extern motor motor_AZ;
extern motor motor_EL;

void fatal(unsigned char msgid) {
  pidAZ.SetMode(MANUAL);
  pidEL.SetMode(MANUAL);
  motor_AZ.stop();
  motor_EL.stop();
  disableTimedInt();
  while(1) {
    myRS485Serial.println(fatal_messages[msgid]);
    delay(1000);
  }  
}

