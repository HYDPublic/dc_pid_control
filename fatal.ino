
extern motor motor_AZ;
extern motor motor_EL;

void fatal(unsigned char msgid) {
  motor_AZ.stop();
  motor_EL.stop();
  while(1)
    Serial.println(fatal_messages[msgid]);
}

