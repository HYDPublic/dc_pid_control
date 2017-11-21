
#define DEADBAND 0.2

void moveTo(double AZ, double EL, int duration)
{
  long now;
  
  setpointAZ = AZ;
  setpointEL = EL;
  now = millis();
  while(millis()-now < duration) {
    //stat1();
    if (pingEncoders)
    {
      pingEncoders = false;
      encoder_AZ.get_pos(&inputAZ);
      encoder_EL.get_pos(&inputEL);
    }
  }
}

unsigned int test_step(double degrees, bool up) {
  double startEL, curEL, savedOutputEL;
  moveTo(0,degrees,10000);
//  setpointEL = 10;
//  while(abs(setpointEL - inputEL)>DEADBAND) {
//    stat1();
//    if (pingEncoders)
//    {
//      pingEncoders = false;
//      encoder_AZ.get_pos(&inputAZ);
//      encoder_EL.get_pos(&inputEL);
//    }
//  }
//  delay(2000);
  Serial.print("Reached setpoint");
  stat1();
  Serial.println("Manual control");

  pidEL.SetMode(MANUAL);
  motor_EL.set_min(0);
  delay(1000);
  encoder_EL.get_pos(&startEL);
  Serial.print("Start position: ");Serial.println(startEL);
  if (up) {
    for(int i=0;i<160; i++) {
      outputEL = i;
      delay(50);
      encoder_EL.get_pos(&curEL);
      if (abs(curEL-startEL) > DEADBAND) break;
    }
  } else {
    for(int i=0;i>-160; i--) {
      outputEL = i;
      delay(50);
      encoder_EL.get_pos(&curEL);
      if (abs(curEL-startEL) > DEADBAND) break;   
    } 
  }
  Serial.print("curEL: ");Serial.println(curEL);
  savedOutputEL = outputEL;
  outputEL = 0;
  inputEL = curEL;
  setpointEL = degrees;
  motor_EL.set_min(115);  
  Serial.print("threshold: ");Serial.println(savedOutputEL);
  pidEL.SetMode(AUTOMATIC);

  return(savedOutputEL);
}

void Testing()
{
  double startEL, curEL, savedOutputEL;
  Serial.print("Testing...");
  moveTo(0,0,10000);

  for(int i=10;i<=90;i+=10) {
    savedOutputEL = test_step(i, true);
  }
  for(int i=90;i>10;i-=10) {
    savedOutputEL = test_step(i, false);
  }
  motor_EL.set_min(savedOutputEL);  
  
  outputEL=0;
  pidEL.SetMode(AUTOMATIC);
}

void stat1() {
    Serial.print(millis());
    Serial.print(" setpointAZ: ");Serial.print(setpointAZ);
    Serial.print(" inputAZ: ");Serial.print(inputAZ);
    Serial.print(" outputAZ:");Serial.print(outputAZ);
    
    Serial.print(" setpointEL: ");Serial.print(setpointEL);
    Serial.print(" inputEL: ");Serial.print(inputEL);
    Serial.print(" outputEL:");Serial.println(outputEL);
}

