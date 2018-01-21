

class motor {

public:
  motor(unsigned char pwm_pin1, unsigned char pwm_pin2, unsigned short maxSpeed, unsigned short minSpeed) {
    _pwm_pin1 = pwm_pin1;
    _pwm_pin2 = pwm_pin2;
    _maxSpeed = maxSpeed;
    _minSpeed = minSpeed;
    stop();
  }
  
  void move(short speed) {
//    if (abs(speed)<_minSpeed) {
//      if (abs(speed)>0) speed=sign(speed)*_minSpeed;
//    }
//    dbgcount++;
//    if ((dbgcount>1000) && (speed!=0)) {
//      dbgcount=0;
//      myRS485Serial.print("speed: ");myRS485Serial.print(speed);
//    }
//    
    if (speed == 0) {
      stop();
      return;
    }
    
    if (speed >= 0)
    {
      speed = speed + _minSpeed;
      if (speed > _maxSpeed)
        speed = _maxSpeed;
      analogWrite(_pwm_pin1, 0);
      analogWrite(_pwm_pin2, speed);
      //myRS485Serial.print("rechts pwm1 ");myRS485Serial.print(_pwm_pin1);myRS485Serial.print("pwm2 ");myRS485Serial.print(_pwm_pin2);myRS485Serial.print("speed: ");myRS485Serial.println(speed);
    }
    else
    {
      speed = -speed;
      speed = speed + _minSpeed;
      if ( speed > _maxSpeed)
        speed = _maxSpeed;
      analogWrite(_pwm_pin1, speed);
      analogWrite(_pwm_pin2, 0);
      //myRS485Serial.print("links pwm1 ");myRS485Serial.print(_pwm_pin1);myRS485Serial.print("pwm2 ");myRS485Serial.print(_pwm_pin2);myRS485Serial.print("speed: ");myRS485Serial.println(speed);
    }
    stopped = false;
  }

  void stop()
  {
    if (!stopped) {
      analogWrite(_pwm_pin1, 0);
      analogWrite(_pwm_pin2, 0);
      stopped = true;
    }
  }

  void set_max(unsigned short max) {
    _maxSpeed = max;
  }
  
private:
  unsigned char _pwm_pin1, _pwm_pin2;
  unsigned short _maxSpeed, _minSpeed;
  unsigned short dbgcount = 0;
  boolean stopped = true;
  
  sign(int x) {
    return (x > 0) - (x < 0);
  }
};
