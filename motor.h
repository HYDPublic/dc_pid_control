

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
    dbgcount++;
    if ((dbgcount>1000) && (speed!=0)) {
      dbgcount=0;
      Serial.print("speed: ");Serial.print(speed);
    }
    
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
      //Serial.print("rechts pwm1 ");Serial.print(_pwm_pin1);Serial.print("pwm2 ");Serial.print(_pwm_pin2);Serial.print("speed: ");Serial.println(speed);
    }
    else
    {
      speed = -speed;
      speed = speed + _minSpeed;
      if ( speed > _maxSpeed)
        speed = _maxSpeed;
      analogWrite(_pwm_pin1, speed);
      analogWrite(_pwm_pin2, 0);
      //Serial.print("links pwm1 ");Serial.print(_pwm_pin1);Serial.print("pwm2 ");Serial.print(_pwm_pin2);Serial.print("speed: ");Serial.println(speed);
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
