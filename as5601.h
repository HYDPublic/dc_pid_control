#include <Wire.h>

// #define AS5601_DEBUG 

/* Maximum Angle for homing scanning */
#define SEEK_MOVE 20
#define MIN_AZ_ANGLE -180
#define MAX_AZ_ANGLE 370
#define MIN_EL_ANGLE -180
#define MAX_EL_ANGLE 180

/* Encoder defines */
#define as5601_adr 0x36
#define raw_ang_high 0x0c
#define raw_ang_low 0x0d
#define status_reg 0x0b
#define agc 0x1a
#define magnitude_high 0x1b
#define magnitude_low 0x1c
#define conf_high 0x07
#define conf_low 0x08

/* Ratio of encoder gear */
#define RATIO 2



template<class T_WIRE_METHOD> class AS5601
{
public:
    AS5601(T_WIRE_METHOD& wire) :
        _wire(wire)
    {
    }

    void Begin()
    {
        _wire.begin();
        _wire.setClock(800000); // does nothing on SoftWire
    }

//
//void setup_as5601() {
//  /* Encoder */
//  Wire.begin();
//  Wire.setClock(800000);
//  Wire2.begin();
//  i2c_transaction(Wire, 1, 1);
//}

  /* Read Encoder */
  unsigned char get_pos(double *new_pos)
  {
    unsigned short raw_angle;
    unsigned char status_val;
    double raw_pos = 0;
    double delta_raw_pos = 0;
    static double raw_prev_pos = 0;
    static double real_pos;
    static int n = 0;
  
    raw_angle = i2c_word_transaction(as5601_adr,raw_ang_high);
    /* Read Status Bits */
    status_val = i2c_byte_transaction(as5601_adr,status_reg);
    /* Check the status register */
    if ((status_val & 0x20) && !(status_val & 0x10) && !(status_val & 0x08))
    {
      raw_pos = (double)raw_angle * 0.0879;
      delta_raw_pos = raw_prev_pos - raw_pos;
      if (delta_raw_pos > 180)
        n++;
      else if (delta_raw_pos < -180)
        n--;
      real_pos = -((raw_pos + 360 * n) / RATIO) - angle_offset;
      raw_prev_pos = raw_pos;
    }
    else
      fatal(FATAL_AS5601);
      
  #ifdef AS5601_DEBUG    
        Serial.print("raw_angle: ");
        Serial.print(raw_angle);
        Serial.print(" status: ");
        Serial.print(status_val&0x38);
        Serial.print("\n");
  #endif
  
    *new_pos = real_pos;
    return status_val;
  }

  unsigned char get_agc() {
    return i2c_byte_transaction(as5601_adr,agc);
  }
  
  unsigned short get_magnitude() {
    return i2c_word_transaction(as5601_adr,magnitude_high);
  }

  unsigned short get_conf() {
    return i2c_word_transaction(as5601_adr,conf_high);
  }

  void set_zero() {
    double current_pos;
    unsigned char status_val = get_pos(&current_pos);
    Serial.print("set_zero: ");Serial.println(current_pos);
    angle_offset = current_pos;
  }
  
private:
  T_WIRE_METHOD& _wire;    
  double angle_offset = 0;

  unsigned char i2c_byte_transaction(unsigned char i2c_address, unsigned char i2c_register)
  {
    _wire.beginTransmission(i2c_address);
    _wire.write(i2c_register);
    _wire.endTransmission();
    _wire.requestFrom(i2c_address, (uint8_t)1);
    while (_wire.available() == 0);
    return _wire.read();
  }

  unsigned short i2c_word_transaction(unsigned char i2c_address, unsigned char i2c_register)
  {
    unsigned char word_high = i2c_byte_transaction(i2c_address,i2c_register);
    unsigned char word_low = i2c_byte_transaction(i2c_address,i2c_register+1);

    return ((word_high << 8) | word_low);
  }
};



