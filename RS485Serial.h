#ifndef RS485_SERIAL
#define RS485_SERIAL

#include <stdarg.h>
#include "Arduino.h"

class RS485Serial {
public:

private:
    HardwareSerial*  _printer;
    unsigned char _tx_enable_pin = 0;
    unsigned short _rs485_delay = 0; 
    
public:
    /*! * default Constructor */
    RS485Serial()
      : _printer(nullptr) {}

    void setTXpin(unsigned char tx_enable_pin) {
      _tx_enable_pin = tx_enable_pin;
    }

    void setDelay(unsigned short rs485_delay) {
      _rs485_delay = rs485_delay;
    }
    
    void setPrinter(Print &printer) {
      _printer = &printer;
    }    
    
    template <typename Type>
    RS485Serial& print(Type tX) {
      digitalWrite(_tx_enable_pin, HIGH);
      _printer->print(tX);
      delay(_rs485_delay);
      digitalWrite(_tx_enable_pin, LOW);
      return *this;
    }

    template <typename Type>
    RS485Serial& p(Type tX) {
      return print(tX);
    }

    template <typename Type>
    RS485Serial& println(Type tX) {
      digitalWrite(_tx_enable_pin, HIGH);
      _printer->println(tX);
      delay(_rs485_delay);
      digitalWrite(_tx_enable_pin, LOW);
      return *this;
    }

    template <typename Type>
    RS485Serial& pln(Type tX) {
      return println(tX);
    }

    RS485Serial& print(long n, int base) {
      _printer->print(n, base);
      digitalWrite(_tx_enable_pin, HIGH);
      delay(_rs485_delay);
      digitalWrite(_tx_enable_pin, LOW);
      return *this;
    }
    inline RS485Serial& p(long n, int base) { return print(n, base); }

    RS485Serial& println(long n, int base) {
      digitalWrite(_tx_enable_pin, HIGH);
      _printer->println(n, base);
      delay(_rs485_delay);
      digitalWrite(_tx_enable_pin, LOW);
      return *this;
    }
    inline RS485Serial& pln(long n, int base) { return println(n, base); }

    RS485Serial& println(void) {
      digitalWrite(_tx_enable_pin, HIGH);
      _printer->println();
      delay(_rs485_delay);
      digitalWrite(_tx_enable_pin, LOW);
      return *this;
    }
    inline RS485Serial& pln(void) { return println(); }

    int available(void) {
      return _printer->available();
    }

    int read(void) {
      digitalWrite(_tx_enable_pin, LOW);
      return _printer->read();
    }
   
};
#endif

