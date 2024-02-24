#include <Arduino.h>
#include <Wire.h>
#include "ms583702ba.h"
//#include <SoftWire.h>

//#define SDA 18 // A4
//#define SCL 19 // A5

void setup() {
  // put your setup code here, to run once:
  float P; float T;
  int ret;
  Serial.begin(9600);
  pt_init();
  ret = pt_read_pressure_and_temperature(&P,&T);
  if( ret > 0){
    Serial.println(P);
    Serial.println(T);
  }
  else{
    Serial.println(ret);
    Serial.println(g_u32PTErrors, HEX);
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}
