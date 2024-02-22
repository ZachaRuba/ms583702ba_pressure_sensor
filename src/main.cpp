#include <Arduino.h>
#include <Wire.h>
#include "ms583702ba.h"
//#include <SoftWire.h>

//#define SDA 18 // A4
//#define SCL 19 // A5

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
  Serial.begin(9600);
  pt_init();
  Serial.println((uint16_t)pt_read_prom(0),HEX);
  Serial.println((uint16_t)pt_read_prom(1),HEX);
  Serial.println((uint16_t)pt_read_prom(2),HEX);
  Serial.println((uint16_t)pt_read_prom(3),HEX);
  Serial.println((uint16_t)pt_read_prom(4),HEX);
  Serial.println((uint16_t)pt_read_prom(5),HEX);
  Serial.println((uint16_t)pt_read_prom(6),HEX);
  Serial.println((uint32_t)pt_read_digital_data(D1_CONVERSION),HEX);
  Serial.println((uint32_t)pt_read_digital_data(D2_CONVERSION),HEX);
}

void loop() {
  // put your main code here, to run repeatedly:

}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}