//********************************************************************
//
// ms583702ba.cpp - Pressure Transducer Module
//
// Author: Zachary Anderson
// Date: Feb 08, 2024
//
// Note:
//  ----
// see docs/ENG_DS_MS5837-02BA01_A8.pdf (pg 7)
//
//********************************************************************

// INCLUDES & DEFINES
//
#include <Arduino.h>
#include <Wire.h>
#include <assert.h>
#include "ms583702ba.h"
#include "io_helper.h"

// Factory Calibration Factors
uint16_t g_u16C1; // Pressure Sensitivity (SENS)
uint16_t g_u16C2; // Pressure Offset (OFF)
uint16_t g_u16C3; // Temperature Coefficient of Pressure Sensitivity (TCS)
uint16_t g_u16C4; // Temperature Coefficient of Pressure Offset (TCO)
uint16_t g_u16C5; // Reference Temperature (Tref)
uint16_t g_u16C6; // Temperature Coefficient of the Temperature (TEMPSENS)

// RAW Pressure and Temperature Values
uint32_t g_u32D1; // Digital Pressure Value
uint32_t g_u32D2; // Digital Temperature Value

void pt_init(){
    Wire.begin();
    pt_reset();
    pt_read_calibration_data_from_prom();
}

void pt_reset(){
    // write Reset Sequence
    Wire.beginTransmission(ADDRESS);
    Wire.write(RESET);
    Wire.endTransmission();
}

int pt_read_prom(uint8_t addr){

    // pg 13 - memory mapping
    // address 0 - crc [15:12] | Product Type [11:5] | factory settings [4:0]
    // address 1-6 - C1-C6
    uint16_t upper_bytes = 0;
    uint16_t lower_bytes = 0;
    int reg = -1;

    assert(addr < 7);

    // write i2c cmd for reading a prom memory
    Wire.beginTransmission(ADDRESS);
    Wire.write(READ_PROM | (addr << 1));
    Wire.endTransmission();

    // read i2c cmd to get data from memory (16bits) (ie 2 bytes)
    Wire.requestFrom(ADDRESS,2);
    if(Wire.available() == 2) {
        upper_bytes = Wire.read();
        lower_bytes = Wire.read();
        reg = 0;
        reg = lower_bytes | (upper_bytes << 8);
    }
    return reg;
}

int32_t pt_read_digital_data(COMMAND cmd){
    uint16_t upper_bytes = 0;
    uint16_t middle_bytes = 0;
    uint16_t lower_bytes = 0;
    int32_t ret = -1;

    if (cmd != D1_CONVERSION && cmd != D2_CONVERSION)
        return -2;

    // write i2c cmd to do conversion
    Wire.beginTransmission(ADDRESS);
    Wire.write(cmd);
    Wire.endTransmission();

    delay(1000);

    // write i2c cmd to read adc
    Wire.beginTransmission(ADDRESS);
    Wire.write(READ_ADC);
    Wire.endTransmission();

    // read i2c cmd to get data from memory (24bits)
    Wire.requestFrom(ADDRESS, 3);
    if(Wire.available() == 3){
        upper_bytes = Wire.read();
        middle_bytes = Wire.read();
        lower_bytes = Wire.read();
        ret = (uint32_t)upper_bytes << 16 | middle_bytes << 8 | lower_bytes;
    }

    return ret;
}

void pt_read_calibration_data_from_prom(){
    g_u16C1 = pt_read_prom(1);
    g_u16C2 = pt_read_prom(2);
    g_u16C3 = pt_read_prom(3);
    g_u16C4 = pt_read_prom(4);
    g_u16C5 = pt_read_prom(5);
    g_u16C6 = pt_read_prom(6);
}

void pt_read_digital_pressure_and_temperature_data(
    uint32_t * pu32D1,
    uint32_t * pu32D2
){
    uint32_t u32D1; // Digital Pressure Value
    uint32_t u32D2; // Digital Temperature Value

    u32D1 = pt_read_digital_data(D1_CONVERSION);
    u32D2 = pt_read_digital_data(D2_CONVERSION);

    // Check Values are ok.
    assert(u32D1 <= MAX_DIGITAL_READ_VALUE);
    assert(u32D2 <= MAX_DIGITAL_READ_VALUE);

    // return u32D1 u32D2
    *pu32D1 = (uint32_t) u32D1;
    *pu32D2 = (uint32_t) u32D2;
}

void pt_calculate_temperature(
    uint32_t u32D2, 
    int32_t * ps32dT, 
    int32_t * ps32TEMP
){
    int32_t s32dT; // Difference between actual and reference temperature
    int32_t s32TEMP; // Actual Temperature (-40 to 85degC with 0.01degC resolution)

    // Calculate Values
    s32dT = u32D2 - g_u16C5 * (1UL << 8); //256L; // 1 << 8 is equivalent to Pow(2,8)

    Serial.println(u32D2);
    Serial.println(g_u16C5);
    Serial.println(s32dT);
    Serial.flush();

    s32TEMP = DEG_C_20 + s32dT * g_u16C6 / (1UL << 23); //8388608L;  // 1 << 23 is equivalent to Pow(2,23)

    Serial.println(DEG_C_20);
    Serial.println(g_u16C6);
    Serial.println(s32TEMP);
    Serial.flush();

    // Check Values are ok.
    assert((MIN_DT <= s32dT) && (s32dT <= MAX_DT));
    assert((MIN_TEMP_VALUE <= s32TEMP) && (s32TEMP <= MAX_TEMP_VALUE));

    // return s32dT s32TEMP
    *ps32dT = s32dT;
    *ps32TEMP = s32TEMP;
}

void pt_calculate_temperature_compensated_pressure(
    uint32_t u32D1, 
    int32_t s32dT,
    int64_t * ps64OFF,
    int64_t * ps64SENS,
    int32_t * ps32P
){
    int64_t s64OFF; // Offset at actual temperature
    int64_t s64SENS; // Sensitivity at actual temperature
    int32_t s32P; // Temperature Compensated Pressure (10-1200mbar with 0.01mbar resolution)

    // Calculate Values
    s64OFF = (g_u16C2) * (1ULL << 17) + (g_u16C4 * s32dT) / (1U << 6);

    Serial.println(g_u16C2);
    Serial.println((1UL << 17));
    Serial.println(g_u16C4);
    Serial.println(s32dT);
    Serial.println((1U << 6));
    bigPrint(s64OFF);
    Serial.flush();

    s64SENS = g_u16C1 * (1ULL << 16) + (g_u16C3 * s32dT) / (1U << 7);

    Serial.println(g_u16C1);
    Serial.println((1UL << 16));
    Serial.println(g_u16C3);
    Serial.println(s32dT);
    Serial.println((1U << 7));
    bigPrint(s64SENS);
    Serial.flush();

    s32P = (u32D1 * s64SENS / (1ULL << 21) - s64OFF) / (1U << 15);
    
    Serial.println(u32D1);
    Serial.println((1UL<<21));
    Serial.println((1U << 15));
    Serial.println(s32P);
    Serial.flush();

    bigPrint(MAX_OFFSET);
    bigPrint(MIN_OFFSET);
    bigPrint(MAX_SENS);
    bigPrint(MIN_SENS);
    Serial.println(MAX_P);
    Serial.println(MIN_P);
    bigPrint(-1000LL);
    Serial.flush();

    Serial.println((MIN_OFFSET <= s64OFF));    
    Serial.println((s64OFF <= MAX_OFFSET));
    Serial.println((MIN_SENS <= s64SENS));
    Serial.println((s64SENS <= MAX_SENS));
    Serial.println(MIN_P <= s32P);
    Serial.println((s32P <= MAX_P));

    // Check Values are ok.
    assert((MIN_OFFSET <= s64OFF) && (s64OFF <= MAX_OFFSET));
    assert((MIN_SENS <= s64SENS) && (s64SENS<= MAX_SENS));
    assert((MIN_P <= s32P) && (s32P <= MAX_P));

    // return s64OFF, s64SENS, s32P
    *ps64OFF = s64OFF;
    *ps64SENS = s64SENS;
    *ps32P = s32P;
}

void pt_calculate_pressure_and_temperature_second_order(
    uint32_t u32D1, 
    int32_t s32dT, 
    int32_t s32TEMP, 
    int64_t s64OFF, 
    int64_t s64SENS, 
    int32_t s32P,
    float * pfTEMP2,
    float * pfP2
){
    int64_t s64OFFi = 0;
    int64_t s64SENSi = 0;
    int32_t s32Ti = 0;
    int64_t s64OFF2;
    int64_t s64SENS2;
    float fTEMP2;
    float fP2;

    // Low Temp Condition
    if(s32TEMP < DEG_C_20) {
        s32Ti = 11ULL * (s32dT * s32dT) / (1ULL << 35);
        s64OFFi = 31ULL * ((s32TEMP - DEG_C_20) * (s32TEMP - DEG_C_20)) / (1U << 3);
        s64SENSi = 63ULL * ((s32TEMP - DEG_C_20) * (s32TEMP - DEG_C_20)) / (1U << 5); 
    }

    // Calculation
    s64OFF2 = s64OFF - s64OFFi;
    s64SENS2 = s64SENS - s64SENSi;
    fTEMP2 = (s32TEMP - s32Ti) / 100.0f; // DegC
    fP2 = (((u32D1 * s64SENS2) / (1UL << 21) - s64OFF2) / (1U << 15)) / 100.0f; // mbar

    // return fTEMP2, fP2
    *pfTEMP2 = fTEMP2;
    *pfP2 = fP2;
}

void read_temp_and_pressure(){
    uint32_t u32D1;
    uint32_t u32D2;
    int32_t s32dT;
    int32_t s32TEMP;
    int64_t s64OFF;
    int64_t s64SENS;
    int32_t s32P;
    float fTEMP2;
    float fP2;

    Serial.println("Reading Digital Pressure and Temperature...");
    pt_read_digital_pressure_and_temperature_data(&u32D1,&u32D2);
    Serial.println(u32D1,HEX);
    Serial.println(u32D2,HEX);

    Serial.println("Calculating Temperature...");
    Serial.flush();
    pt_calculate_temperature(u32D2, &s32dT, &s32TEMP);
    Serial.println(s32dT,HEX);
    Serial.println(s32TEMP);

    Serial.println("Getting First Order Pressure...");
    Serial.flush();
    pt_calculate_temperature_compensated_pressure(u32D1, s32dT, &s64OFF, &s64SENS, &s32P);
    Serial.println(((long) s64OFF));
    Serial.println(((long) s64SENS));
    Serial.println(s32P);

    Serial.println("Getting Second Order Pressure...");
    Serial.flush();
    pt_calculate_pressure_and_temperature_second_order(u32D1,s32dT,s32TEMP,s64OFF,s64SENS,s32P,&fTEMP2,&fP2);
    Serial.println(fP2);
    Serial.println(fTEMP2);
    Serial.flush();
}