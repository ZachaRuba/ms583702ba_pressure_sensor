//********************************************************************
//
// ms583702ba.h - Pressure Transducer Module Header
//
// Author: Zachary Anderson
// Date: Feb 08, 2024
//
// Note:
//  ----
//
//********************************************************************

#include <Arduino.h> // Just for types

// Temperature at 20 Degree
#define DEG_C_20 2000

// For Error Checking
#define MAX_CALIBRATION_VALUE 65535
#define MAX_DIGITAL_READ_VALUE 16777216
#define MAX_DT 16777216LL
#define MIN_DT -16776960LL
#define MAX_TEMP_VALUE 8500
#define MIN_TEMP_VALUE -4000
#define MAX_OFFSET 25769410560LL
#define MIN_OFFSET -17179344900LL
#define MAX_SENS 12884705280LL
#define MIN_SENS -8589672450LL
#define MAX_P 120000L // Pressure
#define MIN_P 1000 // Pressure

#define ADDRESS 0b01110110
//#define WRITE_ADDRESS (ADDRESS | 0b0)
//#define READ_ADDRESS (ADDRESS | 0b1)

enum COMMAND {
    RESET = 0x1E,
    D1_CONVERSION = 0x40,
    D2_CONVERSION = 0x50,
    READ_ADC = 0x00,
    READ_PROM = 0xA0,
};

void pt_init();
void pt_reset();
int pt_read_prom(uint8_t addr);
int32_t pt_read_digital_data(COMMAND cmd);
void pt_read_calibration_data_from_prom();
void pt_read_digital_pressure_and_temperature_data(
    uint32_t * pu32D1,
    uint32_t * pu32D2
);
void pt_calculate_temperature(
    uint32_t u32D2, 
    int32_t * ps32dT, 
    int32_t * ps32TEMP
);
void pt_calculate_temperature_compensated_pressure(
    uint32_t u32D1, 
    int32_t s32dT,
    int64_t * ps64OFF,
    int64_t * ps64SENS,
    int32_t * ps32P
);
void pt_calculate_pressure_and_temperature_second_order(
    uint32_t u32D1, 
    int32_t s32dT, 
    int32_t s32TEMP, 
    int64_t s64OFF, 
    int64_t s64SENS, 
    int32_t s32P,
    float * pfTEMP2,
    float * pfP2
);
void read_temp_and_pressure();
