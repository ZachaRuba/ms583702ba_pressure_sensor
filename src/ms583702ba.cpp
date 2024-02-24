//********************************************************************
//
// ms583702ba.cpp - Pressure Transducer Module
//
// Author: Zachary Anderson
// Date: Feb 08, 2024
//
// Note:
//  ----
// see docs/ENG_DS_MS5837-02BA01_A8.pdf
//
//********************************************************************

// INCLUDES & DEFINES
//

#include <Arduino.h>
#include <Wire.h>
#include "ms583702ba.h"

// GLOBALS
//

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

// For Error Tracking
uint32_t g_u32PTErrors;

// PROTOTYPES
//

int pt_read_calibration_data_from_prom();

// FUNCTIONS
//

//*****************************************************************************
//
//  pt_init - Initialize the pressure transducer
//
//  Parameters - None.
//
//  Returns - SUCCESS (1) on completion or 0 > on failure
//
//*****************************************************************************
int pt_init(){
    g_u32PTErrors = 0;
    Wire.begin();
    pt_reset();
    return pt_read_calibration_data_from_prom();
}

//*****************************************************************************
//
//  pt_reset - Send a reset command to pt (required after power up)
//
//  Parameters - None.
//
//  Returns - None.
//
//*****************************************************************************
void pt_reset(){
    // write Reset Sequence
    Wire.beginTransmission(ADDRESS);
    Wire.write(RESET);
    Wire.endTransmission();
}

//*****************************************************************************
//
//  pt_read_prom - Read a 16bit register from PROM on pt
//
//  Parameters - byte adder - number or register to read (0-6)
//
//  Returns - long containing 16bit result or Failure code (<0)
//
//  Notes:
//      pg 13 - memory mapping
//      address 0 - crc [15:12] | Product Type [11:5] | factory settings [4:0]
//      address 1-6 - C1-C6
//
//*****************************************************************************
int32_t pt_read_prom(uint8_t addr){
    uint16_t upper_bytes = 0;
    uint16_t lower_bytes = 0;
    int32_t reg = FAIL;

    // Check address is correct
    if (addr > 6) {
        g_u32PTErrors |= 1UL << PT_ERR_BAD_ADDRESS;
        return FAIL_BY_BAD_PARAM;
    }

    // write i2c cmd for reading a prom memory
    Wire.beginTransmission(ADDRESS);
    Wire.write(READ_PROM | (addr << 1));
    Wire.endTransmission();

    // read i2c cmd to get data from memory (16bits) (ie 2 bytes)
    Wire.requestFrom(ADDRESS,2);
    if(Wire.available() == 2) {
        upper_bytes = Wire.read();
        lower_bytes = Wire.read();
        reg = (upper_bytes << 8) | lower_bytes;
    }
    else g_u32PTErrors |= 1UL << PT_ERR_FAILED_PROM_READ;

    return reg;
}


//*****************************************************************************
//
//  pt_read_digital_data - Take a measurement and Read 24bit value from the 
//      adc register
//
//  Parameters - None.
//
//  Returns - SUCCESS (1) on completion or 0 > on failure
//
//*****************************************************************************
int32_t pt_read_digital_data(COMMAND cmd){
    uint16_t upper_bytes = 0;
    uint16_t middle_bytes = 0;
    uint16_t lower_bytes = 0;
    int32_t reg = FAIL;

    if (cmd != D1_CONVERSION && cmd != D2_CONVERSION){
        g_u32PTErrors |= 1UL << PT_ERR_BAD_CMD;
        return FAIL_BY_BAD_PARAM;
    }

    // write i2c cmd to do conversion
    Wire.beginTransmission(ADDRESS);
    Wire.write(cmd);
    Wire.endTransmission();

    // Delay for ADC conversion time
    // pg 2 of docs/ENG_DS_MS5837-02BA01_A8.pdf
    delay(20);

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
        reg = (uint32_t)upper_bytes << 16 | middle_bytes << 8 | lower_bytes;
    }
    else {
        g_u32PTErrors |= 1UL << PT_ERR_FAILED_DIGITAL_READ;
    }

    g_u32PTErrors |= \
        ((uint32_t)(reg > MAX_DIGITAL_READ_VALUE)) << PT_ERR_DIGITAL_READ_EXCEED_MAX;

    return reg;
}

//*****************************************************************************
//
//  pt_read_calibration_data_from_prom - Call once to load all calibration
//      constants from PROM
//
//  Parameters - None.
//
//  Returns - SUCCESS (1) on completion or FAIL_READ_CALIB (-3)
//
//*****************************************************************************
int pt_read_calibration_data_from_prom(){
    uint16_t * au16C[6] = {&g_u16C1, &g_u16C2, &g_u16C3, &g_u16C4, &g_u16C5, &g_u16C6};
    int32_t s32PromVal;
    int ret = SUCCESS;

    for (int i = 1; i < 7; i++){
        s32PromVal = pt_read_prom(i);
        if (s32PromVal < 0) ret = FAIL_READ_CALIB;  
        else *(au16C[i-1]) = (uint16_t) s32PromVal;
    }

    return ret;
}

//*****************************************************************************
//
//  pt_read_digital_pressure_and_temperature_data - Read Temperature and 
//      Pressure from the pt's adc.
//
//  Parameters
//      uint32_t * pu32D1 - ptr to return pressure adc value (24bit)
//      uint32_t * pu32D2 - ptr to return temperature adc value (24bit)
//
//  Returns - SUCCESS (1) on completion or FAIL_READ_RAW_P_AND_T (-4)
//
//*****************************************************************************
int pt_read_digital_pressure_and_temperature_data(
    uint32_t * pu32D1,
    uint32_t * pu32D2
){
    uint32_t u32D1; // Digital Pressure Value
    uint32_t u32D2; // Digital Temperature Value
    uint32_t u32PTErrors = 0;

    u32D1 = (uint32_t) pt_read_digital_data(D1_CONVERSION);
    u32D2 = (uint32_t) pt_read_digital_data(D2_CONVERSION);

    // Check Values are ok.
    if (u32D1 > MAX_DIGITAL_READ_VALUE) u32PTErrors |= 1UL << PT_ERR_FAILED_RAW_P_READ;
    if (u32D2 > MAX_DIGITAL_READ_VALUE) u32PTErrors |= 1UL << PT_ERR_FAILED_RAW_T_READ;
    g_u32PTErrors |= u32PTErrors;

    // return u32D1 u32D2
    *pu32D1 = (uint32_t) u32D1;
    *pu32D2 = (uint32_t) u32D2;
    return (u32PTErrors == 0 ? SUCCESS : FAIL_READ_RAW_P_AND_T);
}

//*****************************************************************************
//
//  pt_calculate_temperature - Calculate dT and Temp from raw temp adc value
//
//  Parameters
//      uint32_t u32D2 - pt's raw Temperature adc value (24bit)
//      int32_t * ps32dT - ptr to return Temp Delta ie TEMP - TREF(C5)
//      int32_t * ps32TEMP - ptr to return Temperature (TEMP) in "centi" deg C
//
//  Returns - SUCCESS (1) on completion or FAIL_CALC_TEMP (-5)
//
//  Note:
//      The TEMP value will be an int of centi-Celsius.  2000 -> 20.00 deg C
//      ENG_DS_MS5837-02BA01_A8.pdf pg 7
//
//*****************************************************************************
int pt_calculate_temperature(
    uint32_t u32D2, 
    int32_t * ps32dT, 
    int32_t * ps32TEMP
){
    int32_t s32dT; // Difference between actual and reference temperature
    int32_t s32TEMP; // Actual Temperature (-40 to 85degC with 0.01degC resolution)
    uint32_t u32PTErrors = 0;

    // Calculate Values
    s32dT = u32D2 - g_u16C5 * (1UL << 8); //256L; // 1 << 8 is equivalent to Pow(2,8)
    s32TEMP = DEG_C_20 + s32dT * g_u16C6 / (1UL << 23); //8388608L;  // 1 << 23 is equivalent to Pow(2,23)

    // Check Values are ok.
    u32PTErrors |= ((uint32_t)(MIN_DT > s32dT)) << PT_ERR_DIFTEMP_EXCEED_MIN;
    u32PTErrors |= ((uint32_t)(MAX_DT < s32dT)) << PT_ERR_DIFTEMP_EXCEED_MAX;
    u32PTErrors |= ((uint32_t)(MIN_TEMP_VALUE > s32TEMP)) << PT_ERR_TEMP_EXCEED_MIN;
    u32PTErrors |= ((uint32_t)(MAX_TEMP_VALUE < s32TEMP)) << PT_ERR_TEMP_EXCEED_MAX;
    g_u32PTErrors |= u32PTErrors;

    // return s32dT s32TEMP
    *ps32dT = s32dT;
    *ps32TEMP = s32TEMP;
    return (u32PTErrors == 0 ? SUCCESS : FAIL_CALC_TEMP);
}

//*****************************************************************************
//
//  pt_calculate_temperature_compensated_pressure - Convert pressure adc value
//      and temp delta to a compensated Pressure value (in centi-mBar)
//
//  Parameters
//      uint32_t u32D1 - pt's raw pressure adc value (24bit)
//      int32_t s32dT - pt's Temp Delta ie TEMP - TREF(C5)
//      int64_t * ps64OFF - ptr to return Pressure Offset value 
//      int64_t * ps64SENS - ptr to return Pressure Sensitivity
//      int32_t * ps32P - ptr to return Pressure ("centi" mBar)
//
//  Returns - SUCCESS (1) on completion or FAIL_CALC_PRESSURE (-6)
//
//  Note:
//      The Pressure is int of centi-millibar.  110002 -> 1100.02 mbar
//      ENG_DS_MS5837-02BA01_A8.pdf pg 7
//
//*****************************************************************************
int pt_calculate_temperature_compensated_pressure(
    uint32_t u32D1, 
    int32_t s32dT,
    int64_t * ps64OFF,
    int64_t * ps64SENS,
    int32_t * ps32P
){
    int64_t s64OFF; // Offset at actual temperature
    int64_t s64SENS; // Sensitivity at actual temperature
    int32_t s32P; // Temperature Compensated Pressure (10-1200mbar with 0.01mbar resolution)
    uint32_t u32PTErrors = 0;

    // Calculate Values
    s64OFF = (g_u16C2) * (1ULL << 17) + (g_u16C4 * s32dT) / (1U << 6);
    s64SENS = g_u16C1 * (1ULL << 16) + (g_u16C3 * s32dT) / (1U << 7);
    s32P = (u32D1 * s64SENS / (1ULL << 21) - s64OFF) / (1U << 15);
    
    // Check Values are ok.
    u32PTErrors |= ((uint32_t)(MIN_OFFSET > s64OFF)) << PT_ERR_OFFSET_EXCEED_MIN;
    u32PTErrors |= ((uint32_t)(MAX_OFFSET < s64OFF)) << PT_ERR_OFFSET_EXCEED_MAX;
    u32PTErrors |= ((uint32_t)(MIN_SENS > s64SENS)) << PT_ERR_SENSITIVITY_EXCEED_MIN;
    u32PTErrors |= ((uint32_t)(MAX_SENS < s64SENS)) << PT_ERR_SENSITIVITY_EXCEED_MAX;
    u32PTErrors |= ((uint32_t)(MIN_P > s32P)) << PT_ERR_PRESSURE_EXCEED_MIN;
    u32PTErrors |= ((uint32_t)(MAX_P < s32P)) << PT_ERR_PRESSURE_EXCEED_MAX;
    g_u32PTErrors |= u32PTErrors;

    // return s64OFF, s64SENS, s32P
    *ps64OFF = s64OFF;
    *ps64SENS = s64SENS;
    *ps32P = s32P;
    return (u32PTErrors == 0 ? SUCCESS : FAIL_CALC_PRESSURE);
}

//*****************************************************************************
//
//  pt_calculate_pressure_and_temperature_second_order - Preform a "second ord"
//      calculation of Pressure and Temp and conv to float in mbar and degC.
//
//  Parameters
//       uint32_t u32D1 - pt's raw pressure adc value (24bit)
//       int32_t s32dT - pt's Temp Delta ie TEMP - TREF(C5)
//       int32_t s32TEMP - pt's Temperature (TEMP) in "centi" deg C
//       int64_t s64OFF - pt's Pressure Offset value
//       int64_t s64SENS - pt's Pressure Sensitivity
//       int32_t s32P - pt's Pressure ("centi" mBar)
//       float * pfTEMP2 - ptr to return Second Order Temp in DegC
//       float * pfP2 - ptr to return Second Order Pressure in millibar
//
//  Returns - SUCCESS (1) on completion or FAIL_CALC_P_AND_T_SEC_ORDR (-7)
//
//  Note:
//      ENG_DS_MS5837-02BA01_A8.pdf pg 8
//
//*****************************************************************************
int pt_calculate_pressure_and_temperature_second_order(
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
    return SUCCESS;
}

//*****************************************************************************
//
//  pt_calculate_pressure_and_temperature_second_order - Preform a "second ord"
//      calculation of Pressure and Temp and conv to float in mbar and degC.
//
//  Parameters
//       float * fPressure - ptr to return Second Order Pressure in millibar 
//       float * fTemp - ptr to return Second Order Temp in DegC
//
//  Returns - SUCCESS (1) on completion or Failure Code (<0)
//
//  Note:
//      ENG_DS_MS5837-02BA01_A8.pdf pg 7/8
//
//*****************************************************************************
int pt_read_pressure_and_temperature(float * pfPressure, float * pfTemperature){
    uint32_t u32D1;
    uint32_t u32D2;
    int32_t s32dT;
    int32_t s32TEMP;
    int64_t s64OFF;
    int64_t s64SENS;
    int32_t s32P;
    int ret;

    ret = pt_read_digital_pressure_and_temperature_data(&u32D1,&u32D2);

    if (ret != SUCCESS) goto end;
    ret = pt_calculate_temperature(u32D2, &s32dT, &s32TEMP);

    if (ret != SUCCESS) goto end;
    ret = pt_calculate_temperature_compensated_pressure(
        u32D1, s32dT, &s64OFF, &s64SENS, &s32P
    );

    if (ret != SUCCESS) goto end;
    ret = pt_calculate_pressure_and_temperature_second_order(
        u32D1,s32dT,s32TEMP,s64OFF,s64SENS,s32P,pfTemperature,pfPressure
    );

    end:
    return ret;    
}
