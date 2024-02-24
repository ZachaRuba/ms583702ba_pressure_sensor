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

// INCLUDES
// 

#include <Arduino.h> // Just for types


// DEFINES
//

#define ADDRESS 0b01110110 // Device Address
#define DEG_C_20 2000 // Temperature at 20 Degree

// For Error Checking
#define MAX_CALIBRATION_VALUE 65535
#define MAX_DIGITAL_READ_VALUE 16777216L
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

enum enPT_RETURN_CODES_t {
    SUCCESS = 1,
    FAIL = -1,
    FAIL_BY_BAD_PARAM = -2,
    FAIL_READ_CALIB = -3,
    FAIL_READ_RAW_P_AND_T = -4,
    FAIL_CALC_TEMP = -5,
    FAIL_CALC_PRESSURE = -6,
    FAIL_CALC_P_AND_T_SEC_ORDR = -7,
};

enum enPT_ERRORS_t {
    PT_ERR_BAD_ADDRESS = 0,
    PT_ERR_FAILED_PROM_READ = 1,
    PT_ERR_BAD_CMD = 2,
    PT_ERR_FAILED_DIGITAL_READ = 3,
    PT_ERR_DIGITAL_READ_EXCEED_MAX = 4,
    PT_ERR_FAILED_RAW_P_READ = 5,
    PT_ERR_FAILED_RAW_T_READ = 6,
    PT_ERR_DIFTEMP_EXCEED_MIN = 7,
    PT_ERR_DIFTEMP_EXCEED_MAX = 8,
    PT_ERR_TEMP_EXCEED_MIN = 9,
    PT_ERR_TEMP_EXCEED_MAX = 10,
    PT_ERR_OFFSET_EXCEED_MIN = 11,
    PT_ERR_OFFSET_EXCEED_MAX = 12,
    PT_ERR_SENSITIVITY_EXCEED_MIN = 13,
    PT_ERR_SENSITIVITY_EXCEED_MAX = 14,
    PT_ERR_PRESSURE_EXCEED_MIN = 15,
    PT_ERR_PRESSURE_EXCEED_MAX = 16,
};

// To change OSR etc, 
// See Page 9 of docs/ENG_DS_MS5837-02BA01_A8.pdf
enum COMMAND {
    RESET = 0x1E,
    D1_CONVERSION = 0x4A,
    D2_CONVERSION = 0x5A,
    READ_ADC = 0x00,
    READ_PROM = 0xA0,
};


// EXTERN VARIABLES
//

extern uint32_t g_u32PTErrors;

// PROTOTYPES
//

int pt_init();
void pt_reset();
int32_t pt_read_prom(uint8_t addr);
int32_t pt_read_digital_data(COMMAND cmd);
int pt_read_pressure_and_temperature(float * pfPressure, float * pfTemperature);
