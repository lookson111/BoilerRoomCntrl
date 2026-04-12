/*
 * constants.h
 *
 * Central location for all named constants to eliminate magic numbers
 *
 *  Created on: April 12, 2026
 *      Author: Rinat
 */

#ifndef INC_CONSTANTS_H_
#define INC_CONSTANTS_H_

#include <stdint.h>

#ifdef __cplusplus

/* ============================================================================
 * Display Constants
 * ============================================================================ */

namespace Disp {
constexpr int WIDTH_PIXELS       = 240;
constexpr int HEIGHT_PIXELS      = 320;
constexpr int WIDTH_LANDSCAPE    = 320;  /* Width in landscape orientation */
constexpr int HEIGHT_LANDSCAPE   = 240;  /* Height in landscape orientation */

/* Display UI Layout */
constexpr int TIME_ROW_HEIGHT    = 15;
constexpr int TITLE_ROW_HEIGHT   = 15;
constexpr int FONT_WIDTH         = 12;
constexpr int TITLE_LINE         = 1;
constexpr int LINES_MAX          = 14;
constexpr int TIME_X_POS         = 228;  /* 12 * 19 */
constexpr int VALUES_X_POS       = 216;  /* 12 * 18 */
constexpr int TITLE_X_POS_LEFT   = 12;
constexpr int TITLE_X_POS_RIGHT  = 172;  /* 12 + 160 */
constexpr int HALF_WIDTH         = 160;
}

/* ============================================================================
 * Sensor Constants
 * ============================================================================ */

namespace ADC {
constexpr int SAMPLE_COUNT_TR    = 1000;  /* Number of samples for thermistor reading */
constexpr int SAMPLE_COUNT_PM    = 3;     /* Number of samples for pressure sensor avg */
constexpr int TIMEOUT_MS         = 100;   /* ADC poll timeout in milliseconds */
constexpr int RESOLUTION         = 4095;  /* 12-bit ADC max value */
}

namespace Thermistor {
constexpr int TABLE_SIZE         = 151;   /* Lookup table entries */
constexpr int REF_R              = 3000;  /* Reference resistance factor */
constexpr int BASE_R             = 1000;  /* Base resistance for calculation */
constexpr int BASE_OFFSET        = 26;    /* Temperature table offset */
}

namespace Pressure {
constexpr float VOLT_MIN         = 0.5f;  /* Minimum valid voltage (V) */
constexpr float VOLT_MAX         = 4.5f;  /* Maximum valid voltage (V) */
constexpr float VOLT_DELTA       = 0.05f; /* Voltage tolerance */
constexpr float MULT             = 50.0f; /* Voltage to pressure multiplier */
constexpr float OFFSET           = 25.0f; /* Voltage to pressure offset */
constexpr float DIV              = 14.5038f; /* Voltage to pressure divisor */
constexpr float SCALE            = 1.4751f; /* ADC to voltage scale factor */
constexpr float REF_VOLT         = 3.3f;  /* Reference voltage (V) */
constexpr float MIN_BAR_PER_SEC  = 0.05f; /* Minimum bar/second rate */
constexpr float MIN_POINT        = 1.0f;  /* Minimum pressure setpoint (bar) */
constexpr float MAX_POINT        = 2.0f;  /* Maximum pressure setpoint (bar) */
constexpr int   ERROR_LAG_MS     = 1000;  /* Error detection lag (milliseconds) */
}

namespace DHT22 {
constexpr int READ_INTERVAL_MS   = 2000;  /* Minimum time between reads (ms) */
constexpr int INIT_DELAY_MS      = 250;   /* Initialization stabilization delay */
constexpr int PULL_LOW_DELAY_MS  = 20;    /* Pull low duration (ms) */
constexpr int PULL_HIGH_DELAY_US = 10;    /* Pull high duration (us) */
constexpr int INPUT_SETUP_US     = 1;     /* Input setup time (us) */
constexpr int TIMEOUT_COUNT      = 255;   /* Signal timeout counter */
constexpr int BIT_COUNT          = 40;    /* Expected bit count */
constexpr int DATA_SIZE          = 6;     /* Data buffer size (5 bytes + safety) */
constexpr int BIT_THRESHOLD      = 10;    /* Bit detection threshold */
}

/* ============================================================================
 * Communication Constants
 * ============================================================================ */

namespace Modbus {
constexpr int BAUD_RATE          = 57600;
constexpr int SLAVE_ID           = 1;
constexpr int BAUD_THRESHOLD     = 19200; /* Baud rate threshold for timing */
constexpr int T15_HIGH_BAUD      = 750;   /* 1.5 char time at high baud (us) */
constexpr int T35_HIGH_BAUD      = 1750;  /* 3.5 char time at high baud (us) */
constexpr int T15_MULTIPLIER     = 15000000; /* 1.5 * 10^7 for low baud calculation */
constexpr int T35_MULTIPLIER     = 35000000; /* 3.5 * 10^7 for low baud calculation */
constexpr uint16_t CRC_INIT      = 0xFFFF;
constexpr uint16_t CRC_POLY      = 0xA001;
constexpr int BUFFER_SIZE        = 64;
constexpr int MIN_REQUEST_LEN    = 8;     /* Minimum request packet size */
constexpr uint8_t EXCEPT_BIT     = 0x80;  /* Exception flag bit */
constexpr int EXCEPT_RESPONSE_SIZE = 5;   /* Exception response size in bytes */
constexpr uint16_t CRC_MASK      = 0xFFFF;
}

namespace I2C {
constexpr uint8_t EEPROM_ADDR    = 0x50;
constexpr int CLOCK_SPEED        = 100000;
constexpr uint16_t MEM_ADDR_INIT = 0x0010;
constexpr int TIMEOUT_MS         = 100;
}

/* ============================================================================
 * Time Constants
 * ============================================================================ */

namespace Time {
constexpr int HOUR_MAX           = 24;
constexpr int HOUR_MIN           = 0;
constexpr int MINUTE_MAX         = 60;
constexpr int MINUTE_MIN         = 0;
constexpr int SECOND_MAX         = 60;
constexpr int SECOND_MIN         = 0;
constexpr int DAY_MAX            = 31;
constexpr int DAY_MIN            = 1;
constexpr int MONTH_MAX          = 12;
constexpr int MONTH_MIN          = 1;
constexpr int YEAR_MAX           = 99;
constexpr int YEAR_MIN           = 0;

/* Default RTC values */
constexpr int DEFAULT_HOUR     = 12;
constexpr int DEFAULT_MINUTE   = 0;
constexpr int DEFAULT_SECOND   = 0;
constexpr int DEFAULT_DAY      = 1;
constexpr int DEFAULT_MONTH    = 1;  /* January */
constexpr int DEFAULT_YEAR     = 25;
}

namespace RTOS {
constexpr int TICK_RATE_HZ     = 1000;

/* Task stack sizes */
constexpr int TASK_STACK_DEFAULT = 128;   /* Default task stack (words) */
constexpr int TASK_STACK_SENSOR  = 128;   /* Sensor reading task stack */
constexpr int TASK_STACK_DISPLAY = 800;   /* Display task stack */

/* Task delays */
constexpr int DELAY_INIT_MS        = 900;
constexpr int DELAY_SENSOR_INIT_MS = 1000;
constexpr int DELAY_BUTTON_DEBOUNCE_MS = 100;
constexpr int DELAY_EDIT_EXIT_MS   = 200;
constexpr int DELAY_TIME_UPDATE_MS = 900;
constexpr int DELAY_TIME_BASE_MS   = 1000;
}

// Legacy compatibility
namespace FreeRTOSCompat {
constexpr int TICK_RATE_HZ = RTOS::TICK_RATE_HZ;
constexpr int TASK_STACK_DEFAULT = RTOS::TASK_STACK_DEFAULT;
constexpr int TASK_STACK_SENSOR = RTOS::TASK_STACK_SENSOR;
constexpr int TASK_STACK_DISPLAY = RTOS::TASK_STACK_DISPLAY;
constexpr int DELAY_INIT_MS = RTOS::DELAY_INIT_MS;
constexpr int DELAY_SENSOR_INIT_MS = RTOS::DELAY_SENSOR_INIT_MS;
constexpr int DELAY_BUTTON_DEBOUNCE_MS = RTOS::DELAY_BUTTON_DEBOUNCE_MS;
constexpr int DELAY_EDIT_EXIT_MS = RTOS::DELAY_EDIT_EXIT_MS;
constexpr int DELAY_TIME_UPDATE_MS = RTOS::DELAY_TIME_UPDATE_MS;
constexpr int DELAY_TIME_BASE_MS = RTOS::DELAY_TIME_BASE_MS;
}

/* ============================================================================
 * Timer Config Constants
 * ============================================================================ */

namespace CfgTIM4 {
constexpr int PRESCALER = 71;    /* 72MHz / (71+1) = 1MHz */
constexpr int PERIOD    = 1562;  /* ~1.56ms period */
}

namespace CfgTIM3 {
constexpr int PRESCALER = 20;
}

namespace CfgTIM2 {
constexpr int MAX_COUNT = 65535;
}

/* ============================================================================
 * Menu System Constants
 * ============================================================================ */

namespace Menu {
constexpr int NAME_STR_LEN       = 17;    /* Characters in menu name strings */
constexpr int VALUE_STR_LEN      = 7;     /* Characters in value strings */
constexpr int TITLE_STR_LEN      = 13;    /* Characters in title strings */

/* Menu item types */
enum class Type : uint8_t {
    NONE   = 0,
    ONOFF  = 1,
    INT    = 2,
    FLOAT  = 3,
    TIME   = 4
};

/* Relay count */
constexpr int RELAY_COUNT        = 6;
}

/* ============================================================================
 * Data Buffer Sizes
 * ============================================================================ */

constexpr int UART_BUFFER_SIZE   = 32;
constexpr int EEPROM_DATA_SIZE   = 32;

/* ============================================================================
 * Error Codes
 * ============================================================================ */

namespace Error {
constexpr uint32_t NONE                   = 0x00000000;
constexpr uint32_t PRESS_MET_OUT_OF_VOLT  = 0x00000001;
constexpr uint32_t PRESS_MET_OUT_OF_BAR   = 0x00000002;
constexpr uint32_t WIRE_BREAK             = 0x00000004;
}

/* ============================================================================
 * Miscellaneous
 * ============================================================================ */

namespace Float {
constexpr float TEMP_MAX       = 100.0f;
constexpr float TEMP_MIN       = -50.0f;
constexpr int SCALE_FACTOR     = 100;   /* Scale factor for float to int conversion */
}

namespace Str {
constexpr int BUF_SIZE_6       = 6;     /* 6-character string buffers */
constexpr char NULL_TERM       = 0x00;
}

namespace Encoder {
constexpr uint8_t DIR_CW       = 0x10;
constexpr uint8_t DIR_CCW      = 0x20;
constexpr uint8_t STATE_MASK   = 0x0F;
}

namespace CfgEXTI {
constexpr int LINE_10_BIT      = 10;
constexpr int LINE_11_BIT      = 11;
constexpr int LINE_12_BIT      = 12;
}

namespace CfgNVIC {
constexpr int PRIORITY_5       = 5;
constexpr int PRIORITY_15      = 15;
}

namespace ADCChannels {
constexpr int TR_1             = 1;
constexpr int TR_2             = 2;
constexpr int TR_3             = 3;
constexpr int TR_4             = 4;
constexpr int PM_1             = 8;
constexpr int PM_2             = 9;
}

namespace VoltDiv {
constexpr int FACTOR_3000_4095 = 3000;  /* Thermistor calculation factor */
constexpr int FACTOR_4095      = 4095;  /* ADC resolution factor */
}

namespace UART {
constexpr int TRANSMIT_TIMEOUT = 10;    /* UART transmit timeout in ms */
}

// Legacy compatibility macros (deprecated, use namespace constants instead)
#define DISP_WIDTH_PIXELS       Disp::WIDTH_PIXELS
#define DISP_HEIGHT_PIXELS      Disp::HEIGHT_PIXELS
#define ADC_SAMPLE_COUNT_TR     ADC::SAMPLE_COUNT_TR
#define ADC_SAMPLE_COUNT_PM     ADC::SAMPLE_COUNT_PM
#define MENU_TYPE_NONE          static_cast<int>(Menu::Type::NONE)
#define MENU_TYPE_ONOFF         static_cast<int>(Menu::Type::ONOFF)
#define MENU_TYPE_INT           static_cast<int>(Menu::Type::INT)
#define MENU_TYPE_FLOAT         static_cast<int>(Menu::Type::FLOAT)
#define MENU_TYPE_TIME          static_cast<int>(Menu::Type::TIME)
#define RELAY_COUNT             Menu::RELAY_COUNT
#define UART_BUFFER_SIZE        ::UART_BUFFER_SIZE
#define EEPROM_DATA_SIZE        ::EEPROM_DATA_SIZE

// Additional backward-compat macros for C++ code
#define MENU_NAME_STR_LEN       Menu::NAME_STR_LEN
#define MENU_VALUE_STR_LEN      Menu::VALUE_STR_LEN
#define MENU_TITLE_STR_LEN      Menu::TITLE_STR_LEN
#define MODBUS_SLAVE_ID         Modbus::SLAVE_ID
#define MODBUS_BAUD_RATE        Modbus::BAUD_RATE
#define MODBUS_BAUD_THRESHOLD   Modbus::BAUD_THRESHOLD
#define MODBUS_T15_HIGH_BAUD    Modbus::T15_HIGH_BAUD
#define MODBUS_T35_HIGH_BAUD    Modbus::T35_HIGH_BAUD
#define MODBUS_T15_MULTIPLIER   Modbus::T15_MULTIPLIER
#define MODBUS_T35_MULTIPLIER   Modbus::T35_MULTIPLIER
#define MODBUS_BUFFER_SIZE      Modbus::BUFFER_SIZE
#define MODBUS_MIN_REQUEST_LEN  Modbus::MIN_REQUEST_LEN
#define MODBUS_CRC_INIT         Modbus::CRC_INIT
#define MODBUS_CRC_POLY         Modbus::CRC_POLY
#define MODBUS_CRC_MASK         Modbus::CRC_MASK
#define MODBUS_EXCEPT_BIT       Modbus::EXCEPT_BIT
#define MODBUS_EXCEPT_RESPONSE_SIZE Modbus::EXCEPT_RESPONSE_SIZE
#define DHT22_DATA_SIZE         6
#define DHT22_READ_INTERVAL_MS  2000
#define DHT22_INIT_DELAY_MS     250
#define DHT22_PULL_LOW_DELAY_MS 20
#define DHT22_PULL_HIGH_DELAY_US 10
#define DHT22_INPUT_SETUP_US    1
#define DHT22_TIMEOUT_COUNT     255
#define DHT22_BIT_COUNT         40
#define DHT22_BIT_THRESHOLD     10
#define TASK_STACK_DEFAULT      RTOS::TASK_STACK_DEFAULT
#define TASK_STACK_SENSOR       RTOS::TASK_STACK_SENSOR
#define TASK_STACK_DISPLAY      RTOS::TASK_STACK_DISPLAY
#define DELAY_INIT_MS           RTOS::DELAY_INIT_MS
#define DELAY_SENSOR_INIT_MS    RTOS::DELAY_SENSOR_INIT_MS
#define DELAY_BUTTON_DEBOUNCE_MS RTOS::DELAY_BUTTON_DEBOUNCE_MS
#define DELAY_EDIT_EXIT_MS      RTOS::DELAY_EDIT_EXIT_MS
#define DELAY_TIME_UPDATE_MS    RTOS::DELAY_TIME_UPDATE_MS
#define DELAY_TIME_BASE_MS      RTOS::DELAY_TIME_BASE_MS
#define I2C_CLOCK_SPEED         I2C::CLOCK_SPEED
#define I2C_TIMEOUT_MS          I2C::TIMEOUT_MS
#define I2C_EEPROM_ADDR         0x50
#define I2C_MEM_ADDR_INIT       0x0010
#define ADC_TIMEOUT_MS          ADC::TIMEOUT_MS
#define ADC_RESOLUTION          ADC::RESOLUTION
#define ADC_SAMPLE_COUNT_TR     ADC::SAMPLE_COUNT_TR
#define ADC_SAMPLE_COUNT_PM     ADC::SAMPLE_COUNT_PM
#define THERMISTOR_TABLE_SIZE   Thermistor::TABLE_SIZE
#define THERMISTOR_BASE_R       Thermistor::BASE_R
#define THERMISTOR_REF_R        Thermistor::REF_R
#define THERMISTOR_BASE_OFFSET  Thermistor::BASE_OFFSET
#define PRESSURE_VOLT_MIN       Pressure::VOLT_MIN
#define PRESSURE_VOLT_MAX       Pressure::VOLT_MAX
#define PRESSURE_VOLT_DELTA     Pressure::VOLT_DELTA
#define PRESSURE_ERROR_LAG_MS   Pressure::ERROR_LAG_MS
#define PRESSURE_MIN_POINT      Pressure::MIN_POINT
#define PRESSURE_MAX_POINT      Pressure::MAX_POINT
#define PRESSURE_MIN_BAR_PER_SEC Pressure::MIN_BAR_PER_SEC
#define VOLT_DIV_4095           VoltDiv::FACTOR_4095
#define VOLT_DIV_4095_DIV       VoltDiv::FACTOR_4095
#define PRESSURE_REF_VOLT       Pressure::REF_VOLT
#define PRESSURE_SCALE          Pressure::SCALE
#define PRESSURE_MULT           Pressure::MULT
#define PRESSURE_OFFSET         Pressure::OFFSET
#define PRESSURE_DIV            Pressure::DIV
#define RTC_DEFAULT_HOUR        Time::DEFAULT_HOUR
#define RTC_DEFAULT_MINUTE      Time::DEFAULT_MINUTE
#define RTC_DEFAULT_SECOND      Time::DEFAULT_SECOND
#define RTC_DEFAULT_DAY         Time::DEFAULT_DAY
#define RTC_DEFAULT_MONTH       Time::DEFAULT_MONTH
#define RTC_DEFAULT_YEAR        Time::DEFAULT_YEAR
#define TIM4_PRESCALER          CfgTIM4::PRESCALER
#define TIM4_PERIOD             CfgTIM4::PERIOD
#define TIM3_PRESCALER          CfgTIM3::PRESCALER
#define TIM2_MAX_COUNT          CfgTIM2::MAX_COUNT
#define UART_TRANSMIT_TIMEOUT   UART::TRANSMIT_TIMEOUT
#define ENCODER_DIR_CW          Encoder::DIR_CW
#define ENCODER_DIR_CCW         Encoder::DIR_CCW
#define ENCODER_STATE_MASK      Encoder::STATE_MASK
#define EXTI_LINE_10_BIT        CfgEXTI::LINE_10_BIT
#define EXTI_LINE_11_BIT        CfgEXTI::LINE_11_BIT
#define EXTI_LINE_12_BIT        CfgEXTI::LINE_12_BIT

// Display layout backward-compat macros (plain values for use in array dims/macro args)
#define DISP_TITLE_LINE         1
#define DISP_LINES_MAX          14
#define DISP_TIME_X_POS         228
#define DISP_TIME_ROW_HEIGHT    15
#define DISP_HALF_WIDTH         160
#define DISP_TITLE_X_POS_LEFT   12
#define DISP_TITLE_X_POS_RIGHT  172
#define DISP_VALUES_X_POS       216
#define DISP_FONT_WIDTH         12

// Time constants (plain values)
#define TIME_HOUR_MAX           24
#define TIME_HOUR_MIN           0
#define TIME_MINUTE_MAX         60
#define TIME_MINUTE_MIN         0
#define TIME_SECOND_MAX         60
#define TIME_SECOND_MIN         0
#define TIME_DAY_MAX            31
#define TIME_DAY_MIN            1
#define TIME_MONTH_MAX          12
#define TIME_MONTH_MIN          1
#define TIME_YEAR_MAX           99
#define TIME_YEAR_MIN           0

// String/misc backward-compat macros
#define STR_BUF_SIZE_6          6
#define STR_NULL_TERMINATOR     0x00
#define FLOAT_TEMP_MAX          100.0f
#define FLOAT_TEMP_MIN          -50.0f
#define FLOAT_SCALE_FACTOR      100

// Error codes
#define ERROR_PRESS_MET_OUT_OF_VOLT      0x00000001
#define ERROR_PRESS_MET_OUT_OF_BAR       0x00000002
#define ERROR_WIRE_BREAK                 0x00000004
#define ERROR_PRESS_MET_OUT_OF_BAR_PER_SECOND  0x00000008

// FreeRTOS
#define FREERTOS_TICK_RATE_HZ           1000

#else /* __cplusplus - C compatibility */

/* C-compatible constants (plain #defines for use from C code) */

/* Display */
#define DISP_WIDTH_PIXELS       240
#define DISP_HEIGHT_PIXELS      320
#define DISP_WIDTH_LANDSCAPE    320
#define DISP_HEIGHT_LANDSCAPE   240
#define DISP_TIME_ROW_HEIGHT    15
#define DISP_TITLE_ROW_HEIGHT   15
#define DISP_FONT_WIDTH         12
#define DISP_TITLE_LINE         1
#define DISP_LINES_MAX          14
#define DISP_TIME_X_POS         228
#define DISP_VALUES_X_POS       216
#define DISP_TITLE_X_POS_LEFT   12
#define DISP_TITLE_X_POS_RIGHT  172
#define DISP_HALF_WIDTH         160

/* ADC */
#define ADC_SAMPLE_COUNT_TR     1000
#define ADC_SAMPLE_COUNT_PM     3
#define ADC_TIMEOUT_MS          100
#define ADC_RESOLUTION          4095

/* Thermistor */
#define THERMISTOR_TABLE_SIZE   151
#define THERMISTOR_REF_R        3000
#define THERMISTOR_BASE_R       1000
#define THERMISTOR_BASE_OFFSET  26

/* Pressure */
#define PRESSURE_VOLT_MIN       0.5f
#define PRESSURE_VOLT_MAX       4.5f
#define PRESSURE_VOLT_DELTA     0.05f
#define PRESSURE_MULT           50.0f
#define PRESSURE_OFFSET         25.0f
#define PRESSURE_DIV            14.5038f
#define PRESSURE_SCALE          1.4751f
#define PRESSURE_REF_VOLT       3.3f
#define PRESSURE_MIN_BAR_PER_SEC 0.05f
#define PRESSURE_MIN_POINT      1.0f
#define PRESSURE_MAX_POINT      2.0f
#define PRESSURE_ERROR_LAG_MS   1000

/* DHT22 */
#define DHT22_READ_INTERVAL_MS  2000
#define DHT22_INIT_DELAY_MS     250
#define DHT22_PULL_LOW_DELAY_MS 20
#define DHT22_PULL_HIGH_DELAY_US 10
#define DHT22_INPUT_SETUP_US    1
#define DHT22_TIMEOUT_COUNT     255
#define DHT22_BIT_COUNT         40
#define DHT22_DATA_SIZE         6
#define DHT22_BIT_THRESHOLD     10

/* Modbus */
#define MODBUS_BAUD_RATE                57600
#define MODBUS_SLAVE_ID                 1
#define MODBUS_BAUD_THRESHOLD           19200
#define MODBUS_T15_HIGH_BAUD            750
#define MODBUS_T35_HIGH_BAUD            1750
#define MODBUS_T15_MULTIPLIER           15000000
#define MODBUS_T35_MULTIPLIER           35000000
#define MODBUS_BUFFER_SIZE              64
#define MODBUS_MIN_REQUEST_LEN          8
#define MODBUS_CRC_INIT                 0xFFFF
#define MODBUS_CRC_POLY                 0xA001
#define MODBUS_CRC_MASK                 0xFFFF
#define MODBUS_EXCEPT_BIT               0x80
#define MODBUS_EXCEPT_RESPONSE_SIZE     5

/* I2C */
#define I2C_CLOCK_SPEED         100000
#define I2C_TIMEOUT_MS          100

/* Time */
#define TIME_HOUR_MAX           24
#define TIME_HOUR_MIN           0
#define TIME_MINUTE_MAX         60
#define TIME_MINUTE_MIN         0
#define TIME_SECOND_MAX         60
#define TIME_SECOND_MIN         0
#define TIME_DAY_MAX            31
#define TIME_DAY_MIN            1
#define TIME_MONTH_MAX          12
#define TIME_MONTH_MIN          1
#define TIME_YEAR_MAX           99
#define TIME_YEAR_MIN           0

/* FreeRTOS */
#define FREERTOS_TICK_RATE_HZ           1000
#define FREERTOS_TASK_STACK_DEFAULT     128
#define FREERTOS_TASK_STACK_SENSOR      128
#define FREERTOS_TASK_STACK_DISPLAY     800
#define DELAY_INIT_MS                   900
#define DELAY_SENSOR_INIT_MS            1000
#define DELAY_BUTTON_DEBOUNCE_MS        100
#define DELAY_EDIT_EXIT_MS              200
#define DELAY_TIME_UPDATE_MS            900
#define DELAY_TIME_BASE_MS              1000

/* Timer */
#define TIM4_PRESCALER          71
#define TIM4_PERIOD             1562
#define TIM3_PRESCALER          20
#define TIM2_MAX_COUNT          65535

/* Menu */
#define MENU_NAME_STR_LEN       17
#define MENU_VALUE_STR_LEN      7
#define MENU_TITLE_STR_LEN      13
#define MENU_TYPE_NONE          0
#define MENU_TYPE_ONOFF         1
#define MENU_TYPE_INT           2
#define MENU_TYPE_FLOAT         3
#define MENU_TYPE_TIME          4
#define RELAY_COUNT             6
#define CNTMSYMINSTR            MENU_NAME_STR_LEN
#define CNTVSYMINSTR            MENU_VALUE_STR_LEN

/* Buffer sizes */
#define UART_BUFFER_SIZE        32
#define EEPROM_DATA_SIZE        32
#define UART_TRANSMIT_TIMEOUT   10

/* Error codes */
#define ERROR_PRESS_MET_OUT_OF_VOLT  0x00000001
#define ERROR_PRESS_MET_OUT_OF_BAR   0x00000002
#define ERROR_WIRE_BREAK             0x00000004

/* Miscellaneous */
#define FLOAT_TEMP_MAX          100.0f
#define FLOAT_TEMP_MIN          -50.0f
#define FLOAT_SCALE_FACTOR      100

#endif /* __cplusplus */

#endif /* INC_CONSTANTS_H_ */
