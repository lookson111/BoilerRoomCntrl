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

/* ============================================================================
 * Display Constants
 * ============================================================================ */

/* ILI9341 Display Dimensions */
#define DISP_WIDTH_PIXELS       240
#define DISP_HEIGHT_PIXELS      320
#define DISP_WIDTH_LANDSCAPE    320  /* Width in landscape orientation */
#define DISP_HEIGHT_LANDSCAPE   240  /* Height in landscape orientation */

/* Display UI Layout */
#define DISP_TIME_ROW_HEIGHT    15
#define DISP_TITLE_ROW_HEIGHT   15
#define DISP_FONT_WIDTH         12
#define DISP_TITLE_LINE         1
#define DISP_LINES_MAX          14
#define DISP_TIME_X_POS         228  /* 12 * 19 */
#define DISP_VALUES_X_POS       216  /* 12 * 18 */
#define DISP_TITLE_X_POS_LEFT   12
#define DISP_TITLE_X_POS_RIGHT  172  /* 12 + 160 */
#define DISP_HALF_WIDTH         160

/* ============================================================================
 * Sensor Constants
 * ============================================================================ */

/* ADC Sampling */
#define ADC_SAMPLE_COUNT_TR     1000  /* Number of samples for thermistor reading */
#define ADC_SAMPLE_COUNT_PM     3     /* Number of samples for pressure sensor avg */
#define ADC_TIMEOUT_MS          100   /* ADC poll timeout in milliseconds */
#define ADC_RESOLUTION          4095  /* 12-bit ADC max value */

/* NTC Thermistor */
#define THERMISTOR_TABLE_SIZE   151   /* Lookup table entries */
#define THERMISTOR_REF_R        3000  /* Reference resistance factor */
#define THERMISTOR_BASE_R       1000  /* Base resistance for calculation */
#define THERMISTOR_BASE_OFFSET  26    /* Temperature table offset */

/* Pressure Sensor */
#define PRESSURE_VOLT_MIN       0.5f  /* Minimum valid voltage (V) */
#define PRESSURE_VOLT_MAX       4.5f  /* Maximum valid voltage (V) */
#define PRESSURE_VOLT_DELTA     0.05f /* Voltage tolerance */
#define PRESSURE_MULT           50.0f /* Voltage to pressure multiplier */
#define PRESSURE_OFFSET         25.0f /* Voltage to pressure offset */
#define PRESSURE_DIV            14.5038f /* Voltage to pressure divisor */
#define PRESSURE_SCALE          1.4751f /* ADC to voltage scale factor */
#define PRESSURE_REF_VOLT       3.3f  /* Reference voltage (V) */
#define PRESSURE_MIN_BAR_PER_SEC 0.05f /* Minimum bar/second rate */
#define PRESSURE_MIN_POINT      1.0f  /* Minimum pressure setpoint (bar) */
#define PRESSURE_MAX_POINT      2.0f  /* Maximum pressure setpoint (bar) */
#define PRESSURE_ERROR_LAG_MS   1000  /* Error detection lag (milliseconds) */

/* DHT22 Sensor Timing */
#define DHT22_READ_INTERVAL_MS  2000  /* Minimum time between reads (ms) */
#define DHT22_INIT_DELAY_MS     250   /* Initialization stabilization delay */
#define DHT22_PULL_LOW_DELAY_MS 20    /* Pull low duration (ms) */
#define DHT22_PULL_HIGH_DELAY_US 10   /* Pull high duration (us) */
#define DHT22_INPUT_SETUP_US    1     /* Input setup time (us) */
#define DHT22_TIMEOUT_COUNT     255   /* Signal timeout counter */
#define DHT22_BIT_COUNT         40    /* Expected bit count */
#define DHT22_DATA_SIZE         6     /* Data buffer size (5 bytes + safety) */
#define DHT22_BIT_THRESHOLD     10    /* Bit detection threshold */

/* ============================================================================
 * Communication Constants
 * ============================================================================ */

/* RS-485 Modbus */
#define MODBUS_BAUD_RATE        57600
#define MODBUS_SLAVE_ID         1
#define MODBUS_BAUD_THRESHOLD   19200 /* Baud rate threshold for timing */
#define MODBUS_T15_HIGH_BAUD    750   /* 1.5 char time at high baud (us) */
#define MODBUS_T35_HIGH_BAUD    1750  /* 3.5 char time at high baud (us) */
#define MODBUS_T15_MULTIPLIER   15000000 /* 1.5 * 10^7 for low baud calculation */
#define MODBUS_T35_MULTIPLIER   35000000 /* 3.5 * 10^7 for low baud calculation */
#define MODBUS_CRC_INIT         0xFFFF
#define MODBUS_CRC_POLY         0xA001
#define MODBUS_BUFFER_SIZE      64
#define MODBUS_MIN_REQUEST_LEN  8     /* Minimum request packet size */
#define MODBUS_EXCEPT_BIT       0x80  /* Exception flag bit */
#define MODBUS_EXCEPT_RESPONSE_SIZE 5 /* Exception response size in bytes */
#define MODBUS_CRC_MASK         0xFFFF

/* I2C EEPROM */
#define I2C_EEPROM_ADDR         0x50
#define I2C_CLOCK_SPEED         100000
#define I2C_MEM_ADDR_INIT       0x0010
#define I2C_TIMEOUT_MS          100

/* ============================================================================
 * Time Constants
 * ============================================================================ */

/* RTC Time Ranges */
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

/* Default RTC values */
#define RTC_DEFAULT_HOUR        12
#define RTC_DEFAULT_MINUTE      0
#define RTC_DEFAULT_SECOND      0
#define RTC_DEFAULT_DAY         1
#define RTC_DEFAULT_MONTH       1  /* January */
#define RTC_DEFAULT_YEAR        25

/* FreeRTOS Tick Rate */
#define FREERTOS_TICK_RATE_HZ   1000

/* Task stack sizes */
#define TASK_STACK_DEFAULT      128   /* Default task stack (words) */
#define TASK_STACK_SENSOR       128   /* Sensor reading task stack */
#define TASK_STACK_DISPLAY      800   /* Display task stack */

/* Task delays */
#define DELAY_INIT_MS           900
#define DELAY_SENSOR_INIT_MS    1000
#define DELAY_BUTTON_DEBOUNCE_MS 100
#define DELAY_EDIT_EXIT_MS      200
#define DELAY_TIME_UPDATE_MS    900
#define DELAY_TIME_BASE_MS      1000

/* ============================================================================
 * Timer Constants
 * ============================================================================ */

/* TIM4 (Modbus timing) */
#define TIM4_PRESCALER          71    /* 72MHz / (71+1) = 1MHz */
#define TIM4_PERIOD             1562  /* ~1.56ms period */

/* TIM3 (PWM) */
#define TIM3_PRESCALER          20

/* TIM2 (Button debounce) */
#define TIM2_MAX_COUNT          65535

/* ============================================================================
 * Menu System Constants
 * ============================================================================ */

#define MENU_NAME_STR_LEN       17    /* Characters in menu name strings */
#define MENU_VALUE_STR_LEN      7     /* Characters in value strings */
#define MENU_TITLE_STR_LEN      13    /* Characters in title strings */

/* Menu item types */
#define MENU_TYPE_NONE          0
#define MENU_TYPE_ONOFF         1
#define MENU_TYPE_INT           2
#define MENU_TYPE_FLOAT         3
#define MENU_TYPE_TIME          4

/* Relay count */
#define RELAY_COUNT             6

/* ============================================================================
 * Data Buffer Sizes
 * ============================================================================ */

#define UART_BUFFER_SIZE        32
#define EEPROM_DATA_SIZE        32

/* ============================================================================
 * Error Codes
 * ============================================================================ */

#define ERROR_NULL              0x00000000
#define ERROR_PRESS_MET_OUT_OF_VOLT           0x00000001
#define ERROR_PRESS_MET_OUT_OF_BAR_PER_SECOND 0x00000002
#define ERROR_WIRE_BREAK        0x00000004

/* ============================================================================
 * Miscellaneous
 * ============================================================================ */

/* Temperature/Float display ranges */
#define FLOAT_TEMP_MAX          100.0f
#define FLOAT_TEMP_MIN          -50.0f
#define FLOAT_SCALE_FACTOR      100   /* Scale factor for float to int conversion */

/* String conversion */
#define STR_BUF_SIZE_6          6     /* 6-character string buffers */
#define STR_NULL_TERMINATOR     0x00

/* Encoder states */
#define ENCODER_DIR_CW          0x10
#define ENCODER_DIR_CCW         0x20
#define ENCODER_STATE_MASK      0x0F

/* EXTI line numbers (just the bit position, not the HAL macro) */
#define EXTI_LINE_10_BIT        10
#define EXTI_LINE_11_BIT        11
#define EXTI_LINE_12_BIT        12

/* NVIC priorities */
#define NVIC_PRIORITY_5         5
#define NVIC_PRIORITY_15        15

/* ADC Channels */
#define ADC_CHANNEL_TR_1        1
#define ADC_CHANNEL_TR_2        2
#define ADC_CHANNEL_TR_3        3
#define ADC_CHANNEL_TR_4        4
#define ADC_CHANNEL_PM_1        8
#define ADC_CHANNEL_PM_2        9

/* Voltage divider factors */
#define VOLT_DIV_3000_4095      3000  /* Thermistor calculation factor */
#define VOLT_DIV_4095           4095  /* ADC resolution factor */
#define VOLT_DIV_4095_DIV       4095  /* For pressure calculation */

/* UART/Communication */
#define UART_TRANSMIT_TIMEOUT   10    /* UART transmit timeout in ms */


#endif /* INC_CONSTANTS_H_ */
