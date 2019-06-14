#include "asf.h"
#include "main.h"
#include <string.h>
#include "bsp/include/nm_bsp.h"
#include "driver/include/m2m_wifi.h"
#include "socket/include/socket.h"
#include "bme280.h"

#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- WINC1500 weather client example --"STRING_EOL	\
"-- "BOARD_NAME " --"STRING_EOL	\
"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

#define LED_PIO_ID		  ID_PIOC
#define LED_PIO         PIOC
#define LED_PIN		      8
#define LED_PIN_MASK    (1<<LED_PIN)

#define BUT_PIO_ID            ID_PIOA
#define BUT_PIO               PIOA
#define BUT_PIN		          11
#define BUT_PIN_MASK          (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79

#define CO2_PIO_ID            ID_PIOA
#define CO2_PIO               PIOA
#define CO2_PIN		          24
#define CO2_PIN_MASK          (1 << CO2_PIN)

#define MOLHADO_PIO_ID        ID_PIOA
#define MOLHADO_PIO           PIOA
#define MOLHADO_PIN		      2
#define MOLHADO_PIN_MASK      (1 << MOLHADO_PIN)

#define PRESENCA_PIO_ID       ID_PIOD
#define PRESENCA_PIO          PIOD
#define PRESENCA_PIN		  26
#define PRESENCA_PIN_MASK     (1 << PRESENCA_PIN)

#define BUZ_PIO_ID        ID_PIOA
#define BUZ_PIO           PIOA
#define BUZ_PIN       6
#define BUZ_PIN_MASK  (1u << BUZ_PIN)

#define TWIHS_MCU6050_ID    ID_TWIHS0
#define TWIHS_MCU6050       TWIHS0


#define TASK_WIFI_STACK_SIZE            (2*4096/sizeof(portSTACK_TYPE))
#define TASK_WIFI_STACK_PRIORITY        (tskIDLE_PRIORITY)


#define TASK_LCD_STACK_SIZE            (10*1024/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_GENERICO_STACK_SIZE            (2*2048/sizeof(portSTACK_TYPE))
#define TASK_GENERICO_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define YEAR 0
#define MOUNTH 0
#define DAY 0
#define WEEK 0
#define HOUR 0
#define MINUTE 0
#define SECOND 0