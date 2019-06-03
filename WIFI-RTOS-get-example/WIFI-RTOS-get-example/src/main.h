#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/include/m2m_wifi.h"

/** Wi-Fi Settings */
#define MAIN_WLAN_SSID                    "Insper_IoT" /**< Destination SSID */
#define MAIN_WLAN_AUTH                    M2M_WIFI_SEC_OPEN /**< Security manner */
#define MAIN_WLAN_PSK                     "" /**< Password for Destination SSID */

#define MAIN_M2M_DEVICE_NAME "Liu"
#define MAIN_MAC_ADDRESS {0x42, 0x00, 0x61, 0x96, 0x2E, 0x6C}
	
static uint8_t gau8MacAddr[] = MAIN_MAC_ADDRESS;
static uint8_t gacDeviceName[] = MAIN_M2M_DEVICE_NAME;

/** Using broadcast address for simplicity. */
#define MAIN_SERVER_PORT                    (5000)

/** IP address parsing. */
#define IPV4_BYTE(val, index)               ((val >> (index * 8)) & 0xFF)

/** Send buffer of TCP socket. */
#define MAIN_PREFIX_BUFFER                  "GET /data=20&temp=25&nome=liu HTTP/1.1\r\n Accept: */*\r\n\r\n"

/** Weather information provider server. */
#define MAIN_SERVER_NAME                    "juanjg.pythonanywhere.com"

/** Receive buffer size. */
#define MAIN_WIFI_M2M_BUFFER_SIZE           1400

#define MAIN_HEX2ASCII(x)                   (((x) >= 10) ? (((x) - 10) + 'A') : ((x) + '0'))


#define D_TYPE_TEMP "Timestamp: %s -> Temp: %d\n"
#define D_TYPE_HUMIDITY "Timestamp: %s -> Humidity: %d\n"
#define D_TYPE_PRESSURE "Timestamp: %s -> Pressure: %d\n"
#define D_TYPE_SMOKE "Timestamp: %s -> CO2: %d\n"
typedef struct{
	char type[50];
	int32_t value;
	char *timestamp;
} data;


#ifdef __cplusplus
}
#endif

#endif /* MAIN_H_INCLUDED */
