#include "includes.h"

/************************************************************************/
/* VAR globais                                                          */
/************************************************************************/

/** IP address of host. */
uint32_t gu32HostIp = 0;

/** TCP client socket handlers. */
static SOCKET tcp_client_socket = -1;

/** Receive buffer definition. */
static uint8_t gau8ReceivedBuffer[MAIN_WIFI_M2M_BUFFER_SIZE] = {0};

/** Wi-Fi status variable. */
static bool gbConnectedWifi = false;

/** Get host IP status variable. */
/** Wi-Fi connection state */
static uint8_t wifi_connected;

/** Instance of HTTP client module. */
static bool gbHostIpByName = false;

/** TCP Connection status variable. */
static bool gbTcpConnection = false;

/** Server host name. */
static char server_host_name[] = MAIN_SERVER_NAME;

float rangePerDigit ; // 2G
//const float rangePerDigit = 9.80665f ; // 2G

volatile uint8_t flag_led0 = 1;

int16_t  accX, accY, accZ;
volatile uint8_t  accXHigh, accYHigh, accZHigh;
volatile uint8_t  accXLow,  accYLow,  accZLow;

/* Canal do sensor de temperatura */
#define AFEC_CHANNEL_TEMP_SENSOR 0

QueueHandle_t xQueueSDCard;
QueueHandle_t xQueueTemp;
QueueHandle_t xQueuePress;
QueueHandle_t xQueueUmi;
QueueHandle_t xQueueCo;
QueueHandle_t xQueuePresen;
QueueHandle_t xQueueMolhado;
QueueHandle_t xQueueBuz;
QueueHandle_t xQueueICo;

SemaphoreHandle_t xSemaphoreRTC;


/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void BUT_init(void);
void LED_init(int estado);
//void pin_toggle(Pio *pio, uint32_t mask);
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);
static void resolve_cb(uint8_t *hostName, uint32_t hostIp);
static void wifi_cb(uint8_t u8MsgType, void *pvMsg);
static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg);
static void Button1_Handler(uint32_t id, uint32_t mask);
static void AFEC_Temp_callback(void);
void RTC_Handler(void);
static void configure_console(void);
int inet_aton(const char *cp, in_addr *ap);
static void set_dev_name_to_mac(uint8_t *name, uint8_t *mac_addr);
uint8_t bme280_i2c_read_reg(uint CHIP_ADDRESS, uint reg_address, char *value);
int8_t bme280_i2c_config_temp(void);
int8_t bme280_i2c_read_temp(uint *temp);
int8_t bme280_i2c_read_umi(uint *umi);
int8_t bme280_i2c_read_press(uint *press);
uint8_t bme280_validate_id(void);
static int32_t convert_adc_to_temp(int32_t ADC_value);
static void config_ADC_TEMP(void);
void bme280_i2c_bus_init(void);
void RTC_init();
static void task_monitor(void *pvParameters);
static void task_wifi(void *pvParameters);
static void task_bme(void *pvParameters);
static void task_presenca(void *pvParameters);
static void task_co2(void *pvParameters);
static void task_molhado(void *pvParameters);
void task_adc(void);
void task_sd_card(void);
void CO2_init(void);
void MOLHADO_init(void);
void PRESENCA_init(void);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/


extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	* identify which task has overflowed its stack.
	*/
	for (;;) {
	}
}

extern void vApplicationIdleHook(void)
{
	
}

extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

static void resolve_cb(uint8_t *hostName, uint32_t hostIp)
{
	gu32HostIp = hostIp;
	gbHostIpByName = true;
	printf("resolve_cb: %s IP address is %d.%d.%d.%d\r\n\r\n", hostName,
	(int)IPV4_BYTE(hostIp, 0), (int)IPV4_BYTE(hostIp, 1),
	(int)IPV4_BYTE(hostIp, 2), (int)IPV4_BYTE(hostIp, 3));
}

static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
		case M2M_WIFI_RESP_CON_STATE_CHANGED:
		{
			tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
			if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
				printf("wifi_cb: M2M_WIFI_CONNECTED\r\n");
				m2m_wifi_request_dhcp_client();
				} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
				printf("wifi_cb: M2M_WIFI_DISCONNECTED\r\n");
				gbConnectedWifi = false;
				wifi_connected = 0;
			}

			break;
		}

		case M2M_WIFI_REQ_DHCP_CONF:
		{
			uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
			printf("wifi_cb: IP address is %u.%u.%u.%u\r\n",
			pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
			wifi_connected = M2M_WIFI_CONNECTED;
			
			/* Obtain the IP Address by network name */
			//gethostbyname((uint8_t *)server_host_name);
			break;
		}

		default:
		{
			break;
		}
	}
}

static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	/* Check for socket event on TCP socket. */
	int flag_temp = 0;
	if (sock == tcp_client_socket) {
		
		
		switch (u8Msg) {
			case SOCKET_MSG_CONNECT:
			{
				//printf("socket_msg_connect\n");
				if (gbTcpConnection) {
					memset(gau8ReceivedBuffer, 0, sizeof(gau8ReceivedBuffer));
					

					tstrSocketConnectMsg *pstrConnect = (tstrSocketConnectMsg *)pvMsg;
					if (pstrConnect && pstrConnect->s8Error >= SOCK_ERR_NO_ERROR) {
						
						//printf("send \n");
						if (xQueueReceive( xQueueTemp, &flag_temp, ( TickType_t )  100 / portTICK_PERIOD_MS)){
							sprintf((char *)gau8ReceivedBuffer, "%s", TEMP_PREFIX_BUFFER);
							send(tcp_client_socket, gau8ReceivedBuffer, strlen((char *)gau8ReceivedBuffer), 0);

							memset(gau8ReceivedBuffer, 0, MAIN_WIFI_M2M_BUFFER_SIZE);
							recv(tcp_client_socket, &gau8ReceivedBuffer[0], MAIN_WIFI_M2M_BUFFER_SIZE, 0);
						}
						else if (xQueueReceive( xQueueICo, &flag_temp, ( TickType_t )  100 / portTICK_PERIOD_MS)){
							sprintf((char *)gau8ReceivedBuffer, "%s", CO_PREFIX_BUFFER);
							send(tcp_client_socket, gau8ReceivedBuffer, strlen((char *)gau8ReceivedBuffer), 0);

							memset(gau8ReceivedBuffer, 0, MAIN_WIFI_M2M_BUFFER_SIZE);
							recv(tcp_client_socket, &gau8ReceivedBuffer[0], MAIN_WIFI_M2M_BUFFER_SIZE, 0);
						}
						else{
							printf("Nada a enviar!\n");
							gbTcpConnection = false;
							close(tcp_client_socket);
							tcp_client_socket = -1;
						}
						
					}
					else {
						//printf("socket_cb: connect error!\r\n");
						gbTcpConnection = false;
						close(tcp_client_socket);
						tcp_client_socket = -1;
					}
				}
			}
			break;
			


			case SOCKET_MSG_RECV:
			{
				char *pcIndxPtr;
				char *pcEndPtr;

				tstrSocketRecvMsg *pstrRecv = (tstrSocketRecvMsg *)pvMsg;
				if (pstrRecv && pstrRecv->s16BufferSize > 0) {
					printf(pstrRecv->pu8Buffer);
					
					char *ponteiro = strstr(pstrRecv->pu8Buffer, "nome");
					
					
					
					memset(gau8ReceivedBuffer, 0, sizeof(gau8ReceivedBuffer));
					recv(tcp_client_socket, &gau8ReceivedBuffer[0], MAIN_WIFI_M2M_BUFFER_SIZE, 0);
					} else {
					printf("socket_cb: recv error!\r\n");
					close(tcp_client_socket);
					tcp_client_socket = -1;
				}
			}
			break;

			default:
			break;
		}
	}
}

static void Button1_Handler(uint32_t id, uint32_t mask)
{
	int flage = 0;
	BaseType_t xHigherPriorityTaskWoken;

	// We have not woken a task at the start of the ISR.
	xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(xQueueBuz, &flage, &xHigherPriorityTaskWoken);
}

static void CO2_Handler(uint32_t id, uint32_t mask)
{
	printf("CO2 Handler");
}

static void PRESENCA_Handler(uint32_t id, uint32_t mask)
{
	printf("Presença Handler");
}

static void MOLHADO_Handler(uint32_t id, uint32_t mask)
{
	printf("Chão molhado Handler");
}

static void AFEC_Temp_callback(void)
{
	int32_t adcVal;
	adcVal = afec_channel_get_value(AFEC0, AFEC_CHANNEL_TEMP_SENSOR);
	xQueueSendFromISR( xQueueCo, &adcVal, 0);
	//
	//g_ul_value = afec_channel_get_value(AFEC0, AFEC_CHANNEL_TEMP_SENSOR);
	//g_is_conversion_done = true;
	//printf("converteu");
}
/*
void RTC_Handler(void) {
	uint32_t ul_status = rtc_get_status(RTC);
	uint16_t hour;
	uint16_t m;
	uint16_t se;

	//INTERRUP??O POR SEGUNDO
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		xSemaphoreGiveFromISR(xSemaphoreRTC, NULL);
	}

	//INTERRUP??O POR ALARME
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {

		rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	}

	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}
*/
/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void)
{
	
	/* Configura USART1 Pinos */
	sysclk_enable_peripheral_clock(ID_PIOB);
	sysclk_enable_peripheral_clock(ID_PIOA);
	pio_set_peripheral(PIOB, PIO_PERIPH_D, PIO_PB4);  // RX
	pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA21); // TX
	MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO4;
	
	const usart_serial_options_t uart_serial_options = {
		.baudrate =		CONF_UART_BAUDRATE,
		.charlength =	CONF_UART_CHAR_LENGTH,
		.paritytype =	CONF_UART_PARITY,
		.stopbits =		CONF_UART_STOP_BITS,
	};

	/* Configure UART console. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

/* http://www.cs.cmu.edu/afs/cs/academic/class/15213-f00/unpv12e/libfree/inet_aton.c */
int inet_aton(const char *cp, in_addr *ap)
{
	int dots = 0;
	register u_long acc = 0, addr = 0;

	do {
		register char cc = *cp;

		switch (cc) {
			case '0':
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
			case '7':
			case '8':
			case '9':
			acc = acc * 10 + (cc - '0');
			break;

			case '.':
			if (++dots > 3) {
				return 0;
			}
			/* Fall through */

			case '\0':
			if (acc > 255) {
				return 0;
			}
			addr = addr << 8 | acc;
			acc = 0;
			break;

			default:
			return 0;
		}
	} while (*cp++) ;

	/* Normalize the address */
	if (dots < 3) {
		addr <<= 8 * (3 - dots) ;
	}

	/* Store it if requested */
	if (ap) {
		ap->s_addr = _htonl(addr);
	}

	return 1;
}

static void set_dev_name_to_mac(uint8_t *name, uint8_t *mac_addr)
{
	/* Name must be in the format WINC1500_00:00 */
	uint16 len;

	len = m2m_strlen(name);
	if (len >= 5) {
		name[len - 1] = MAIN_HEX2ASCII((mac_addr[5] >> 0) & 0x0f);
		name[len - 2] = MAIN_HEX2ASCII((mac_addr[5] >> 4) & 0x0f);
		name[len - 4] = MAIN_HEX2ASCII((mac_addr[4] >> 0) & 0x0f);
		name[len - 5] = MAIN_HEX2ASCII((mac_addr[4] >> 4) & 0x0f);
	}
}

void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

uint8_t bme280_i2c_read_reg(uint CHIP_ADDRESS, uint reg_address, char *value){
	uint i = 1;
	
	twihs_packet_t p_packet;
	p_packet.chip         = CHIP_ADDRESS;//BME280_ADDRESS;
	p_packet.addr_length  = 0;

	char data = reg_address; //BME280_CHIP_ID_REG;
	p_packet.buffer       = &data;
	p_packet.length       = 1;
	
	if(twihs_master_write(TWIHS_MCU6050, &p_packet) != TWIHS_SUCCESS)
	return 1;

	p_packet.addr_length  = 0;
	p_packet.length       = 1;
	p_packet.buffer       = value;

	if(twihs_master_read(TWIHS_MCU6050, &p_packet) != TWIHS_SUCCESS)
	return 1;
	
	return 0;
}

int8_t bme280_i2c_config_temp(void){
	int32_t ierror = 0x00;
	
	twihs_packet_t p_packet;
	p_packet.chip         = BME280_ADDRESS;//BME280_ADDRESS;
	p_packet.addr[0]      = BME280_CTRL_MEAS_REG;
	p_packet.addr_length  = 1;

	char data = 0b00100111; //BME280_CHIP_ID_REG;
	p_packet.buffer       = &data;
	p_packet.length       = 1;
	
	if(twihs_master_write(TWIHS_MCU6050, &p_packet) != TWIHS_SUCCESS){
		return 1;
	}
	
	delay_ms(10);
	
	twihs_packet_t p_packet2;
	p_packet2.chip         = BME280_ADDRESS;//BME280_ADDRESS;
	p_packet2.addr[0]      = BME280_CTRL_HUMIDITY_REG;
	p_packet2.addr_length  = 1;

	char data2 = 0b00000001; //BME280_CHIP_ID_REG;
	p_packet2.buffer       = &data2;
	p_packet2.length       = 1;
	
	if(twihs_master_write(TWIHS_MCU6050, &p_packet2) != TWIHS_SUCCESS){
		return 1;
	}
	
	return 0;
}

int8_t bme280_i2c_read_temp(uint *temp)
{
	int32_t ierror = 0x00;
	char tmp[3];
	
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_TEMPERATURE_MSB_REG, &tmp[2]);
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_TEMPERATURE_MSB_REG, &tmp[2]);
	
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_TEMPERATURE_LSB_REG, &tmp[1]);
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_TEMPERATURE_LSB_REG, &tmp[1]);

	*temp = tmp[2] << 8 | tmp[1];
	return 0;
}

int8_t bme280_i2c_read_umi(uint *umi)
{
	int32_t ierror = 0x00;
	char tmp[3];
	
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_HUMIDITY_MSB_REG, &tmp[2]);
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_HUMIDITY_MSB_REG, &tmp[2]);
	
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_HUMIDITY_LSB_REG, &tmp[1]);
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_HUMIDITY_LSB_REG, &tmp[1]);

	*umi = tmp[2] << 8 | tmp[1];
	return 0;
}

int8_t bme280_i2c_read_press(uint *press)
{
	int32_t ierror = 0x00;
	char tmp[3];
	
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_PRESSURE_MSB_REG, &tmp[2]);
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_PRESSURE_MSB_REG, &tmp[2]);
	
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_PRESSURE_LSB_REG, &tmp[1]);
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_PRESSURE_LSB_REG, &tmp[1]);

	*press = tmp[2] << 8 | tmp[1];
	return 0;
}

uint8_t bme280_validate_id(void){
	char id;
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_CHIP_ID_REG, &id );
	if (bme280_i2c_read_reg(BME280_ADDRESS, BME280_CHIP_ID_REG, &id ))
	return 1;
	if (id != 0x60)
	return 1;
	return 0;
}

/**
* converte valor lido do ADC para temperatura em graus celsius
* input : ADC reg value
* output: Temperature in celsius
*/
static int32_t convert_adc_to_temp(int32_t ADC_value){

	int32_t ul_vol;
	int32_t ul_temp;

	/*
	* converte bits -> tens?o (Volts)
	*/
	ul_vol = ADC_value * 100 / 4096;

	/*
	* According to datasheet, The output voltage VT = 0.72V at 27C
	* and the temperature slope dVT/dT = 2.33 mV/C
	*/
	return(ul_vol);
}

static void config_ADC_TEMP(void){
	afec_enable(AFEC0);

	/* struct de configuracao do AFEC */
	struct afec_config afec_cfg;

	/* Carrega parametros padrao */
	afec_get_config_defaults(&afec_cfg);

	/* Configura AFEC */
	afec_init(AFEC0, &afec_cfg);

	/* Configura trigger por software */
	afec_set_trigger(AFEC0, AFEC_TRIG_SW);

	/* configura call back */
	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_0,	AFEC_Temp_callback, 7);

	/*** Configuracao espec?fica do canal AFEC ***/
	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(AFEC0, AFEC_CHANNEL_TEMP_SENSOR, &afec_ch_cfg);

	/*
	* Calibracao:
	* Because the internal ADC offset is 0x200, it should cancel it and shift
	down to 0.
	*/
	afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL_TEMP_SENSOR, 0x200);

	/***  Configura sensor de temperatura ***/
	struct afec_temp_sensor_config afec_temp_sensor_cfg;

	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
	afec_temp_sensor_set_config(AFEC0, &afec_temp_sensor_cfg);

	/* Selecina canal e inicializa convers?o */
	afec_channel_enable(AFEC0, AFEC_CHANNEL_TEMP_SENSOR);
}

int8_t bme280_i2c_read_compensation_T(ushort *temp, uint reg1, uint reg2)
{
	
	int32_t ierror = 0x00;
	char tmp[3];
	
	bme280_i2c_read_reg(BME280_ADDRESS, reg2, &tmp[2]);
	bme280_i2c_read_reg(BME280_ADDRESS, reg2, &tmp[2]);
	
	bme280_i2c_read_reg(BME280_ADDRESS, reg1, &tmp[1]);
	bme280_i2c_read_reg(BME280_ADDRESS, reg1, &tmp[1]);

	*temp = tmp[2] << 8 | tmp[1];
	return 0;
}

int8_t bme280_i2c_read_compensation_P(ushort *press, uint reg1, uint reg2)
{
	
	int32_t ierror = 0x00;
	char tmp[3];
	
	bme280_i2c_read_reg(BME280_ADDRESS, reg2, &tmp[2]);
	bme280_i2c_read_reg(BME280_ADDRESS, reg2, &tmp[2]);
	
	bme280_i2c_read_reg(BME280_ADDRESS, reg1, &tmp[1]);
	bme280_i2c_read_reg(BME280_ADDRESS, reg1, &tmp[1]);

	*press = tmp[2] << 8 | tmp[1];
	return 0;
}

int8_t bme280_i2c_read_compensation_H(ushort *umi, uint reg1, uint reg2)
{
	
	int32_t ierror = 0x00;
	char tmp[3];
	
	bme280_i2c_read_reg(BME280_ADDRESS, reg2, &tmp[2]);
	bme280_i2c_read_reg(BME280_ADDRESS, reg2, &tmp[2]);
	
	bme280_i2c_read_reg(BME280_ADDRESS, reg1, &tmp[1]);
	bme280_i2c_read_reg(BME280_ADDRESS, reg1, &tmp[1]);

	*umi = tmp[2] << 8 | tmp[1];
	return 0;
}

int8_t bme280_i2c_read_compensation_H2(ushort *umi, uint reg1, uint reg2)
{
	
	int32_t ierror = 0x00;
	char tmp[3];
	
	bme280_i2c_read_reg(BME280_ADDRESS, reg2, &tmp[2]);
	bme280_i2c_read_reg(BME280_ADDRESS, reg2, &tmp[2]);
	
	bme280_i2c_read_reg(BME280_ADDRESS, reg1, &tmp[1]);
	bme280_i2c_read_reg(BME280_ADDRESS, reg1, &tmp[1]);

	*umi = tmp[2] << 4 | (tmp[1] << 12) >> 12;
	return 0;
}

int8_t bme280_i2c_read_compensation_H_lower(uint8_t *umi, uint reg)
{
	
	int32_t ierror = 0x00;
	uint8_t out;
	
	bme280_i2c_read_reg(BME280_ADDRESS, reg, &out);
	bme280_i2c_read_reg(BME280_ADDRESS, reg, &out);
	
	*umi = out;
	return 0;
}

int32_t t_fine;
int32_t BME280_compensate_T_int32(int32_t adc_T, ushort dig_T1, short dig_T2, short dig_T3)
{
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) *
	((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

uint32_t BME280_compensate_P_int64(int32_t adc_P, ushort dig_P1, short dig_P2, short dig_P3, short dig_P4, short dig_P5, short dig_P6, short dig_P7, short dig_P8, short dig_P9)
{
	int64_t var1, var2, p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
	var2 = var2 + (((int64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
	return (uint32_t)p;
}

uint32_t bme280_compensate_H_int32(int32_t adc_H, uint8_t dig_H1, short dig_H2, uint8_t dig_H3, short dig_H4, short dig_H5, int8_t dig_H6)
{
	int32_t v_x1_u32r;
	v_x1_u32r = (t_fine - ((int32_t)76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) +
	((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r *
	((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
	((int32_t)dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	return (uint32_t)(v_x1_u32r>>12);
}

/************************************************************************/
/* inits                                                                */
/************************************************************************/

void BUT_init(void){
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(BUT_PIO_ID);
	pio_set_input(BUT_PIO, BUT_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	/* config. interrupcao em borda de descida no botao do kit */
	/* indica funcao (but_Handler) a ser chamada quando houver uma interrup��o */
	
	
	
	/* habilita interrup�c�o do PIO que controla o botao */
	/* e configura sua prioridade                        */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 6);
	NVIC_ClearPendingIRQ(BUT_PIO_ID);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIN_MASK, PIO_IT_FALL_EDGE, Button1_Handler);

pio_enable_interrupt(BUT_PIO, BUT_PIN_MASK);

};


void CO2_init(void){
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(CO2_PIO_ID);
	pio_set_input(CO2_PIO, CO2_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	/* config. interrupcao em borda de descida no botao do kit */
	/* indica funcao (CO2_Handler) a ser chamada quando houver uma interrup��o */
	pio_enable_interrupt(CO2_PIO, CO2_PIN_MASK);
	pio_handler_set(CO2_PIO, CO2_PIO_ID, CO2_PIN_MASK, PIO_IT_FALL_EDGE, CO2_Handler);
	
	/* habilita interrup�c�o do PIO que controla o botao */
	/* e configura sua prioridade                        */
	NVIC_EnableIRQ(CO2_PIO_ID);
	NVIC_SetPriority(CO2_PIO_ID, 1);
};

void MOLHADO_init(void){
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(MOLHADO_PIO_ID);
	pio_set_input(MOLHADO_PIO, MOLHADO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	/* config. interrupcao em borda de descida no botao do kit */
	/* indica funcao (MOLHADO_Handler) a ser chamada quando houver uma interrup��o */
	pio_enable_interrupt(MOLHADO_PIO, MOLHADO_PIN_MASK);
	pio_handler_set(MOLHADO_PIO, MOLHADO_PIO_ID, MOLHADO_PIN_MASK, PIO_IT_FALL_EDGE, MOLHADO_Handler);
	
	/* habilita interrup�c�o do PIO que controla o botao */
	/* e configura sua prioridade                        */
	NVIC_EnableIRQ(MOLHADO_PIO_ID);
	NVIC_SetPriority(MOLHADO_PIO_ID, 1);
};

void PRESENCA_init(void){
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(PRESENCA_PIO_ID);
	pio_set_input(PRESENCA_PIO, PRESENCA_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	/* config. interrupcao em borda de descida no botao do kit */
	/* indica funcao (PRESENCA_Handler) a ser chamada quando houver uma interrup��o */
	pio_enable_interrupt(PRESENCA_PIO, PRESENCA_PIN_MASK);
	pio_handler_set(PRESENCA_PIO, PRESENCA_PIO_ID, PRESENCA_PIN_MASK, PIO_IT_FALL_EDGE, PRESENCA_Handler);
	
	/* habilita interrup�c�o do PIO que controla o botao */
	/* e configura sua prioridade                        */
	NVIC_EnableIRQ(PRESENCA_PIO_ID);
	NVIC_SetPriority(PRESENCA_PIO_ID, 1);
};

void LED_init(int estado){
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_set_output(LED_PIO, LED_PIN_MASK, estado, 0, 0 );
};

void bme280_i2c_bus_init(void)
{
	twihs_options_t bno055_option;
	pmc_enable_periph_clk(TWIHS_MCU6050_ID);

	/* Configure the options of TWI driver */
	bno055_option.master_clk = sysclk_get_cpu_hz();
	bno055_option.speed      = 10000;
	twihs_master_init(TWIHS_MCU6050, &bno055_option);
}

void RTC_init() {
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(RTC, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(RTC, YEAR, MOUNTH, DAY, WEEK);
	rtc_set_time(RTC, HOUR, MINUTE, SECOND);
	

	/* Configure RTC interrupts */
	//NVIC_DisableIRQ(RTC_IRQn);
	//NVIC_ClearPendingIRQ(RTC_IRQn);
	//NVIC_SetPriority(RTC_IRQn, 5);
	//NVIC_EnableIRQ(RTC_IRQn);

	/* Ativa interrupcao via alarme */
	//	rtc_enable_interrupt(RTC, RTC_IER_ALREN);
	//rtc_enable_interrupt(RTC, RTC_IER_SECEN);
}

void BUZ_init(void){
	pmc_enable_periph_clk(BUZ_PIO_ID);
	pio_set_output(BUZ_PIO, BUZ_PIN_MASK, 0, 0, 0);
}
/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_monitor(void *pvParameters)
{
	static portCHAR szList[256];
	UNUSED(pvParameters);

	for (;;) {
		printf("--- task ## %u", (unsigned int)uxTaskGetNumberOfTasks());
		vTaskList((signed portCHAR *)szList);
		printf(szList);
		vTaskDelay(1000);
	}
}

static void task_wifi(void *pvParameters) {
	tstrWifiInitParam param;
	int8_t ret;
	uint8_t mac_addr[6];
	uint8_t u8IsMacAddrValid;
	struct sockaddr_in addr_in;
	
	/* Initialize the BSP. */
	nm_bsp_init();
	
	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

	/* Initialize Wi-Fi driver with data and status callbacks. */
	param.pfAppWifiCb = wifi_cb;
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) {
		//printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
		while (1) {
		}
	}
	
	/* Initialize socket module. */
	socketInit();

	/* Register socket callback function. */
	registerSocketCallback(socket_cb, resolve_cb);

	/* Connect to router. */
	printf("main: connecting to WiFi AP %s...\r\n", (char *)MAIN_WLAN_SSID);
	m2m_wifi_set_mac_address(gau8MacAddr);
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);

	addr_in.sin_family = AF_INET;
	addr_in.sin_port = _htons(MAIN_SERVER_PORT);
	inet_aton(MAIN_SERVER_NAME, &addr_in.sin_addr);
	printf("Inet aton : %d", addr_in.sin_addr);
	
	while(1){
		m2m_wifi_handle_events(NULL);

		if (wifi_connected == M2M_WIFI_CONNECTED) {
			/* Open client socket. */
			if (tcp_client_socket < 0) {
				//printf("socket init \n");
				if ((tcp_client_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
					//printf("main: failed to create TCP client socket error!\r\n");
					continue;
				}

				/* Connect server */
				//printf("socket connecting\n\n");
				
				if (connect(tcp_client_socket, (struct sockaddr *)&addr_in, sizeof(struct sockaddr_in)) != SOCK_ERR_NO_ERROR) {
					close(tcp_client_socket);
					tcp_client_socket = -1;
					//printf("error\n");
				}else{
					//printf("CONECTADO no socket");
					gbTcpConnection = true;
				}
			}
		}
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
}

static void task_bme(void *pvParameters){
	//PINO PA6
	
	bme280_i2c_bus_init();
	
	Bool validado = false;
	uint temperatura, pressao, umidade;
	int32_t dig_T1, dig_T2, dig_T3;
	int32_t dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
	int32_t dig_H1, dig_H2, dig_H3, dig_H4, dig_H5, dig_H6;
	
	uint8_t second;
	uint8_t minute;
	uint8_t hour;
	data d;
	char clock_buffer[100];
	
	while(1){
		if(bme280_validate_id()){
			printf("Chip nao encontrado\n");
			validado = false;
		}
		else if (!validado){
			validado = true;
			bme280_i2c_config_temp();
			
			bme280_i2c_read_compensation_T(&dig_T1, BME280_DIG_T1_LSB_REG, BME280_DIG_T1_MSB_REG);
			bme280_i2c_read_compensation_T(&dig_T2, BME280_DIG_T2_LSB_REG, BME280_DIG_T2_MSB_REG);
			bme280_i2c_read_compensation_T(&dig_T3, BME280_DIG_T3_LSB_REG, BME280_DIG_T3_MSB_REG);
			
			bme280_i2c_read_compensation_P(&dig_P1, BME280_DIG_P1_LSB_REG, BME280_DIG_P1_MSB_REG);
			bme280_i2c_read_compensation_P(&dig_P2, BME280_DIG_P2_LSB_REG, BME280_DIG_P2_MSB_REG);
			bme280_i2c_read_compensation_P(&dig_P3, BME280_DIG_P3_LSB_REG, BME280_DIG_P3_MSB_REG);
			bme280_i2c_read_compensation_P(&dig_P4, BME280_DIG_P4_LSB_REG, BME280_DIG_P4_MSB_REG);
			bme280_i2c_read_compensation_P(&dig_P5, BME280_DIG_P5_LSB_REG, BME280_DIG_P5_MSB_REG);
			bme280_i2c_read_compensation_P(&dig_P6, BME280_DIG_P6_LSB_REG, BME280_DIG_P6_MSB_REG);
			bme280_i2c_read_compensation_P(&dig_P7, BME280_DIG_P7_LSB_REG, BME280_DIG_P7_MSB_REG);
			bme280_i2c_read_compensation_P(&dig_P8, BME280_DIG_P8_LSB_REG, BME280_DIG_P8_MSB_REG);
			bme280_i2c_read_compensation_P(&dig_P9, BME280_DIG_P9_LSB_REG, BME280_DIG_P9_MSB_REG);
			
			bme280_i2c_read_compensation_H_lower(&dig_H1, BME280_DIG_H1_REG);
			bme280_i2c_read_compensation_H(&dig_H2, BME280_DIG_H2_LSB_REG, BME280_DIG_H2_MSB_REG);
			bme280_i2c_read_compensation_H_lower(&dig_H3, BME280_DIG_H3_REG);
			bme280_i2c_read_compensation_H(&dig_H4, BME280_DIG_H4_LSB_REG, BME280_DIG_H4_MSB_REG);
			short aux = ((dig_H4 & 0x00FF) << 4) | (((dig_H4 & 0xFF00) >> 12) & 0b1111);
			dig_H4 = aux;
			bme280_i2c_read_compensation_H(&dig_H5, BME280_DIG_H4_LSB_REG, BME280_DIG_H5_MSB_REG);
			aux = ((dig_H5 & 0xFF00) >> 4) | (dig_H5 & 0xb1111);
			dig_H5 = aux;
			bme280_i2c_read_compensation_H_lower(&dig_H6, BME280_DIG_H6_REG);
		}
		uint32_t flag = 1;
		
		uint32_t flag2 = 5;
		uint32_t flag3 = 10;


		if(validado){
			rtc_get_time(RTC, &hour, &minute, &second);
			
			sprintf(clock_buffer, "H: %02d M: %02d S: %02d", hour, minute, second);
			
			if (bme280_i2c_read_temp(&temperatura)){
				printf("erro ao ler temperatura \n");
			}
			else{
				temperatura = BME280_compensate_T_int32((int32_t)temperatura << 4, dig_T1, dig_T2, dig_T3)/100;
					
				//printf("Temperatura: %d C\n", temperatura);
				data d = {D_TYPE_TEMP, temperatura, clock_buffer};
				xQueueSend( xQueueSDCard, &d, 0);
				if(temperatura >= 32){
					xQueueSend(xQueueTemp, &flag, 0);
					xQueueSend(xQueueBuz, &flag2, 0);
				}
				if(temperatura <= 30){
					xQueueSend(xQueueBuz, &flag3, 0);
				}
			}
			
			if (bme280_i2c_read_press(&pressao)){
				printf("erro ao ler pressao \n");
			}
			else{
				pressao = BME280_compensate_P_int64((int32_t)pressao << 4, dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9);
				//printf("Pressao: %u Pa\n", pressao/256);
				
				data d = {D_TYPE_PRESSURE, pressao/256, clock_buffer};
				xQueueSend( xQueueSDCard, &d, 0);
				
			}
			
			if (bme280_i2c_read_umi(&umidade)){
				printf("erro ao ler umidade \n");
			}
			else{
				umidade = bme280_compensate_H_int32((int32_t)umidade << 4, dig_H1, dig_H2, dig_H3, dig_H4, dig_H5, dig_H6)/1024;
				//printf("Umidade: %u %%\n", umidade);
				
				data d = {D_TYPE_HUMIDITY, umidade, clock_buffer};
				xQueueSend( xQueueSDCard, &d, 0);
			}
		}
		vTaskDelay(5000/portTICK_PERIOD_MS);
	}
	
}

/*
static void task_presenca(void *pvParameters){
	xQueuePresen = xQueueCreate(10, sizeof(int32_t));
	
	PRESENCA_init();
	
	while(1){
		
		printf("task PRESENCA\n");
		vTaskDelay(5000/portTICK_PERIOD_MS);
	}
}
*/

/*
static void task_co2(void *pvParameters){
	xQueueCo = xQueueCreate(10, sizeof(int32_t));
	
	CO2_init();
	
	while(1){
		printf("task CO2\n");
		vTaskDelay(5000/portTICK_PERIOD_MS);
	}
}
*/

/*
static void task_molhado(void *pvParameters){
	xQueueMolhado = xQueueCreate(10, sizeof(int32_t));
	
	MOLHADO_init();
	
	while(1){
		printf("task molahdo\n");
		vTaskDelay(6000/portTICK_PERIOD_MS);
	}
}
*/

 void task_adc(void){
	//PINO PD30
 	
 
 	config_ADC_TEMP();
 	afec_start_software_conversion(AFEC0);
 	int32_t adcVal;
	 
	 uint8_t second2;
	 uint8_t minute2;
	 uint8_t hour2;
	 char clock_buffer2[100];
	 uint32_t flag = 1;
	 
	 uint32_t flag2 = 5;
	 uint32_t flag3 = 10;
 
 	while (true) {
 		if (xQueueReceive( xQueueCo, &(adcVal), ( TickType_t )  5000 / portTICK_PERIOD_MS)) {


			 int convertido = adcVal*2-580;
			printf("CO2: %d g/m3\n", convertido);
			rtc_get_time(RTC, &hour2, &minute2, &second2);
			
			sprintf(clock_buffer2, "H: %02d M: %02d S: %02d", hour2, minute2, second2);
			data d = {D_TYPE_SMOKE, convertido, clock_buffer2};
			afec_start_software_conversion(AFEC0);
			xQueueSend( xQueueSDCard, &d, 0);
			
			if(convertido >= 300){
				xQueueSend(xQueueICo, &flag, 0);
				xQueueSend(xQueueBuz, &flag2, 0);
			}
			if(convertido <= 280){
				xQueueSend(xQueueBuz, &flag3, 0);
			}
			
			
	 		
 		}
 		
 		vTaskDelay(5000/portTICK_PERIOD_MS);
 
 	}
 }
 
void task_sd_card(void){	

	char test_file_name[] = "0:sd_mmc_test.txt";
	char analog_file_name[] = "0:analog.txt";

	Ctrl_status status;
	FRESULT res;
	FATFS fs;
	FIL file_object;
	
	
	/* Initialize SD MMC stack */
	sd_mmc_init();

	printf("\x0C\n\r-- SD/MMC/SDIO Card Example on FatFs --\n\r");
	printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);
	

	data d;

	while(true){
		
		printf("Please plug an SD, MMC or SDIO card in slot.\n\r");

		/* Wait card present and ready */
		do {
			status = sd_mmc_test_unit_ready(0);
			if (CTRL_FAIL == status) {
				printf("Card install FAIL\n\r");
				printf("Please unplug and re-plug the card.\n\r");
				while (CTRL_NO_PRESENT != sd_mmc_check(0)) {
				}
			}
		} while (CTRL_GOOD != status);

		printf("Mount disk (f_mount)...\r\n");
		memset(&fs, 0, sizeof(FATFS));
		res = f_mount(LUN_ID_SD_MMC_0_MEM, &fs);
		if (FR_INVALID_DRIVE == res) {
			//printf("[FAIL] res %d\r\n", res);
			}else{
			printf("[Mounting SDCARD Successful]\r\n");
			break;
		}
	}
	while (true) {
		
		//printf("Opening/Creating a file (f_open)...\r\n");
		analog_file_name[0] = LUN_ID_SD_MMC_0_MEM + '0';
		res = f_open(&file_object,
		(char const *)analog_file_name,
		FA_OPEN_EXISTING | FA_WRITE);
		if (res != FR_OK) {
			//printf("[FAIL] res %d\r\n", res);
		}

		if (xQueueReceive( xQueueSDCard, &(d), ( TickType_t )  10 / portTICK_PERIOD_MS)) {
			//printf("Chegou Temp Information: %d",d.value);
			f_lseek( &file_object, file_object.fsize );

			if (0 == f_printf(&file_object, d.type, d.timestamp, d.value)) {
				f_close(&file_object);
				//printf("[FAIL]\r\n");
			}

			//printf("[OK]\r\n");
			f_close(&file_object);
		}

		vTaskDelay(1000/portTICK_PERIOD_MS);

	}
}

void task_buz(void){
	BUZ_init();
	Bool alarm = false;
	
	int flag_buz = 0;
	Bool cd = false;
	
	while (true){
		if(xQueueReceive(xQueueBuz, &flag_buz, ( TickType_t )  5000 / portTICK_PERIOD_MS)){
			if(flag_buz == 5){
				if(!alarm && !cd){
					pin_toggle(LED_PIO, LED_PIN_MASK);
					pio_set(PIOA, BUZ_PIN_MASK);
					alarm = true;
				}
			}
			if (flag_buz == 10){
				cd = false;
			}
			if(flag_buz == 0){
				if(alarm){
					pio_clear(PIOA, BUZ_PIN_MASK);
					pin_toggle(LED_PIO, LED_PIN_MASK);
					alarm = false;
					cd = true;
				}
				
			}
		}
	}
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void)
{

	/* buffer para recebimento de dados */
	uint8_t bufferRX[100];
	uint8_t bufferTX[100];
	
	uint8_t rtn;
	
	/* Initialize the board. */
	sysclk_init();
	board_init();
	
	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;

	RTC_init();
			
	/* Initialize the UART console. */
	configure_console();
	printf(STRING_HEADER);
	
	/* Configura Leds */
	LED_init(1);
	
	/* Configura os bot�es */
	xQueueBuz = xQueueCreate(10, sizeof(int));
	xQueueTemp = xQueueCreate(10, sizeof(int));
	xQueuePress = xQueueCreate(10, sizeof(int));
	xQueueUmi = xQueueCreate(10, sizeof(int));
	xQueueCo = xQueueCreate( 10, sizeof( int32_t ) );
	xQueueICo = xQueueCreate( 10, sizeof( int32_t ) );
	xQueueSDCard = xQueueCreate( 10, sizeof( data ) );
	BUT_init();
	
	if (xTaskCreate(task_wifi, "Wifi", TASK_WIFI_STACK_SIZE, NULL,TASK_WIFI_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Wifi task\r\n");
	}
	
	
	if (xTaskCreate(task_bme, "bme", TASK_WIFI_STACK_SIZE, NULL,TASK_WIFI_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create BME task\r\n");
	}
	
	/*
	if (xTaskCreate(task_molhado, "molhado", TASK_GENERICO_STACK_SIZE, NULL,TASK_GENERICO_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Molhado task\r\n");
	}
	*/
	/*
	if (xTaskCreate(task_co2, "CO2", TASK_WIFI_STACK_SIZE, NULL,TASK_WIFI_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create CO2 task\r\n");
	}
	*/
	
	/*
	if (xTaskCreate(task_presenca, "Presenca", TASK_GENERICO_STACK_SIZE, NULL,TASK_GENERICO_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Presen�a task\r\n");
	}
	*/
	
	///* Create task to handler LCD */
	
	
	if (xTaskCreate(task_adc, "adc", TASK_WIFI_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test adc task\r\n");
	}
	
	
	
	if (xTaskCreate(task_sd_card, "SSCard", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test SDCard task\r\n");
	}
	
	
	if (xTaskCreate(task_buz, "buzzer", TASK_GENERICO_STACK_SIZE, NULL, TASK_GENERICO_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test bUZZER task\r\n");
	}
	
	vTaskStartScheduler();
	
	while(1) {};
	return 0;

}
