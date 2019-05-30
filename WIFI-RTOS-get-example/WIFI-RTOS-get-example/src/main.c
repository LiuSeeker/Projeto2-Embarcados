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
#define BUT_PIN		            11
#define BUT_PIN_MASK          (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79

#define TWIHS_MCU6050_ID    ID_TWIHS0
#define TWIHS_MCU6050       TWIHS0


#define TASK_WIFI_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_WIFI_STACK_PRIORITY        (tskIDLE_PRIORITY)


#define TASK_LCD_STACK_SIZE            (10*1024/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY        (tskIDLE_PRIORITY)


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

/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void BUT_init(void);
void LED_init(int estado);
void pin_toggle(Pio *pio, uint32_t mask);

/* Canal do sensor de temperatura */
#define AFEC_CHANNEL_TEMP_SENSOR 0
QueueHandle_t xQueueAnalog;
QueueHandle_t xQueueSDCard;


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
	if (sock == tcp_client_socket) {
		
		switch (u8Msg) {
			case SOCKET_MSG_CONNECT:
			{
				printf("socket_msg_connect\n");
				if (gbTcpConnection) {
					memset(gau8ReceivedBuffer, 0, sizeof(gau8ReceivedBuffer));
					sprintf((char *)gau8ReceivedBuffer, "%s", MAIN_PREFIX_BUFFER);

					tstrSocketConnectMsg *pstrConnect = (tstrSocketConnectMsg *)pvMsg;
					if (pstrConnect && pstrConnect->s8Error >= SOCK_ERR_NO_ERROR) {
						printf("send \n");
						send(tcp_client_socket, gau8ReceivedBuffer, strlen((char *)gau8ReceivedBuffer), 0);

						memset(gau8ReceivedBuffer, 0, MAIN_WIFI_M2M_BUFFER_SIZE);
						recv(tcp_client_socket, &gau8ReceivedBuffer[0], MAIN_WIFI_M2M_BUFFER_SIZE, 0);
					}
					else {
						printf("socket_cb: connect error!\r\n");
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
	pin_toggle(PIOD, (1<<28));
	pin_toggle(LED_PIO, LED_PIN_MASK);
}

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

/************************************************************************/
/* inits                                                                */
/************************************************************************/

void BUT_init(void){
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(BUT_PIO_ID);
	pio_set_input(BUT_PIO, BUT_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	/* config. interrupcao em borda de descida no botao do kit */
	/* indica funcao (but_Handler) a ser chamada quando houver uma interrupção */
	pio_enable_interrupt(BUT_PIO, BUT_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIN_MASK, PIO_IT_FALL_EDGE, Button1_Handler);
	
	/* habilita interrupçcão do PIO que controla o botao */
	/* e configura sua prioridade                        */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 1);
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
		printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
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
				printf("socket init \n");
				if ((tcp_client_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
					printf("main: failed to create TCP client socket error!\r\n");
					continue;
				}

				/* Connect server */
				printf("socket connecting\n");
				
				if (connect(tcp_client_socket, (struct sockaddr *)&addr_in, sizeof(struct sockaddr_in)) != SOCK_ERR_NO_ERROR) {
					close(tcp_client_socket);
					tcp_client_socket = -1;
					printf("error\n");
					}else{
					gbTcpConnection = true;
				}
			}
		}
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
}

static void task_bme(void *pvParameters){
	bme280_i2c_bus_init();
	
	Bool validado = false;
	uint temperatura, pressao, umidade;
	
	while(1){
		if(bme280_validate_id()){
			printf("Chip nao encontrado\n");
			validado = false;
		}
		else if (!validado){
			validado = true;
			bme280_i2c_config_temp();
		}
		if(validado){
			if (bme280_i2c_read_temp(&temperatura)){
				printf("erro ao ler temperatura \n");
			}
			else{
				printf("Temperatura: %d \n", temperatura);
			}
			if (bme280_i2c_read_press(&pressao)){
				printf("erro ao ler pressao \n");
			}
			else{
				printf("Pressao: %d \n", pressao);
			}
			if (bme280_i2c_read_umi(&umidade)){
				printf("erro ao ler umidade \n");
			}
			else{
				printf("Umidade: %d \n", umidade);
			}
		}
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
	
}

static void AFEC_Temp_callback(void)
{
	int32_t adcVal;
	adcVal = afec_channel_get_value(AFEC0, AFEC_CHANNEL_TEMP_SENSOR);
	xQueueSendFromISR( xQueueAnalog, &adcVal, 0);
	//
	//g_ul_value = afec_channel_get_value(AFEC0, AFEC_CHANNEL_TEMP_SENSOR);
	//g_is_conversion_done = true;
	//printf("converteu");
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
	/*************************************
	* Ativa e configura AFEC
	*************************************/
	/* Ativa AFEC - 0 */
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
	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_0,	AFEC_Temp_callback, 5);

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

void task_adc(void){
	xQueueAnalog = xQueueCreate( 10, sizeof( int32_t ) );

	config_ADC_TEMP();
	afec_start_software_conversion(AFEC0);
	int32_t adcVal;
	data d;
	int32_t tempVal;
	printf("\nentrou na task adc\n");

	while (true) {
		
		if (xQueueReceive( xQueueAnalog, &(adcVal), ( TickType_t )  10 / portTICK_PERIOD_MS)) {
			data d = {D_TYPE_TEMP, convert_adc_to_temp(adcVal), 983019283};
			afec_start_software_conversion(AFEC0);
			xQueueSend( xQueueSDCard, &d, 0);

		}

		vTaskDelay(1000/portTICK_PERIOD_MS);

	}
}
void task_sd_card(void){
	//xQueueSDCard = xQueueCreate( 10, sizeof( int32_t ) );
	xQueueSDCard = xQueueCreate( 10, sizeof( data ) );

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
			printf("[FAIL] res %d\r\n", res);
			}else{
			printf("[Mounting SDCARD Successful]\r\n");
			break;
		}
	}
	while (true) {
		
		printf("Opening/Creating a file (f_open)...\r\n");
		analog_file_name[0] = LUN_ID_SD_MMC_0_MEM + '0';
		res = f_open(&file_object,
		(char const *)analog_file_name,
		FA_OPEN_EXISTING | FA_WRITE);
		if (res != FR_OK) {
			printf("[FAIL] res %d\r\n", res);
		}

		if (xQueueReceive( xQueueSDCard, &(d), ( TickType_t )  10 / portTICK_PERIOD_MS)) {
			printf("Chegou Temp Information: %d",d.value);
			f_lseek( &file_object, file_object.fsize );

			if (0 == f_printf(&file_object, d.type, d.timestamp, d.value)) {
				f_close(&file_object);
				printf("[FAIL]\r\n");
			}

			printf("[OK]\r\n");
			f_close(&file_object);
		}

		vTaskDelay(1000/portTICK_PERIOD_MS);

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

	/* Initialize the UART console. */
	configure_console();
	printf(STRING_HEADER);
	
	/* Configura Leds */
	LED_init(1);
	
	/* Configura os botões */
	BUT_init();
	//
	//if (xTaskCreate(task_wifi, "Wifi", TASK_WIFI_STACK_SIZE, NULL,
	//TASK_WIFI_STACK_PRIORITY, NULL) != pdPASS) {
		//printf("Failed to create Wifi task\r\n");
	//}
	//
	//if (xTaskCreate(task_bme, "bme", TASK_WIFI_STACK_SIZE, NULL,
	//TASK_WIFI_STACK_PRIORITY, NULL) != pdPASS) {
		//printf("Failed to create Wifi task\r\n");
	//}

	/* Create task to handler LCD */
	if (xTaskCreate(task_adc, "adc", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test adc task\r\n");
	}
	if (xTaskCreate(task_sd_card, "SSCard", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test SDCard task\r\n");
	}
	vTaskStartScheduler();
	
	while(1) {};
	return 0;

}
