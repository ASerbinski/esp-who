#include <lwip/sockets.h>
#include <esp_log.h>
#include <string.h>
#include <errno.h>
#include "sdkconfig.h"

#include <stdio.h>
#include "driver/uart.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include <cJSON.h>

#define COMMAND_PORT 100
#define LOGGING_PORT 101

// This is working on the "ESP32-CAM" boards:
//#define UART1_TXD 14
//#define UART1_RXD 15

// This is for the robot camera (M5CAMW?)
#define UART1_TXD 4
#define UART1_RXD 33

#define UART1_RTS (UART_PIN_NO_CHANGE)
#define UART1_CTS (UART_PIN_NO_CHANGE)

#define UART1_PORT_NUM      UART_NUM_1
#define UART1_BAUD_RATE     115200

#define BUF_SIZE (1024)

static char tag[] = "command_server";
static char logtag[] = "logging_server";

static int logsock = 0;

static intr_handle_t handle_console;
static xQueueHandle logging_queue;
#define LOGGING_QUEUE_SIZE 10

typedef struct {
	uint8_t *data;
	uint16_t len;
} logging_data_t;

/**
 * Create a listening socket.  We then wait for a client to connect.
 * Once a client has connected, we then read until there is no more data
 * and log the data read.  We then close the client socket and start
 * waiting for a new connection.
 */
void command_server() {
	struct sockaddr_in clientAddress;
	struct sockaddr_in serverAddress;

	// Create a socket that we will listen upon.
	int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (sock < 0) {
		ESP_LOGI(tag, "socket: %d %s", sock, strerror(errno));
		goto END;
	}

	// Bind our server socket to a port.
	serverAddress.sin_family = AF_INET;
	serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
	serverAddress.sin_port = htons(COMMAND_PORT);
	int rc  = bind(sock, (struct sockaddr *)&serverAddress, sizeof(serverAddress));
	if (rc < 0) {
		ESP_LOGI(tag, "bind: %d %s", rc, strerror(errno));
		goto END;
	}

	// Flag the socket as listening for new connections.
	rc = listen(sock, 5);
	if (rc < 0) {
		ESP_LOGI(tag, "listen: %d %s", rc, strerror(errno));
		goto END;
	}

	while (1) {
		ESP_LOGI(tag, "Listening on port %d", COMMAND_PORT);
		// Listen for a new client connection.
		socklen_t clientAddressLength = sizeof(clientAddress);
		int clientSock = accept(sock, (struct sockaddr *)&clientAddress, &clientAddressLength);
		if (clientSock < 0) {
			ESP_LOGI(tag, "accept: %d %s", clientSock, strerror(errno));
			goto END;
		}

		ESP_LOGI(tag, "Client Connected!");

		// We now have a new client ...
		int total =	10*1024;
		char *data = malloc(total);

		// Loop reading data.
		while(1) {
			ESP_LOGI(tag, "Read Loop.");
			ssize_t sizeRead = recv(clientSock, data, total, 0);
			if (sizeRead < 0) {
				ESP_LOGI(tag, "recv: %d %s", sizeRead, strerror(errno));
				break;
			}
			if (sizeRead == 0) {
				break;
			}
			ESP_LOGI(tag, "Read (%d): %.*s", sizeRead, sizeRead, data);
			//if (logsock > 0) send(logsock, data, sizeRead, 0);
			data[sizeRead] = 0;
			cJSON *input_json = cJSON_Parse(data);
			if (input_json != NULL){
				int dataN = 0, dataD1 = 0, dataD2 = 0;
				cJSON *fieldN = cJSON_GetObjectItemCaseSensitive(input_json, "N");
				cJSON *fieldD1= cJSON_GetObjectItemCaseSensitive(input_json, "D1");
				cJSON *fieldD2= cJSON_GetObjectItemCaseSensitive(input_json, "D2");
					
				if (cJSON_IsNumber(fieldN))  dataN  = fieldN->valueint;
				if (cJSON_IsNumber(fieldD1)) dataD1 = fieldD1->valueint;
				if (cJSON_IsNumber(fieldD2)) dataD2 = fieldD2->valueint;

				ESP_LOGI(tag, "Block: %d/%d/%d", dataN, dataD1, dataD2);
				// 100/0/0 = stop everything
				// 101/1/0 = line following
				// 101/2/0 = object avoidance mode
				// 101/3/0 = magnet? mode
				// 102/1/250 = forward
				// 102/2/250 = reverse
				// 102/3/250 = left
				// 102/4/250 = right
				// 102/9/0 = stop
				// 106/1/0 = servo-up
				// 106/2/0 = servo-down
				// 106/3/0 = servo-left
				// 106/4/0 = servo-right

				uint8_t dt[] = {0x7b, 0x0a, 0x22, 0x4e, 0x22, 0x3a, 0x31, 0x30,
						0x32, 0x2c, 0x0a, 0x22, 0x44, 0x31, 0x22, 0x3a,
						0x31, 0x2c, 0x0a, 0x22, 0x44, 0x32, 0x22, 0x3a,
						0x32, 0x35, 0x30, 0x0a, 0x7d};
				uint8_t len;
				switch (dataN){
				case 100:
					dt[8] = 0x30;
					dt[9] = 0x0a;
					dt[10] = 0x7d;
					uart_write_bytes(UART1_PORT_NUM, (const char *) dt, 11);
					break;
				case 101:
					dt[8] = 0x31;
					dt[9] = 0x2c;
					dt[10] = 0x0a;
					dt[11] = 0x22;
					dt[12] = 0x44;
					dt[13] = 0x31;
					dt[14] = 0x22;
					dt[15] = 0x3a;
					dt[17] = 0x0a;
					dt[18] = 0x7d;
					switch (dataD1){
					case 1:	
						dt[16] = 0x31;
						break;
					case 2:
						dt[16] = 0x32;
						break;
					case 3:
						dt[16] = 0x33;
					}
					uart_write_bytes(UART1_PORT_NUM, (const char *) dt, 19);
					break;
				case 102:
					len = 29;
					switch (dataD1){
					case 1:
						dt[16] = 0x31;
						break;
					case 2:
						dt[16] = 0x32;
						break;
					case 3:
						dt[16] = 0x33;
						break;
					case 4:
						dt[16] = 0x34;
						break;
					case 9:
						dt[16] = 0x39;
						dt[24] = 0x30;
						dt[25] = 0x0a;
						dt[26] = 0x7d;
						len = 27;
					}
					uart_write_bytes(UART1_PORT_NUM, (const char *) dt, len);
					break;
				case 106:
					dt[8] = 0x36;
					dt[9] = 0x2c;
					dt[10] = 0x0a;
					dt[11] = 0x22;
					dt[12] = 0x44;
					dt[13] = 0x31;
					dt[14] = 0x22;
					dt[15] = 0x3a;
					dt[17] = 0x0a;
					dt[18] = 0x7d;
					switch (dataD1){
					case 1:	
						dt[16] = 0x31;
						break;
					case 2:
						dt[16] = 0x32;
						break;
					case 3:
						dt[16] = 0x33;
						break;
					case 4:
						dt[16] = 0x34;
					}
					uart_write_bytes(UART1_PORT_NUM, (const char *) dt, 19);
					break;
				}

				cJSON_Delete(input_json);
			}
		}

		// Finished reading data.
		ESP_LOGI(tag, "Read loop completed.");
		free(data);
		close(clientSock);
	}
	END:
	ESP_LOGI(tag, "Bailing out.");
	vTaskDelete(NULL);
}

void logging_server(){
	struct sockaddr_in clientAddress;
	struct sockaddr_in serverAddress;

	// Create a socket that we will listen upon.
	int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (sock < 0) {
		ESP_LOGI(logtag, "socket: %d %s", sock, strerror(errno));
		goto END;
	}

	// Bind our server socket to a port.
	serverAddress.sin_family = AF_INET;
	serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
	serverAddress.sin_port = htons(LOGGING_PORT);
	int rc  = bind(sock, (struct sockaddr *)&serverAddress, sizeof(serverAddress));
	if (rc < 0) {
		ESP_LOGI(logtag, "bind: %d %s", rc, strerror(errno));
		goto END;
	}

	// Flag the socket as listening for new connections.
	rc = listen(sock, 5);
	if (rc < 0) {
		ESP_LOGI(logtag, "listen: %d %s", rc, strerror(errno));
		goto END;
	}

	while (1) {
		ESP_LOGI(logtag, "Listening on port %d", LOGGING_PORT);
		// Listen for a new client connection.
		socklen_t clientAddressLength = sizeof(clientAddress);
		logsock = accept(sock, (struct sockaddr *)&clientAddress, &clientAddressLength);
		if (logsock < 0) {
			ESP_LOGI(logtag, "accept: %d %s", logsock, strerror(errno));
			goto END;
		}

		ESP_LOGI(logtag, "Client Connected!");

		// We now have a new client ...
		int total =	10*1024;
		char *data = malloc(total);

		// Loop reading data.
		while(1) {
			ESP_LOGI(logtag, "Read Loop.");
			ssize_t sizeRead = recv(logsock, data, total, 0);
			if (sizeRead < 0) {
				ESP_LOGI(logtag, "recv: %d %s", sizeRead, strerror(errno));
				break;
			}
			if (sizeRead == 0) {
				break;
			}
			ESP_LOGI(logtag, "Read (%d): %.*s", sizeRead, sizeRead, data);
		}

		// Finished reading data.
		ESP_LOGI(logtag, "Read loop terminated");
		free(data);
		close(logsock);
	}
	END:
	ESP_LOGI(logtag, "Bailing out.");
	vTaskDelete(NULL);
}

void logging_task(){
	BaseType_t res;
	logging_data_t receiver;
	while (1){
		res = xQueueReceive(logging_queue, &receiver, portMAX_DELAY);
		if (res == pdPASS){
			if (logsock > 0 && receiver.len > 0) send(logsock, receiver.data, receiver.len, 0);
			free(receiver.data);
		}
	}
}

/*
 * Define UART interrupt subroutine to ackowledge interrupt
 */
static void IRAM_ATTR uart_intr_handle(void *arg){
	uint16_t rx_fifo_len;
	uint16_t i = 0;
	logging_data_t receiver;
  
	UART1.int_st.val; // read UART interrupt Status
	rx_fifo_len = UART1.status.rxfifo_cnt; // read number of bytes in UART buffer

	receiver.data = malloc(rx_fifo_len);
	receiver.len = rx_fifo_len;

	while(rx_fifo_len){
		receiver.data[i++] = UART1.fifo.rw_byte; // read all bytes
		rx_fifo_len--;
	}

	// after reading bytes from buffer clear UART interrupt status
	uart_clear_intr_status(UART1_PORT_NUM, UART_RXFIFO_FULL_INT_CLR | UART_RXFIFO_TOUT_INT_CLR);

	xQueueSend(logging_queue, &receiver, 800 / portTICK_PERIOD_MS);
}

void socket_server(){
	xTaskCreate(&command_server, "task_command_server", 1024 * 2, NULL, 10, NULL);
	xTaskCreate(&logging_server, "task_log_server", 1024 * 2, NULL, 10, NULL);

	logging_queue = xQueueCreate(LOGGING_QUEUE_SIZE, sizeof(logging_data_t));
	if (logging_queue == NULL){
		ESP_LOGE(logtag, "Failed to create logging_queue.");
	}

	xTaskCreate(&logging_task, "task_logging", 1024 * 2, NULL, 10, NULL);

	/* Configure parameters of an UART driver,
	* communication pins and install the driver */
	uart_config_t uart_config = {
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};

	ESP_ERROR_CHECK(uart_param_config(UART1_PORT_NUM, &uart_config));

	//Set UART log level
	esp_log_level_set(logtag, ESP_LOG_INFO);

	//Set UART pins (using UART0 default pins ie no changes.)
	ESP_ERROR_CHECK(uart_set_pin(UART1_PORT_NUM, UART1_TXD, UART1_RXD, UART1_RTS, UART1_CTS));

	//Install UART driver, and get the queue.
	ESP_ERROR_CHECK(uart_driver_install(UART1_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));

	// release the pre registered UART handler/subroutine
	ESP_ERROR_CHECK(uart_isr_free(UART1_PORT_NUM));

	// register new UART subroutine
	ESP_ERROR_CHECK(uart_isr_register(UART1_PORT_NUM, uart_intr_handle, NULL, ESP_INTR_FLAG_IRAM, &handle_console));

	// enable RX interrupt
	ESP_ERROR_CHECK(uart_enable_rx_intr(UART1_PORT_NUM));
}

