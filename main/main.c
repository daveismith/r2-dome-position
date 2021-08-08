#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_console.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"

#define HOST		HSPI_HOST

#define PIN_NUM_CIPO	25
#define PIN_NUM_COPI	-1
//23
#define PIN_NUM_CLK	19
#define PIN_NUM_CS	22

#define BUF_SIZE (128)

#define SENSOR_TICK_PERIOD	10
#define MAX_MEASURED_VALUE	4096
#define OVERFLOW_THRESHOLD_SIZE	512
#define OVERFLOW_THRESHOLD_UPPER	(MAX_MEASURED_VALUE - OVERFLOW_THRESHOLD_SIZE)
#define OVERFLOW_THRESHOLD_LOWER	(OVERFLOW_THRESHOLD_SIZE)

#define CONSOLE_UART_CHANNEL	(0)
#define CONSOLE_UART_RX_PIN	(3)
#define CONSOLE_UART_TX_PIN	(1)

#define UARTS_BAUD_RATE		(115200)

float current_rpm = 0;
int32_t setpoint = 0;

const char test_message[] = "This is an example string, if you can read this, the example is a success!";

static void driver_task(void *arg)
{
	uart_config_t uart_config = {
		.baud_rate = 9600,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_APB,
	};
	int intr_alloc_flags = 0;

	ESP_ERROR_CHECK(uart_driver_install(2, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
	ESP_ERROR_CHECK(uart_param_config(2, &uart_config));
	ESP_ERROR_CHECK(uart_set_pin(2, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

	uint8_t exit_safe_start[] = {0xAA, 0x0D, 0x03};

	uint8_t move[] = {0xAA, 0x0D, 0x05, 0x00, 0x64};
	uint8_t reverse[] = {0xAA, 0x0D, 0x06, 0x00, 0x32};
	uint8_t stop[] = {0xAA, 0x0D, 0x12, 0x20};

	uart_write_bytes(2, (const char *)exit_safe_start, sizeof(exit_safe_start));
	vTaskDelay(100 / portTICK_RATE_MS);

	while (1) {
		/*
		// Write Command
		//
		uart_write_bytes(2, (const char *)move, sizeof(move));
		vTaskDelay(1000 / portTICK_RATE_MS); 
		
		// Write Command 
		uart_write_bytes(2, (const char *)stop, sizeof(stop));
		vTaskDelay(1000 / portTICK_RATE_MS);

		// Write Command
		uart_write_bytes(2, (const char*)reverse, sizeof(reverse));
		vTaskDelay(1000 / portTICK_RATE_MS);

		// Write Command
		uart_write_bytes(2, (const char*)stop, sizeof(stop));
		vTaskDelay(1000 / portTICK_RATE_MS);
		*/

		if (setpoint == 0) {
			uart_write_bytes(2, (const char *)stop, sizeof(stop));
		} else if (setpoint < 0) {
			uint16_t val = (uint16_t)abs(setpoint);
			reverse[4] = (val >> 8) & 0xff;
			reverse[3] = val & 0xff;
			uart_write_bytes(2, (const char *)reverse, sizeof(reverse));
		} else {
			uint16_t val = (uint16_t)setpoint;
			move[4] = (val >> 8) & 0xff;
			move[3] = val & 0xff;
			uart_write_bytes(2, (const char *)move, sizeof(move));
		}
		vTaskDelay(10 / portTICK_RATE_MS);
	}
}

static void sensor_task(void *arg)
{
	esp_err_t ret;

	// Configure The SPI Port
	spi_device_handle_t spi;
	spi_bus_config_t buscfg = {
		.miso_io_num = PIN_NUM_CIPO,
		.mosi_io_num = PIN_NUM_COPI,
		.sclk_io_num = PIN_NUM_CLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 4,
		.flags = SPICOMMON_BUSFLAG_MASTER
	};
	spi_device_interface_config_t devcfg = {
		.clock_speed_hz=1*1000*1000,	// Clock out at 1 MHz
		.mode = 1,
		.spics_io_num = PIN_NUM_CS,
		.queue_size = 1
	};

	// Initialize the SPI bus
	ret = spi_bus_initialize(HOST, &buscfg, SPI_DMA_CH_AUTO);
	ESP_ERROR_CHECK(ret);

	ret = spi_bus_add_device(HOST, &devcfg, &spi);

	// SPI Main Loop
	spi_transaction_t t;
	uint16_t last_value = UINT16_MAX;
       	int32_t delta;
       	float rpm;
	for(;;) {
		memset(&t, 0, sizeof(t));
		t.length = 24;
		t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
		ret = spi_device_polling_transmit(spi, &t);	// Transceive
		assert(ret == ESP_OK);
		
		// Ignore First Bit
		uint16_t data = (t.rx_data[0] & 0x7f) << 5 | (t.rx_data[1] >> 3 & 0x1f);

	       	if (last_value == UINT16_MAX) {
			// Ignore
			delta = 0;
		} else if (last_value > OVERFLOW_THRESHOLD_UPPER && data < OVERFLOW_THRESHOLD_LOWER) {
		       // Rollover from high to low
		       // Sign Should Be negative
		       delta = (data + MAX_MEASURED_VALUE) - last_value;
		} else if (last_value < OVERFLOW_THRESHOLD_LOWER && data > OVERFLOW_THRESHOLD_UPPER) {
			// Rollover from low to high
			// Sign Should Be  Negative
			delta = -((MAX_MEASURED_VALUE - data) + last_value);
		} else {
			// Value Is Increasing or Decreasign
			delta = data - last_value;
		}

		rpm = (delta * ((1000.0 * 60) / SENSOR_TICK_PERIOD)) / MAX_MEASURED_VALUE;
		current_rpm = rpm;
		
		/*bool ocf = (t.rx_data[1] & 0x04) == 0x04;
		bool cof = (t.rx_data[1] & 0x02) == 0x02;
		bool lin = (t.rx_data[1] & 0x01) == 0x01;
		bool mag_inc = (t.rx_data[2] & 0x04) == 0x04;
		bool mag_dec = (t.rx_data[2] & 0x02) == 0x02;
		bool crc = (t.rx_data[2] & 0x01) == 0x01;
		*/
		//printf("data: %u, ocf: %u, cof: %u, lin: %u, magINC: %u, magDEC: %u, crc: %u\n", data, ocf, cof, lin, mag_inc, mag_dec, crc);
		//printf("%u, %d, %f\n", data, delta, rpm);
		last_value = data;
		vTaskDelay(SENSOR_TICK_PERIOD / portTICK_RATE_MS);
	}

}

static struct {
	struct arg_int *setpoint;
	struct arg_lit *reverse;
	struct arg_end *end;
} rpm_args;

static int rpm(int argc, char **argv)
{
	int nerrors = arg_parse(argc, argv, (void **)&rpm_args);
	if (nerrors != 0) {
		arg_print_errors(stderr, rpm_args.end, argv[0]);
		return 1;
	}

	if (rpm_args.reverse->count == 0) {
		setpoint = rpm_args.setpoint->ival[0];
	} else {
		setpoint = -rpm_args.setpoint->ival[0];
	}
	printf("setpoint is %d\n", setpoint);
	return 0;
}

void register_rpm(void)
{
	rpm_args.setpoint = arg_int0(NULL, NULL, "<s>", "RPM Setpoint");
	rpm_args.reverse = arg_litn("r", NULL, 0, 1, "Reverse");
	rpm_args.end = arg_end(2);

	const esp_console_cmd_t rpm_cmd = {
		.command = "rpm",
		.help = "Set The RPM",
		.hint = NULL,
		.func = &rpm,
		.argtable = &rpm_args
	};

	ESP_ERROR_CHECK( esp_console_cmd_register(&rpm_cmd) );
}

/**
 * @brief Function called when command `consoletest` will be invoked.
 * It will simply print `test_message` defined above.
 */
static int console_test(int argc, char **argv) {
	printf("%s\n", test_message);
	return 0;
}

void app_main(void)
{
	printf("hello world\n");

	// Initialize The UART
	xTaskCreate(driver_task,"driver_task", 1024, NULL, 10, NULL);
	xTaskCreate(sensor_task, "sensor_task", 2048, NULL, 15, NULL);

	// Set Up the REPL
	esp_console_repl_t *repl = NULL;
	esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
	repl_config.prompt = "r2 >";
	const esp_console_cmd_t cmd = {
		.command = "consoletest",
		.help = "Test console by sending a message",
		.func = &console_test
	};
	esp_console_dev_uart_config_t uart_config = {
							.channel = CONSOLE_UART_CHANNEL,
							.baud_rate = UARTS_BAUD_RATE,
							.tx_gpio_num = CONSOLE_UART_TX_PIN,
							.rx_gpio_num = CONSOLE_UART_RX_PIN
						};

	/**
	 * As we don't have a real serial terminal, (we just use default UART to
	 * send and received commands, ) we won't handle any escape sequence, so the
	 * easiest thing to do is set the console to "dumb" mode.  */
	linenoiseSetDumbMode(1);

	ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
	ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
	register_rpm();
	ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
