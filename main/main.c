#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#define HOST		HSPI_HOST

#define PIN_NUM_CIPO	25
#define PIN_NUM_COPI	-1
//23
#define PIN_NUM_CLK	19
#define PIN_NUM_CS	22

void app_main(void)
{
	printf("hello world\n");

	esp_err_t ret;
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

	spi_transaction_t t;
	for(;;) {
		memset(&t, 0, sizeof(t));
		t.length = 24;
		t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
		ret = spi_device_polling_transmit(spi, &t);	// Transceive
		assert(ret == ESP_OK);
		
		// Ignore First Bit
		uint16_t data = (t.rx_data[0] & 0x7f) << 5 | (t.rx_data[1] >> 3 & 0x1f);
	       	/*bool ocf = (t.rx_data[1] & 0x04) == 0x04;
		bool cof = (t.rx_data[1] & 0x02) == 0x02;
		bool lin = (t.rx_data[1] & 0x01) == 0x01;
		bool mag_inc = (t.rx_data[2] & 0x04) == 0x04;
		bool mag_dec = (t.rx_data[2] & 0x02) == 0x02;
		bool crc = (t.rx_data[2] & 0x01) == 0x01;
		*/
		//printf("data: %u, ocf: %u, cof: %u, lin: %u, magINC: %u, magDEC: %u, crc: %u\n", data, ocf, cof, lin, mag_inc, mag_dec, crc);
		printf("%u\n", data);
		vTaskDelay(10 / portTICK_RATE_MS);
	}

}
