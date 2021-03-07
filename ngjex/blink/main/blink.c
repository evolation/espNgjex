/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/task.h"

#include "esp_system.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

// #include "TMC5160/TMC5160.h"
/* Can use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO GPIO_NUM_4
#define GSPI_IOMUX_PIN_NUM_MISO GPIO_NUM_7
#define GSPI_IOMUX_PIN_NUM_MOSI GPIO_NUM_8
#define GSPI_IOMUX_PIN_NUM_CLK GPIO_NUM_6
#define GSPI_IOMUX_PIN_NUM_CS GPIO_NUM_11

#define TAG "TMC_APP"
static spi_device_handle_t spi;

void tmc5160_readWriteArray(uint8_t channel, uint8_t *data, size_t p_length)
{
    size_t length = p_length * 8;
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t)); //Zero out the t    spi_device_handle_t spi;ransaction
    t.length = length;
    t.tx_buffer = data;
    
    t.rxlength=length;
    t.rx_buffer = data;
    
    //The data is the cmd itself
    ESP_LOGI(TAG, "Request ID ->%.2x %.2x %.2x %.2x %.2x ", ((uint8_t *)t.tx_buffer)[0], ((uint8_t *)t.tx_buffer)[1], ((uint8_t *)t.tx_buffer)[2], ((uint8_t *)t.tx_buffer)[3], ((uint8_t *)t.tx_buffer)[4]);

    t.user = (void *)0;                         //D/C needs to be set to 0
    ret = spi_device_polling_transmit(spi, &t); //Transmit!
    assert(ret == ESP_OK);
    ESP_LOGW(TAG, "Result Status %.2x %.2x %.2x %.2x %.2x ", ((uint8_t *)t.rx_buffer)[0], ((uint8_t *)t.rx_buffer)[1], ((uint8_t *)t.rx_buffer)[2], ((uint8_t *)t.rx_buffer)[3], ((uint8_t *)t.rx_buffer)[4]);

    // memset(&t, 0, sizeof(t));
    // t.length = length;
    // t.rxlength = length;
    // t.rx_buffer = data;
    // t.user = (void *)1;
    // ret = spi_device_polling_transmit(spi, &t);
    // assert(ret == ESP_OK);
    // ESP_LOGW(TAG, "Result Status %.2x %.2x %.2x %.2x %.2x ", ((uint8_t *)t.rx_buffer)[0], ((uint8_t *)t.rx_buffer)[1], ((uint8_t *)t.rx_buffer)[2], ((uint8_t *)t.rx_buffer)[3], ((uint8_t *)t.rx_buffer)[4]);
}

// static void send_cmd(spi_device_handle_t spi, const uint8_t* cmd)
// {
//     esp_err_t ret;
//     spi_transaction_t t;
//     memset(&t, 0, sizeof(t));       //Zero out the transaction
//     t.length=32;                     //Command is 8 bits
//     t.tx_buffer=cmd;               //The data is the cmd itself
//     t.user=(void*)0;                //D/C needs to be set to 0
//     ret=spi_device_polling_transmit(spi, &t);  //Transmit!
//     assert(ret==ESP_OK);            //Should have had no issues.
// }

// static uint32_t get_id(spi_device_handle_t spi)
// {
//     spi_transaction_t t;
//     memset(&t, 0, sizeof(t));
//     t.length=32;
//     t.flags = SPI_TRANS_USE_RXDATA;
//     t.user = (void*)1;

//     esp_err_t ret = spi_device_polling_transmit(spi, &t);
//     assert( ret == ESP_OK );

//     return *(uint32_t*)t.rx_data;
// }

static void spi_main(void)
{
    esp_err_t ret;
    gpio_set_pull_mode(VSPI_IOMUX_PIN_NUM_MISO, GPIO_FLOATING);
    gpio_set_direction(VSPI_IOMUX_PIN_NUM_MISO, GPIO_MODE_INPUT);

    const spi_bus_config_t bus_config = {
        .mosi_io_num = VSPI_IOMUX_PIN_NUM_MOSI,
        .miso_io_num = VSPI_IOMUX_PIN_NUM_MISO,
        .sclk_io_num = VSPI_IOMUX_PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 4 * 1000 * 1000,    //Clock out at 10 MHz
        .mode = 0,                             //SPI mode 0
        .spics_io_num = VSPI_IOMUX_PIN_NUM_CS, //CS pin
        .queue_size = 7                        //We want to be able to queue 7 transactions at a time
    };

    // Initialize the SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &bus_config, 1));

    //Attach the LCD to the SPI bus
    ret = spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    // //Initialize the LCD

    while (1)
    {
        uint8_t data[5] = {0x21, 0x00, 0x00, 0x00, 0x00};
        tmc5160_readWriteArray(0, data, sizeof(data));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    xTaskCreate(spi_main, "spi_main_task", 10000, NULL, tskIDLE_PRIORITY + 1, NULL);

    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while (1)
    {
        /* Blink off (output low) */
        printf("Turning off the LED\n");
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        printf("Turning on the LED\n");
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
