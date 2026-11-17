/*
 * ESP32 SPI master for the DE1-SoC SPI register-file peripheral.
 * Classic ESP32, ESP-IDF v5.x, SPI Mode 0, console on UART0 (115200).
 *
 * Wiring: GPIO18 SCLK, GPIO23 MOSI, GPIO19 MISO, GPIO5 CS, shared GND.
 *
 * Commands (hex addr/data, decimal hz/n):
 *   R <addr>           -> OK R <addr> <data>
 *   W <addr> <data>    -> OK W <addr> <data>
 *   F <hz>             -> OK F <hz> actual_khz=<khz>   (real SCLK after divider rounding)
 *   S <n>              -> OK S n=<n> errors=<e>   (random SCRATCH write/readback)
 */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_random.h"

#define PIN_SCLK      18
#define PIN_MOSI      23
#define PIN_MISO      19
#define PIN_CS        5
#define ADDR_SCRATCH  0x04

static spi_device_handle_t dev = NULL;

static void spi_set_freq(int hz)
{
    if (dev) spi_bus_remove_device(dev);
    spi_device_interface_config_t cfg = {
        .mode           = 0,
        .clock_speed_hz = hz,
        .spics_io_num   = PIN_CS,
        .queue_size     = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &cfg, &dev));
}

/* Write: 16 bits = {0, addr}, data */
static void fpga_write(uint8_t addr, uint8_t data)
{
    spi_transaction_t t = {
        .flags   = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .length  = 16,
        .tx_data = { addr & 0x7F, data },
    };
    spi_device_polling_transmit(dev, &t);
}

/* Read: 24 bits = {1, addr}, dummy, data (data arrives in byte 2) */
static uint8_t fpga_read(uint8_t addr)
{
    spi_transaction_t t = {
        .flags   = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .length  = 24,
        .tx_data = { 0x80 | (addr & 0x7F), 0x00, 0x00 },
    };
    spi_device_polling_transmit(dev, &t);
    return t.rx_data[2];
}

/* Read one line from UART0 (blocking). */
static void read_line(char *buf, int max)
{
    int len = 0;
    while (1) {
        char c;
        uart_read_bytes(UART_NUM_0, &c, 1, portMAX_DELAY);
        if (c == '\r' || c == '\n') {
            if (len == 0) continue;
            buf[len] = '\0';
            return;
        }
        if (len < max - 1) buf[len++] = c;
    }
}

void app_main(void)
{
    spi_bus_config_t bus = {
        .sclk_io_num   = PIN_SCLK,
        .mosi_io_num   = PIN_MOSI,
        .miso_io_num   = PIN_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_DISABLED));
    spi_set_freq(1000000);
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));

    printf("OK READY\n");

    char line[32];
    unsigned a, d;
    int n;

    while (1) {
        read_line(line, sizeof(line));

        if (sscanf(line, "R %x", &a) == 1) {
            printf("OK R %02X %02X\n", a, fpga_read(a));
        }
        else if (sscanf(line, "W %x %x", &a, &d) == 2) {
            fpga_write(a, d);
            printf("OK W %02X %02X\n", a, d);
        }
        else if (sscanf(line, "F %d", &n) == 1) {
            spi_set_freq(n);
            int khz = 0;   /* real SCLK after the driver rounds to 80 MHz / n */
            spi_device_get_actual_freq(dev, &khz);
            printf("OK F %d actual_khz=%d\n", n, khz);
        }
        else if (sscanf(line, "S %d", &n) == 1) {
            int errors = 0;
            for (int i = 0; i < n; i++) {
                uint8_t v = esp_random() & 0xFF;
                fpga_write(ADDR_SCRATCH, v);
                if (fpga_read(ADDR_SCRATCH) != v) errors++;
                if ((i & 0xFF) == 0xFF) vTaskDelay(1);   /* keep the watchdog happy */
            }
            printf("OK S n=%d errors=%d\n", n, errors);
        }
        else {
            printf("ERR\n");
        }
    }
}
