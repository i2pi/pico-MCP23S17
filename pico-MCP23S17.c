#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"


// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16 // Pico Pin 21
#define PIN_CS   17 // Pico Pin 22
#define PIN_SCK  18 // Pico Pin 24
#define PIN_MOSI 19 // Pico Pin 25

enum MCP23S17_Register {
    IODIRA = 0x00,
    IODIRB = 0x01,
    IPOLA = 0x02,
    IPOLB,
    GPINTENA,
    GPINTENB,
    DEFVALA,
    DEFVALB,
    INTCONA,
    INTCONB,
    IOCON = 0x0A,
    GPPUPA = 0x0C,
    GPPUPB = 0x0D,
    INTFA,
    INTFB,
    INTCAPA,
    INTCAPB,
    GPIOA,
    GPIOB,
    OLATA,
    OLATB
};

void mcp23s17_init(void) {
    // SPI initialisation. This example will use SPI at 100kHz
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    spi_init(SPI_PORT, 100 * 1000);
    spi_set_format(SPI_PORT, 8, 0, 0, SPI_MSB_FIRST);

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 0);
}

void mcp23s17_write(enum MCP23S17_Register register_addr, uint8_t data) {
    uint8_t spi_message[4];
    // Assumes address is 000
    uint8_t device_opcode_write = 0b01000000;

    spi_message[0] = device_opcode_write;
    spi_message[1] = register_addr;
    spi_message[2] = data;

    gpio_put(PIN_CS, 0);
    while (!spi_is_writable(SPI_PORT));
    spi_write_blocking(SPI_PORT, spi_message, 3);
    gpio_put(PIN_CS, 1);
}

uint8_t mcp23s17_read(uint8_t register_addr) {
    uint8_t spi_message[4];
    // Assumes address is 000
    uint8_t device_opcode_write = 0b01000001;
    uint8_t data;

    spi_message[0] = device_opcode_write;
    spi_message[1] = register_addr;

    gpio_put(PIN_CS, 0);
    while (!spi_is_writable(SPI_PORT));
    data = spi_write_blocking(SPI_PORT, spi_message, 2);
    gpio_put(PIN_CS, 1);

    return(data);
}

int main()
{
    uint8_t i = 0;
    uint16_t delay = 150;

    mcp23s17_init();

    uint8_t device_opcode_read  = 0b01000001; 

    mcp23s17_write(IODIRA, 0x00);
    mcp23s17_write(GPIOA, 0xFF);


    while (true) {
        mcp23s17_write(GPIOA, 1 << i);
        sleep_ms(delay);
        mcp23s17_write(GPIOA, 0x00);
        sleep_ms(delay);

        i++;
        if (i >= 8) i = 0;
    }
}
