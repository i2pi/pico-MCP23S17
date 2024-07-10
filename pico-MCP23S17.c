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
    IODIRB,
    IPOLA,
    IPOLB,
    GPINTENA,
    GPINTENB,
    DEFVALA,
    DEFVALB,
    INTCONA,
    INTCONB,
    IOCON = 0x0A,
    GPPUPA = 0x0C,
    GPPUPB,
    INTFA,
    INTFB,
    INTCAPA,
    INTCAPB,
    GPIOA,
    GPIOB,
    OLATA,
    OLATB
};

typedef struct MCP23S17_t {
    spi_inst_t *spi;
    uint8_t     cs_pin;
    uint8_t     addr;

    uint8_t     read_opcode;
    uint8_t     write_opcode;
} MCP23S17_t;

void mcp23s17_init(MCP23S17_t *xpndr, spi_inst_t *spi, uint8_t addr, uint8_t cs_pin, uint32_t baudrate) {
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    spi_init(spi, baudrate);
    spi_set_format(spi, 8, 0, 0, SPI_MSB_FIRST); 

    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1);

    xpndr->spi = spi;
    xpndr->cs_pin = cs_pin;
    xpndr->addr = addr;

    xpndr->read_opcode  = 0b01000001 | (addr << 1);
    xpndr->write_opcode = 0b01000000 | (addr << 1);
}

void mcp23s17_write(MCP23S17_t *xpndr, enum MCP23S17_Register register_addr, uint8_t data) {
    uint8_t spi_message[3];

    spi_message[0] = xpndr->write_opcode;
    spi_message[1] = register_addr;
    spi_message[2] = data;

    gpio_put(xpndr->cs_pin, 0);
    spi_write_blocking(xpndr->spi, spi_message, 3);
    gpio_put(xpndr->cs_pin, 1);
}

uint8_t mcp23s17_read(MCP23S17_t *xpndr, uint8_t register_addr) {
    uint8_t spi_message[3];
    uint8_t data_buffer[3];

    spi_message[0] = xpndr->read_opcode;
    spi_message[1] = register_addr;
    spi_message[2] = 0x00;

    gpio_put(xpndr->cs_pin, 0);
    spi_write_read_blocking(xpndr->spi, spi_message, data_buffer, 3);
    gpio_put(xpndr->cs_pin, 1);

    return(data_buffer[2]);
}

int main()
{
    MCP23S17_t xpndr;
    uint8_t i = 0;
    uint16_t delay = 150;

    uint8_t data;

    mcp23s17_init(&xpndr, SPI_PORT, 0b000, PIN_CS, 1e5);

    mcp23s17_write(&xpndr, IODIRA, 0x00);
    mcp23s17_write(&xpndr, GPIOA, 0xFF);

    mcp23s17_write(&xpndr, GPPUPB, 0xFF); // enable weak pullup on B

    while (true) {
        mcp23s17_write(&xpndr, GPIOA, 1 << i);
        sleep_ms(delay);
        mcp23s17_write(&xpndr, GPIOA, 0x00);
        sleep_ms(delay);

        data = mcp23s17_read(&xpndr, GPIOB);

        i++;
        if (i >= 8) i = 0;
    }
}
