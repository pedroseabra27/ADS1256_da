#ifndef __arm__

#include <bcm2835.h>
#include <stdint.h>

void bcm2835_gpio_write( uint8_t pin, uint8_t on ){}
uint8_t bcm2835_gpio_lev( uint8_t pin ) { return 0;}
int bcm2835_spi_begin(void) { return 0; };
void bcm2835_spi_setBitOrder( uint8_t order ) {};
void bcm2835_spi_setDataMode( uint8_t mode ) {};
void bcm2835_spi_setClockDivider( uint16_t divider ) {};
void bcm2835_gpio_fsel( uint8_t pin, uint8_t mode ) {};
void bcm2835_gpio_set_pud( uint8_t pin, uint8_t pud ) {};
uint8_t bcm2835_spi_transfer( uint8_t value ) { return 0; };
int bcm2835_init(void) { return 0; };
void bcm2835_spi_end(void) {};
int bcm2835_close(void) { return 1; };

#endif
