#include "AD-DA-MOCKSPI.h"

int cpt_drdy = 0;

bool spi_mock_modulo_cpt(int modulo)
{
    cpt_drdy++;
    return (cpt_drdy) % modulo == 0;
}

void spi_delay_us(uint64_t micros)
{
    printf("Waiting for %lld us \n", micros);
}

uint8_t spi_transfer(uint8_t data)
{
    printf("transfer %d\n", data);
    uint8_t r = 48;
    printf("returning %d\n", r);
    return r;
}

int spi_init()
{
    printf("spi_init\n");
    return 1;
}

int spi_begin()
{
    printf("spi_begin\n");
    return 1;
}

int spi_init_adc_dac_board()
{
    printf("spi_init_adc_dac_board\n");
    return 1;
}

void spi_end()
{
    printf("spi_end\n");
}

int spi_close()
{
    printf("spi_close\n");
    return 1;
}
