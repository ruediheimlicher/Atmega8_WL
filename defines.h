/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <joerg@FreeBSD.ORG> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.        Joerg Wunsch
 * ----------------------------------------------------------------------------
 *
 * General stdiodemo defines
 *
 * $Id: defines.h,v 1.1.2.2 2006/10/08 21:51:14 joerg_wunsch Exp $
 */

/* CPU frequency */
//#define F_CPU 8000000UL

#define LOOPLED_PORT	PORTD
#define LOOPLED_DDR	DDRD
#define LOOPLED_PIN	4

#define ADCPORT   PORTC
#define ADCDDR   DDRC

//Oszi
#define OSZIPORT           PORTC
#define OSZIDDR            DDRC
#define PULSA              5
//#define OSZI_PULS_B        5
#define OSZIA_LO OSZIPORT &= ~(1<<PULSA)
#define OSZIA_HI OSZIPORT |= (1<<PULSA)
#define OSZIA_TOGG OSZIPORT ^= (1<<PULSA)

#define TEST_PIN           5

#define SPI_DDR			DDRB						// DDR fuer SPI
#define SPI_PORT        PORTB						// Port fuer SPI
#define SPI_PORTPIN     PINB						// Port-Pin fuer SPI

#define SPI_MISO           PB4
#define SPI_MOSI           PB3
#define SPI_CLK            PB5
#define SPI_SS             PB2

#define OSZIA_LO OSZIPORT &= ~(1<<PULSA)
#define OSZIA_HI OSZIPORT |= (1<<PULSA)
#define OSZIA_TOGG OSZIPORT ^= (1<<PULSA)

// Port fuer INT0
#define INTERRUPT_PORT   PORTD
#define INTERRUPT_DDR   DDRD

#define INT0_PIN        PD2
#define INT1_PIN        PD3

// Port fuer Chip select
#define SPI_WL_PORT     PORTC
#define SPI_WL_DDR      DDRC
#define SPI_WL_CE       PC0
#define SPI_WL_CSN       PC1

