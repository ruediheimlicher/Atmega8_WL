//
//  Tastenblinky.c
//  Tastenblinky
//
//  Created by Sysadmin on 03.10.07.
//  Copyright Ruedi Heimlihcer 2007. All rights reserved.
//



#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
#include <string.h>
#include <inttypes.h>
//#define F_CPU 4000000UL  // 4 MHz
#include <avr/delay.h>
#include "lcd.c"
#include "adc.c"

//#include "onewire.c"
//#include "ds18x20.c"
//#include "crc8.c"

//#include "conio.h"
#include "defines.h"
#include "wireless.c"
#include "wl_module.h" 
#include "nRF24L01.h"



uint16_t loopCount0=0;
uint16_t loopCount1=0;
uint16_t loopCount2=0;

#define PROGRAMM_DS	0
#define GRUPPE_DS		0xC0
//#define GRUPPE_DS	0xB0

#define TWI_PORT		PORTC
#define TWI_PIN		PINC
#define TWI_DDR		DDRC

#define SDAPIN		4
#define SCLPIN		5

#define SPI_PORT     PORTB
#define SPI_PIN      PINB
#define SPI_DDR      DDRB



#define TASTE1		19
#define TASTE2		29
#define TASTE3		44
#define TASTE4		67
#define TASTE5		94
#define TASTE6		122
#define TASTE7		155
#define TASTE8		186
#define TASTE9		205
#define TASTEL		223
#define TASTE0		236
#define TASTER		248
#define TASTATURPORT PORTC
#define TASTATURPIN		3

#define MANUELL_PORT		PORTD
#define MANUELL_DDR		DDRD
#define MANUELL_PIN		PIND

#define MANUELL			7	// Bit 7 von Status 
#define MANUELLPIN		6	// Pin 6 von PORT D fuer Anzeige Manuell
#define MANUELLNEU		7	// Pin 7 von Status. Gesetzt wenn neue Schalterposition eingestellt
#define MANUELLTIMEOUT	100 // Loopled-counts bis Manuell zurueckgesetzt wird. 02FF: ca. 100 s


#define FOSC 1000000    /* oscillator-frequency in Hz */
#define BAUD 57600  //valid values:9600, 19200, 57600 kbits


volatile uint8_t					Programmstatus=0x00;
	uint8_t Tastenwert=0;
	uint8_t TastaturCount=0;
volatile uint16_t					Manuellcounter=0; // Countr fuer Timeout	
	uint16_t TastenStatus=0;
	uint16_t Tastencount=0;
	uint16_t Tastenprellen=0x01F;

volatile uint8_t data;

volatile uint16_t	spiwaitcounter=0;


volatile uint8_t spi_status=0;

//Variablen WL
// MARK: WL Defs
volatile uint8_t wl_status=0;
volatile uint8_t PTX=0;
volatile uint8_t int0counter=0;
volatile uint8_t sendcounter=0;
volatile uint8_t wl_spi_status;
char itoabuffer[20];
volatile uint8_t wl_data[wl_module_PAYLOAD] = {};


//volatile char text[] = {'*','M','a','s','t','e','r','*'};
char* text = "* Master *";

//#define MAXSENSORS 5
/*
static uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];
static int16_t gTempdata[MAXSENSORS]; // temperature times 10
static uint8_t gTemp_measurementstatus=0; // 0=ok,1=error
static int8_t gNsensors=0;
static uint8_t gScratchPad[9];
static volatile uint8_t sensornummer=0;
// Code 1_wire start
//uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];


uint8_t search_sensors(void)
{
	uint8_t i;
	uint8_t id[OW_ROMCODE_SIZE];
	uint8_t diff, nSensors;
	
	
	ow_reset();
	
	nSensors = 0;
	
	diff = OW_SEARCH_FIRST;
	while ( diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ) 
	{
		DS18X20_find_sensor( &diff, &id[0] );
		
		if( diff == OW_PRESENCE_ERR ) 
		{
			lcd_gotoxy(0,1);
			lcd_puts("No Sensor found\0" );
			break;
		}
		
		if( diff == OW_DATA_ERR ) 
		{
			lcd_gotoxy(0,1);
			lcd_puts("Bus Error\0" );
			break;
		}
		lcd_gotoxy(4,1);

		for ( i=0; i < OW_ROMCODE_SIZE; i++ )
			{
				//lcd_gotoxy(15,1);
				//lcd_puthex(id[i]);

			gSensorIDs[nSensors][i] = id[i];
			//delay_ms(100);
			}
			
		nSensors++;
	}
	
	return nSensors;
}

// start a measurement for all sensors on the bus:
void start_temp_meas(void){
        gTemp_measurementstatus=0;
        if ( DS18X20_start_meas(NULL) != DS18X20_OK) 
		  {
                gTemp_measurementstatus=1;
        }
}

// read the latest measurement off the scratchpad of the ds18x20 sensor
// and store it in gTempdata
void read_temp_meas(void){
        uint8_t i;
        uint8_t subzero, cel, cel_frac_bits;
        for ( i=0; i<gNsensors; i++ ) 
		  {
			  
			  if ( DS18X20_read_meas( &gSensorIDs[i][0], &subzero,
											 &cel, &cel_frac_bits) == DS18X20_OK ) 
			  {
				  gTempdata[i]=cel*10;
				  gTempdata[i]+=DS18X20_frac_bits_decimal(cel_frac_bits);
				  if (subzero)
				  {
					  gTempdata[i]=-gTempdata[i];
				  }
			  }
			  else
			  {
				  gTempdata[i]=0;
			  }
        }
}

uint8_t Sensornummerlesen(uint8_t index, uint8_t* nummer)
{
	uint8_t tempScratchPad[9];
	if  ((DS18X20_read_scratchpad(&gSensorIDs[index][0], tempScratchPad ))== DS18X20_OK)
	{
			lcd_gotoxy(14,1);
		lcd_puts("GUT\0");

		*nummer=tempScratchPad[2];
		return DS18X20_OK;
	}
	
	else 
	{
				lcd_gotoxy(14,1);
		lcd_puts("BAD\0");

		*nummer=0xFF;
		return DS18X20_ERROR;
	}
}

uint8_t Sensornummer(uint8_t index)
{
	uint8_t tempScratchPad[9];
	if  ((DS18X20_read_scratchpad(&gSensorIDs[index][0], tempScratchPad ))== DS18X20_OK)
	{
		lcd_gotoxy(14,1);
		lcd_puts("GUT\0");
		return tempScratchPad[2]; // Byte 2: Nummer des Sensors
	}
	
	else 
	{
			lcd_gotoxy(14,1);
		lcd_puts("BAD\0");

		return 0xFF;
	}
}



// Code 1_wire end
*/
void delay_ms(unsigned int ms)
/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}

uint8_t Tastenwahl(uint8_t Tastaturwert)
{
//lcd_gotoxy(0,1);
//lcd_putint(Tastaturwert);
if (Tastaturwert < TASTE1)
return 1;
if (Tastaturwert < TASTE2)
return 2;
if (Tastaturwert < TASTE3)
return 3;
if (Tastaturwert < TASTE4)
return 4;
if (Tastaturwert < TASTE5)
return 5;
if (Tastaturwert < TASTE6)
return 6;
if (Tastaturwert < TASTE7)
return 7;
if (Tastaturwert < TASTE8)
return 8;
if (Tastaturwert < TASTE9)
return 9;
if (Tastaturwert < TASTEL)
return 10;
if (Tastaturwert < TASTE0)
return 0;
if (Tastaturwert < TASTER)
return 12;

return -1;
}

void SPI_Master_init (void)
{
   
   SPCR |= (1<<MSTR);// Set as Master
   
   //  SPCR0 |= (1<<CPOL0)|(1<<CPHA0);
   
   /*
    SPI2X 	SPR1 	SPR0     SCK Frequency
    0       0        0     fosc/4
    0       0        1     fosc/16
    0       1        0     fosc/64
    0       1        1     fosc/128
    1       0        0     fosc/2
    1       0        1     fosc/8
    1       1        0     fosc/32
    1       1        1     fosc/64
    */
   
   //SPCR |= (1<<SPR0);               // div 16 SPI2X: div 8
   SPCR |= (1<<SPR1);               // div 64 SPI2X: div 32
   //SPCR |= (1<<SPR1) | (1<<SPR0);   // div 128 SPI2X: div 64
   //SPCR |= (1<<SPI2X0);
   
   SPCR |= (1<<SPE); // Enable SPI
   spi_status = SPSR;								//Status loeschen
   wl_spi_status = 0;
}

void slaveinit(void)
{
//	MANUELL_DDR |= (1<<MANUELLPIN);		//Pin 5 von PORT D als Ausgang fuer Manuell
	//MANUELL_PORT &= ~(1<<MANUELLPIN);
 	//DDRD |= (1<<CONTROL_A);	//Pin 6 von PORT D als Ausgang fuer Servo-Enable
	//DDRD |= (1<<CONTROL_B);	//Pin 7 von PORT D als Ausgang fuer Servo-Impuls
	LOOPLED_DDR |= (1<<LOOPLED_PIN);
	//PORTD &= ~(1<<CONTROL_B);
	//PORTD &= ~(1<<CONTROL_A);
   OSZIDDR |= (1<<PULSA);	//Pin 0 von  als Ausgang fuer OSZI
   OSZIPORT |= (1<<PULSA);		// HI
   

	

	DDRB |= (1<<PORTB1);	//Bit 1 von PORT B als Ausgang fuer PWM
	PORTB &= ~(1<<PORTB1);	//LO
	

	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);	//Pin 5 von PORT C als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 6 von PORT C als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 7 von PORT C als Ausgang fuer LCD

#if defined(__AVR_ATmega8__)
   // Initialize external interrupt 0 (PD2)
   MCUCR = ((1<<ISC11)|(0<<ISC10)|(1<<ISC01)|(0<<ISC00));	// Set external interupt on falling edge
   GICR  = ((0<<INT1)|(1<<INT0));							// Activate INT0
   
   
#endif // __AVR_ATmega8__
   
#if defined(__AVR_ATmega88A__)
   EICRA = ((1<<ISC11)|(0<<ISC10)|(1<<ISC01)|(0<<ISC00));	// Set external interupt on falling edge for INT0 and INT1
   EIMSK  = ((0<<INT1)|(1<<INT0));							// Activate INT0
#endif // __AVR_ATmega88A__
   
#if defined(__AVR_ATmega168__)
   // Initialize external interrupt on port PD6 (PCINT22)
   DDRB &= ~(1<<PD6);
   PCMSK2 = (1<<PCINT22);
   PCICR  = (1<<PCIE2);
#endif // __AVR_ATmega168__
   
#if defined(__AVR_ATmega32U4__)
   // Initialize external interrupt on port PD0
   INTERRUPT_DDR &= ~(1<<INT0_PIN);
   INTERRUPT_PORT |= (1<<INT0_PIN);

   EICRA = ((1<<ISC11)|(0<<ISC10)|(1<<ISC01)|(0<<ISC00));
   
   // Set external interupt on falling edge for INT0 and INT1
   EIMSK  = ((1<<INT0));
   
#endif // __AVR_ATmega32U4__
   
}

char SPI_get_put_char(int cData)
{
   
   //Putchar -- Master
   /* Start transmission */
   SPDR = cData;
   /* Wait for transmission complete */
   while(!(SPSR & (1<<SPIF)))
      ;
   /* Return data register */
   return SPDR;
}

void SPI_Init(void)
{
   //http://www.atmel.com/dyn/resources/prod_documents/doc2467.pdf  page:165
   /*
   SPI_DDR |= (1<<2)|(1<<3)|(1<<5);
   SPI_DDR &= ~(1<<4);
   SPCR |= (1<<MSTR);
   SPCR |= (1<<SPR0)|(1<<SPR1);
   SPCR |= (1<<SPE);
   */
   
   
  
   //Master init
   // Set MOSI and SCK output, all others input
   SPI_DDR &= ~(1<<SPI_MISO);
   SPI_DDR |= (1<<SPI_MOSI)|(1<<SPI_CLK)|(1<<SPI_SS);
   
   
   // Enable SPI, Master, set clock rate fck/16 
   SPCR =   (1<<SPE)|
            (1<<MSTR)|
            (1<<SPR0)|
            (1<<SPR1);
   
   /*
    Slave init
    // Set MISO output, all others input
    DDR_SPI = (1<<DD_MISO);
    // Enable SPI 
    SPCR = (1<<SPE);
    */
   
}

#pragma mark INT0 WL
ISR(INT0_vect)
{
   wl_spi_status |= (1<<7);
   /*
   uint8_t status;
   // Read wl_module status
   
   
   wl_module_CSN_lo;
   _delay_us(10);

   status = spi_fast_shift(NOP);
   _delay_us(10);

   wl_status = status;
   wl_module_CSN_hi;
   // Pull down chip select
   // Read status register
   // Pull up chip select

   if (status & (1<<TX_DS)) // IRQ: Package has been sent
   {
      wl_module_config_register(STATUS, (1<<TX_DS)); //Clear Interrupt Bit
      PTX=0;
   }
   
   if (status & (1<<MAX_RT)) // IRQ: Package has not been sent, send again
   {
      wl_module_config_register(STATUS, (1<<MAX_RT)); // Clear Interrupt Bit
      wl_module_CE_hi;
      _delay_us(10);
      wl_module_CE_lo;
   }
   
   if (status & (1<<TX_FULL))            //TX_FIFO Full <-- this is not an IRQ
   {
      wl_module_CSN_lo;                // Pull down chip select
      spi_fast_shift(FLUSH_TX);        // Flush TX-FIFO
      wl_module_CSN_hi;                // Pull up chip select
   }
   */
}

ISR (SPI_STC_vect)
{
  // SPDR = data;
}



int main (void)
{
	/* INITIALIZE */
//	LCD_DDR |=(1<<LCD_RSDS_PIN);
//	LCD_DDR |=(1<<LCD_ENABLE_PIN);
//	LCD_DDR |=(1<<LCD_CLOCK_PIN);
	
	slaveinit();
	
   SPI_Init();
   SPI_Master_init();
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
	lcd_puts("Guten Tag\0");
	delay_ms(1000);
   
	lcd_cls();
   lcd_gotoxy(0,0);
	lcd_puts("READY\0");
	
	// DS1820 init-stuff begin
	// DS1820 init-stuff end
   
   volatile char incoming=0;
   size_t poscounter=0;
   
   // MARK: WL main
   uint8_t payload[wl_module_PAYLOAD];
   uint8_t maincounter =0;
   //Array for Payload
   
   wl_module_init();
   _delay_ms(50);
  
   sei();
   
   wl_module_rx_config();
  
   delay_ms(50);
   wl_module_CE_hi;
   //
   lcd_clr_line(0);
   
   //lcd_puts(" und");
   /*
   DDRC |= (1<<0);
   DDRC |= (1<<1);
   PORTC |= (1<<0);
   PORTC &= ~(1<<1);
   */
   uint8_t delaycount=10;
  #pragma mark while
   uint8_t readstatus = wl_module_get_data((void*)&wl_data);
   //lcd_puts(" los");
   while (1)
	{
    //  PORTC |= (1<<0);
		loopCount0 ++;
		//_delay_ms(2);
		//LOOPLED_PORT ^= (1<<LOOPLED_PIN);
      //incoming = SPDR;
      
      if (wl_spi_status & (1<<7)) // in ISR gesetzt
      {
         
         if (int0counter < 0x2F)
         {
            int0counter++;
         }
         else
         {
            int0counter=0;
         }
         
         wl_spi_status &= ~(1<<7);
         
         lcd_gotoxy(14,1);
         lcd_puthex(int0counter);
         lcd_gotoxy(18,1);
         lcd_puthex(wl_status);
// MARK: WL Loop
         /*
         lcd_gotoxy(0,1);
         lcd_puthex(wl_status & (1<<RX_DR));
         lcd_puthex(wl_status & (1<<TX_DS));
         lcd_puthex(wl_status & (1<<MAX_RT));
         lcd_puthex(wl_status & (1<<TX_FULL));
         */
         wl_status = wl_module_get_status();
         
         lcd_gotoxy(0,0);
         lcd_puts("          ");
         if (wl_status & (1<<RX_DR)) // IRQ: Package has been sent
         {
            OSZIA_LO;
            lcd_gotoxy(0,0);
            lcd_puts("RX");
            uint8_t rec = wl_module_get_rx_pw(0);
            lcd_gotoxy(0,3);
            lcd_puthex(rec);
            lcd_putc(' ');
            uint8_t readstatus = wl_module_get_data((void*)&wl_data);
            uint8_t i;
            lcd_puthex(readstatus);
            lcd_putc(' ');
            lcd_putint1(wl_data[0]);
            lcd_putc('.');
            for (i=2; i<5; i++)
            {
               lcd_putint1(wl_data[i]);
            }
            lcd_putc(' ');
            lcd_puthex(wl_data[9]);
            OSZIA_HI;
            
            wl_spi_status |= (1<<6);
            
          }
         
         if (wl_status & (1<<TX_DS)) // IRQ: Package has been sent
         {
            OSZIA_LO;
            sendcounter++;
            lcd_gotoxy(3,0);
            lcd_puts("TX");
            wl_module_config_register(STATUS, (1<<TX_DS)); //Clear Interrupt Bit
            PTX=0;
            OSZIA_HI;
         }
         
         if (wl_status & (1<<MAX_RT)) // IRQ: Package has not been sent, send again
         {
            lcd_gotoxy(6,0);
            lcd_puts("RT");
            
            wl_module_config_register(STATUS, (1<<MAX_RT)); // Clear Interrupt Bit
            wl_module_CE_hi;
            _delay_us(10);
            wl_module_CE_lo;
         }
       
         
         
      } // end ISR abarbeiten

      
		if (loopCount0 >=0xFE)
		{
			
			loopCount1++;

			if ((loopCount1 >0x002F) )//&& (!(Programmstatus & (1<<MANUELL))))
			{
            LOOPLED_PORT ^= (1<<LOOPLED_PIN);
            // WL-Routinen
            
            // WL
            //OSZIA_LO;
            /*
            wl_module_CE_lo;
            _delay_ms(5);
            wl_module_CSN_lo;
            _delay_ms(5);
            wl_module_CSN_hi;
            _delay_ms(5);
            wl_module_CE_hi;
             */
            
            
            
            uint8_t k;
            for (k=0; k<wl_module_PAYLOAD; k++)
            {
               payload[k] = wl_module_PAYLOAD-k;
            }
            // Euler 2,71828182
            payload[0] = maincounter;
            payload[1] = 0;
            payload[2] = 2;
            payload[3] = 0;
            payload[4] = 7;
            payload[5] = 1;
            payload[6] = 8;
            payload[7] = 2;
            
            if (wl_spi_status & (1<<6))
            {
               wl_spi_status &= ~(1<<6);
               //payload[8] = 8;
               //payload[9] = 1;
               /*
                wl_module_CE_lo;
                _delay_ms(5);
                wl_module_CSN_lo;
                _delay_ms(5);
                wl_module_CSN_hi;
                _delay_ms(5);
                wl_module_CE_hi;
                */
               
               lcd_gotoxy(0,1);
               
               lcd_putc('a');
               
               wl_module_tx_config(0);
               lcd_putc('b');
               
                wl_module_send(payload,wl_module_PAYLOAD);
                lcd_putc('c');
               
               
               uint8_t tx_status = wl_module_get_status();
               lcd_putc(' ');
               lcd_puthex(tx_status);
               lcd_putc(' ');
                lcd_puthex(sendcounter);
                
                
                maincounter++;
                PTX=0;
                /*
                */
               wl_module_rx_config();
               
            } // if
            //lcd_gotoxy(0,3);
            
            //lcd_puthex(maincounter);
            
            if (maincounter >250)
               
            {
               maincounter = 0;
            }
            
            loopCount2++;
            
            //lcd_gotoxy(18,0);
            //lcd_puthex(loopCount2);
 
            loopCount1=0;
            wl_status=0;
				// DS1820 loop-stuff begin
            /*
				start_temp_meas();
				delay_ms(800);
				read_temp_meas();
				
				//Sensor 1
				lcd_gotoxy(0,1);
				lcd_puts("1:\0");
				if (gTempdata[0]/10>=100)
				{
					lcd_gotoxy(1,1);
					lcd_putint((gTempdata[0]/10));
				}
				else
				{
					lcd_gotoxy(2,1);
					lcd_putint2((gTempdata[0]/10));
				}
				
				lcd_putc('.');
				lcd_putint1(gTempdata[0]%10);
				
				// Sensor 2
				lcd_gotoxy(7,1);
				lcd_puts("2:\0");
				if (gTempdata[1]/10>=100)
				{
					lcd_gotoxy(8,1);
					lcd_putint((gTempdata[1]/10));
				}
				else
				{
					lcd_gotoxy(9,1);
					lcd_putint2((gTempdata[1]/10));
				}
				
				lcd_putc('.');
				lcd_putint1(gTempdata[1]%10);
				
				// Sensor 3
				lcd_gotoxy(14,1);
				lcd_puts("3:\0");
				if (gTempdata[2]/10>=100)
				{
					lcd_gotoxy(15,1);
					lcd_putint((gTempdata[2]/10));
				}
				else
				{
					lcd_gotoxy(16,1);
					lcd_putint2((gTempdata[2]/10));
				}
				
				lcd_putc('.');
				lcd_putint1(gTempdata[2]%10);
				
				
				
				lcd_gotoxy(15,0);
				lcd_puts("   \0");
				lcd_gotoxy(15,0);
				lcd_puthex(gTemp_measurementstatus);

				*/
				
				// DS1820 loop-stuff end
				
				
				//lcd_putint(gTempdata[1]);
				//lcd_putint(gTempdata[2]);
				//delay_ms(1000);
			}
			
			loopCount0 =0;
		}
      
      /*
		if (!(PINB & (1<<PB0))) // Taste 0
		{
			lcd_gotoxy(10,1);
			lcd_puts("P0 Down\0");
         lcd_puthex(TastenStatus);
			
			if (! (TastenStatus & (1<<PB0))) //Taste 0 war nicht nicht gedrueckt
			{
            lcd_gotoxy(10,1);
            lcd_puts("P0 neu \0");
				TastenStatus |= (1<<PB0);
            delay_ms(1000);
				Tastencount=0;
				//lcd_gotoxy(3,1);
				//lcd_puts("P0 \0");
				//lcd_putint(Servoimpulsdauer);
				//delay_ms(800);
				SPDR = 'x';
            while(!(SPSR & (1<<SPIF)) && spiwaitcounter<0xFFF)
            {
               spiwaitcounter++;
            }
            spiwaitcounter=0;
            delay_ms(1000);
            //lcd_gotoxy(10,1);
				//lcd_puts("       \0");
            lcd_gotoxy(19,1);
            lcd_putc('+');
			}
			else
			{
				lcd_gotoxy(19,1);
            lcd_putc('$');
				//lcd_puts("*         *\0");
				
				Tastencount ++;
				if (Tastencount >= Tastenprellen)
				{
					Tastencount=0;
					TastenStatus &= ~(1<<PB0);
               lcd_gotoxy(10,1);
               lcd_puts("*         *\0");
               

               
				}
			}//	else
			
		} // Taste 0
*/
		
#pragma mark Tastatur 
		/* ******************** */
//		initADC(TASTATURPIN);
//		Tastenwert=(readKanal(TASTATURPIN)>>2);
		
//		lcd_gotoxy(3,1);
//		lcd_putint(Tastenwert);
//		Tastenwert=0;
		if (Tastenwert>5)
		{
			/*
			 0:											1	2	3
			 1:											4	5	6
			 2:											7	8	9
			 3:											x	0	y
			 4: Schalterpos -
			 5: Manuell ein
			 6: Schalterpos +
			 7: 
			 8: 
			 9: 
			 
			 12: Manuell aus
			 */
			 
			TastaturCount++;
			if (TastaturCount>=200)
			{
				
				 
				 //lcd_gotoxy(17,1);
				 //lcd_puts("T:  \0");
				 //lcd_putint(Tastenwert);
				 
				uint8_t Taste=Tastenwahl(Tastenwert);
				//Taste=0;
				 //lcd_gotoxy(19,1);
				 //lcd_putint1(Taste);
				 //delay_ms(600);
				// lcd_clr_line(1);
				 

				TastaturCount=0;
				Tastenwert=0x00;
				uint8_t i=0;
				uint8_t pos=0;
//				lcd_gotoxy(18,1);
//				lcd_putint2(Taste);
				continue;
				switch (Taste)
				{
					case 0:// Schalter auf Null-Position
					{ 
						if (Programmstatus & (1<<MANUELL))
						{
							Manuellcounter=0;
							Programmstatus |= (1<<MANUELLNEU);
							/*
							lcd_gotoxy(13,0);
							lcd_puts("S\0");
							lcd_gotoxy(19,0);
							lcd_putint1(Schalterposition); // Schalterstellung
							lcd_gotoxy(0,1);
							lcd_puts("SI:\0");
							lcd_putint(ServoimpulsdauerSpeicher); // Servoimpulsdauer
							lcd_gotoxy(5,0);
							lcd_puts("SP\0");
							lcd_putint(Servoimpulsdauer); // Servoimpulsdauer
							*/
						}
						
					}break;
						
					case 1:	//	
					{ 
					if (Programmstatus & (1<<MANUELL))
						{
						uint8_t i=0;
						lcd_gotoxy(0,0);
						lcd_puts("Sens\0");
						lcd_putc('1');
						lcd_putc(' ');
                     /*
						for (i=0;i<OW_ROMCODE_SIZE;i++)
						{
						lcd_puthex(gSensorIDs[0][i]);
						if (i==3)
						{
						lcd_gotoxy(0,1);
						}
						lcd_putc(' ');
						}
                      */
						Manuellcounter=0;
						
						}
					}break;
						
					case 2://
					{ 
					
						if (Programmstatus & (1<<MANUELL))
						{
                     /*
						uint8_t i=0;
						lcd_gotoxy(0,0);
						lcd_puts("Sens\0");
						lcd_putc('1');
						lcd_putc(' ');
						for (i=0;i<OW_ROMCODE_SIZE;i++)
						{
						lcd_puthex(gSensorIDs[1][i]);
						if (i==3)
						{
						lcd_gotoxy(0,1);
						}
						lcd_putc(' ');
						}
                      */
						Manuellcounter=0;
						
						
						}
						
					}break;
						
					case 3: //	Uhr aus
					{ 
						if (Programmstatus & (1<<MANUELL))
						{
                     /*
						uint8_t i=0;
						lcd_gotoxy(0,0);
						lcd_puts("Sens\0");
						lcd_putc('1');
						lcd_putc(' ');
						for (i=0;i<OW_ROMCODE_SIZE;i++)
						{
						lcd_puthex(gSensorIDs[2][i]);
						if (i==3)
						{
						lcd_gotoxy(0,1);
						}
						lcd_putc(' ');
						}
                      */
						Manuellcounter=0;
						

						}
					}break;
						
					case 4://
					{ 
						//DS18X20_read_scratchpad(&gSensorIDs[0][0], gScratchPad );
                  /*
                  uint8_t i=0;
						lcd_gotoxy(0,0);
						lcd_puts("Sens\0");
						lcd_putc('0');
						lcd_putc(' ');
						for (i=0;i<OW_ROMCODE_SIZE;i++)
						{
						lcd_puthex(gScratchPad[i]);
						if (i==3)
						{
						lcd_gotoxy(0,1);
						}
						lcd_putc(' ');
						}
*/
					}break;
						
					case 5://
					{ 
						Programmstatus |= (1<<MANUELL);	// MANUELL ON
						Manuellcounter=0;
						MANUELL_PORT |= (1<<MANUELLPIN);
						Programmstatus |= (1<<MANUELLNEU);
						lcd_clr_line(1);
						/*
							lcd_gotoxy(13,0);
							lcd_puts("S\0");
							lcd_putint1(Schalterposition); // Schalterstellung
							lcd_gotoxy(0,1);
							lcd_puts("SP:\0");
							lcd_putint(ServoimpulsdauerSpeicher); // Servoimpulsdauer
							lcd_gotoxy(5,0);
							lcd_puts("SI\0");
							lcd_putint(Servoimpulsdauer); // Servoimpulsdauer
						*/
					}break;
						
					case 6://
					{ 
					//sensornummer=0xAF;
					//Sensornummerlesen(0,&sensornummer);
					//	lcd_gotoxy(0,0);
					//	lcd_puts("Sens\0");
					//	lcd_putc('1');
					//	lcd_putc(' ');
					//	lcd_puthex(sensornummer);
					
					}break;
						
					case 7:// Schalter rückwaerts
					{
                  /*
					sensornummer=0x00;
					Sensornummerlesen(0,&sensornummer);
						lcd_gotoxy(0,0);
						lcd_puts("Sens\0");
						lcd_putc('0');
						lcd_putc(' ');
						lcd_puthex(sensornummer);
					*/
					}break;
						
					case 8://
					{
                  /*
					sensornummer=0x00;
					Sensornummerlesen(1,&sensornummer);
						lcd_gotoxy(0,0);
						lcd_puts("Sens\0");
						lcd_putc('1');
						lcd_putc(' ');
						lcd_puthex(sensornummer);
					*/

					}break;
						
					case 9:// Schalter vorwaerts
					{
                  /*
					sensornummer=0x00;
					Sensornummerlesen(2,&sensornummer);
						lcd_gotoxy(0,0);
						lcd_puts("Sens\0");
						lcd_putc('2');
						lcd_putc(' ');
						lcd_puthex(sensornummer);
					*/
					}break;

					case 10:// *
					{ 
						
					}break;

					case 11://
					{ 
						
					}break;
						
					case 12: // # Normalbetrieb einschalten
					{
						Programmstatus &= ~(1<<MANUELL); // MANUELL OFF
						Programmstatus &= ~(1<<MANUELLNEU);
						MANUELL_PORT &= ~(1<<MANUELLPIN);
					}
						
				}//switch Tastatur
				
//				delay_ms(400);
//				lcd_gotoxy(18,1);
//				lcd_puts("  ");		// Tastenanzeige loeschen

			}//if TastaturCount	
			
		}
	}
	
	
	return 0;
}
