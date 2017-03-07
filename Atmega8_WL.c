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
#include <avr/pgmspace.h>
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

// lookup-Tabelle KTY84

// pgm_read_word(&KTY[xy])
#define KTY_OFFSET   30             // Offset, Start bei bei -32 °C
#define ADC_OFFSET   207            // Startwert der ADC-Messung
#define KTY_FAKTOR   96             // 0x60, Multiplikator
const uint16_t KTY[] PROGMEM =
{
   0x1D,	0x1FC,	0x3E8,	0x5B2,	0x789,	0x942,	0xB08,	0xCB2,
   0xE6A,	0x1007,	0x11B4,	0x1346,	0x14E8,	0x1672,	0x180B,	0x198D,
   0x1B1E,	0x1C9A,	0x1E24,	0x1F9A,	0x211F,	0x228F,	0x23FD,	0x257B,
   0x26E5,	0x285F,	0x29C6,	0x2B3D,	0x2CA0,	0x2E14,	0x2F75,	0x30E7,
   0x3246,	0x33B5,	0x3513,	0x3681,	0x37DD,	0x394A,	0x3AA5,	0x3C11,
   0x3D6B,	0x3ED7,	0x4031,	0x419C,	0x42F6,	0x4461,	0x45BB,	0x4716,
   0x4882,	0x49DD,	0x4B49,	0x4CA5,	0x4E13,	0x4F70,	0x50DE,	0x523D,
   0x53AD,	0x550C,	0x567F,	0x57E0,	0x5954,	0x5AB7,	0x5C2D,	0x5D93,
   0x5F0B,	0x6073,	0x61EF,	0x6359,	0x64D7,	0x6645,	0x67C6,	0x6936,
   0x6AA8,	0x6C2F,	0x6DA4,	0x6F2E,	0x70A7,	0x7235,	0x73B2,	0x7544,
   0x76C5,	0x785C,	0x79E2,	0x7B7D,	0x7D07,	0x7EA8,	0x8037,	0x81DD,
   0x8371,	0x851D,	0x86B6,	0x8868,	0x8A07,	0x8BBF,	0x8D65,	0x8F23,
   0x90CF,	0x927E,	0x9447,	0x95FE,	0x97CE,	0x998B,	0x9B63,	0x9D29,
   0x9F09,	0xA0D7,	0xA2C0,	0xA496,	0xA688,	0xA867,	0xAA63,	0xAC4C,
   0xAE52,	0xB045,	0xB256,	0xB453,	0xB66F,	0xB877,	0xBAA0,	0xBCB3,
   0xBEE8,	0xC109,	0xC32F,	0xC578,	0xC7AC,	0xCA04,	0xCC46,	0xCEAD,
   0xD0FE,	0xD375,	0xD5D7,	0xD85F,	0xDAD1,	0xDD6C,	0xDFF0,	0xE29E,
   0xE535,	0xE7F7,	0xEAA3,	0xED7B,	0xF03C,	0xF32C,	0xF604,	0xF90E,
  
};


uint16_t loopCount0=0;
uint16_t loopCount1=0;
uint16_t loopCount2=0;

uint16_t pwmpos=0;

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

// ACD https://www.avrprogrammers.com/howto/attiny-comparator
// ACD https://www.avrprogrammers.com/howto/attiny-comparator
#define COMP_PORT PORTB
#define COMP_DDR DDRB

// Pins fuer Drive der RC
#define COMP_DRIVE_PIN_A  1
#define COMP_DRIVE_PIN_B  2

#define COMP_ADC_PORT PORTC
#define COMP_ADC_DDR DDRC

#define COMP_ADC_PIN_A  4
#define COMP_ADC_PIN_B  5

#define COMP_AIN_PORT   PORTD
#define COMP_AIN_DDR    DDRD
#define COMP_AIN0       6
#define COMP_AIN1       7


#define MULTIPLEX 1

volatile uint16_t captured_value;
volatile uint8_t captured;
volatile uint8_t overflow=0;
volatile uint8_t captcounter=0;
volatile uint16_t mittelwertA[4];
volatile uint16_t mittelwertB[4];
volatile uint8_t mposA=0;
volatile uint8_t mposB=0;
volatile uint8_t adckanal=0;

// end ACD

// end ACD


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

void deviceinit(void)
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
   
   ADCDDR &= ~(1<<PORTC2);
   ADCPORT &= ~(1<<PORTC2);
   ADCDDR &= ~(1<<PORTC3);
   ADCPORT &= ~(1<<PORTC3);
	

	DDRB |= (1<<PORTB1);	//OC1A: Bit 1 von PORT B als Ausgang fuer PWM
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
   (1<<SPR0);//|
            //(1<<SPR1);
   
   /*
    Slave init
    // Set MISO output, all others input
    DDR_SPI = (1<<DD_MISO);
    // Enable SPI 
    SPCR = (1<<SPE);
    */
   
}

void timer1_comp(void)
{
   // Set pin for driving resistor low.
   COMP_DDR |= (1<<COMP_DRIVE_PIN_A);
   COMP_PORT &= ~(1<<COMP_DRIVE_PIN_A);
   COMP_DDR |= (1<<COMP_DRIVE_PIN_B);
   COMP_PORT &= ~(1<<COMP_DRIVE_PIN_B);
   
   // Disable the digital input buffers.
   //   DIDR = (1<<AIN1D) | (1<<AIN0D);
   if (MULTIPLEX)
   {
      // ADC-Eingaenge fuer Capt
      COMP_ADC_DDR &= ~(1<<COMP_ADC_PIN_A);
      COMP_ADC_PORT &= ~(1<<COMP_ADC_PIN_A);
      
      COMP_ADC_DDR &= ~(1<<COMP_ADC_PIN_B);
      COMP_ADC_PORT &= ~(1<<COMP_ADC_PIN_B);
      
      // AIN0, AIN1 Eingang
      COMP_AIN_DDR &= ~(1<<COMP_AIN0);
      COMP_AIN_DDR &= ~(1<<COMP_AIN1);
      
      
      SFIOR |= (1<<ACME);
      //ADMUX = 3;
   }
   
   
   //ADCSRA =0;//| = (1<<ADEN);                    // disable ADC if necessary
   ACSR =   (1<<ACIC) | (1<<ACIS1) | (1<<ACIS0);   // Comparator enabled, no bandgap, input capture.
   // Timer...
   TCCR1A = 0;
   TCCR1B =   (1<<CS10);                        // F_CPU / 1
   //TCCR1B =  (1<<ICES1);                      // Input capture on rising edge
   TCNT1 = 0;
   TIMSK |= (1<<TOIE1) | (1<<TICIE1);           // Timer interrupts on capture and overflow.
   sei();
}


#pragma mark timer1
// Timer1 Servo
/*
void timer1(void)
{
   
   // https://www.mikrocontroller.net/topic/83609
   
   
   OCR1A = 0x3E8;           // Pulsdauer 1ms
   OCR1A = 0x200;
 
   ICR1 = 0x6400;          // 0x6400: Pulsabstand 50 ms
   // http://www.ledstyles.de/index.php/Thread/18214-ATmega32U4-Schaltungen-PWM/
   DDRB |= (1<<PB1);
   
   TCCR1A |= (1<<COM1A1)|(1<<COM1B1)|(1<<WGM10);
   
   TCCR1B |= (1<<WGM12)|(1<<CS11);
   
   //  TIMSK |= (1<<OCIE1A) | (1<<TICIE1); // OC1A Int enablad
}

*/

ISR(TIMER1_CAPT_vect)
{
   // Save the captured value and drop the drive line.
   if (captured == 0)
   {
      // captured_value = ICR1;
      captcounter++;
      
      if (adckanal == COMP_ADC_PIN_A)
      {
         mittelwertA[mposA++] = ICR1;           // Ringbuffer fuer gleitenden Mittelwert
         mposA &= 0x03;                         // position incrementieren
         COMP_PORT &= ~(1<<COMP_DRIVE_PIN_A);   // auf 4 beschraenken
      }
      
      if (adckanal == COMP_ADC_PIN_B)
      {
         mittelwertB[mposB++] = ICR1;
         mposB &= 0x03;
         COMP_PORT &= ~(1<<COMP_DRIVE_PIN_B);
      }
      TCNT1 = 0;
      captured = 1;
   }
   //TCNT1 = 0;
}


#pragma mark INT0 WL
ISR(INT0_vect)
{
   wl_spi_status |= (1<<7);
   
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
	
	deviceinit();
	
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
  // timer1_comp();
   initADC(0);
   
   uint8_t delaycount=10;
  #pragma mark while
   uint8_t readstatus = wl_module_get_data((void*)&wl_data);
   //lcd_puts(" los");
   sei();
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
         
         //lcd_gotoxy(14,1);
         //lcd_puthex(int0counter);
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
         //lcd_puts("          ");
         if (wl_status & (1<<RX_DR)) // IRQ: Package has been sent
         {
            //OSZIA_LO;
            lcd_gotoxy(0,0);
            lcd_puts("RX");
            uint8_t rec = wl_module_get_rx_pw(0);
            //lcd_gotoxy(0,3);
            //lcd_puthex(rec);
            //lcd_putc(' ');
            uint8_t readstatus = wl_module_get_data((void*)&wl_data);
            uint8_t i;
            lcd_gotoxy(4,0);
            lcd_puts("rs:");
            lcd_puthex(readstatus);
            lcd_putc(' ');
            lcd_putint1(wl_data[0]);
            lcd_putc('.');
            for (i=2; i<4; i++)
            {
               lcd_putint1(wl_data[i]);
            }
            //lcd_putc(' ');
            uint16_t temperatur = (wl_data[11]<<8);
            temperatur |= wl_data[10];
            //lcd_putint12(temperatur);
            
            temperatur /=4; // *256/1024, unkalibriert
        //    lcd_putint2(temperatur/10);
        //    lcd_putc('.');
        //   lcd_putint1(temperatur%10);

            //lcd_put_tempbis99(temperatur);

            lcd_gotoxy(18,3);
            lcd_puthex(wl_data[9]);
            pwmpos = temperatur;
            OCR1A = temperatur;
            OSZIA_HI;
            
            wl_spi_status |= (1<<6);
            
          }
         
         if (wl_status & (1<<TX_DS)) // IRQ: Package has been sent
         {
            //OSZIA_LO; // 50 ms mit Anzeige, 140us ohne Anzeige
            sendcounter++;
       //     lcd_gotoxy(3,0);
       //     lcd_puts("TX");
            wl_module_config_register(STATUS, (1<<TX_DS)); //Clear Interrupt Bit
            PTX=0;
            //OSZIA_HI;
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

			if ((loopCount1 >0x00AF) )//&& (!(Programmstatus & (1<<MANUELL))))
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
            
            // MARK: ADC Loop
            
            uint16_t adc2wert = readKanal(2);
            
            lcd_gotoxy(0,3);
            lcd_puthex(adc2wert&0x00FF);
            lcd_puthex((adc2wert&0xFF00)>>8);
            lcd_gotoxy(6,3);
            lcd_putint12(adc2wert);

            lcd_gotoxy(12,3);
            //lcd_putint12(adcwert);
            uint16_t temperatur2 =  adc2wert*10/4; // *256/1024, unkalibriert
            lcd_putint(temperatur2/10);
            lcd_putc('.');
            lcd_putint1(temperatur2%10);
           
 
            
            uint16_t adc3wert = readKanal(3); // KTY
            lcd_gotoxy(0,2);
            //lcd_puthex(adc3wert&0x00FF);
            //lcd_puthex((adc3wert&0xFF00)>>8);
            //lcd_gotoxy(6,2);
            lcd_putint12(adc3wert);
         //   adc3wert-=6;
            /*
             #define KTY_OFFSET   30             // Offset, Start bei bei -30 °C
             #define ADC_OFFSET   204            // Startwert der ADC-Messung
             #define KTY_FAKTOR   96             // 0x60, Multiplikator
             // pgm_read_word(&KTY[xy])
             */
            uint16_t tableindex = ((adc3wert - ADC_OFFSET)>>3); // abrunden auf Intervalltakt
            lcd_putc(' ');
           // lcd_putint(tableindex);
            uint8_t col = (adc3wert - ADC_OFFSET) & 0x07;
           // lcd_putc(' ');
           //lcd_putint2(col);
            uint16_t ktywert = pgm_read_word(&KTY[tableindex]); // Wert in Tabelle, unterer Wert
            lcd_putint12(ktywert);

            if (col) // nicht exakter wert, interpolieren
           {
              uint16_t diff = pgm_read_word(&KTY[tableindex+1])-ktywert;
              //diff = (diff * col)<<3;
              ktywert += (diff * col)>>3;
           }
            
            lcd_putc(' ');
            lcd_putint12((ktywert));

            lcd_putc(' ');
            lcd_putint12((ktywert/KTY_FAKTOR)-KTY_OFFSET);

            
//            lcd_putc(' ');
//            uint16_t diff = pgm_read_word(&KTY[tableindex+1])-ktywert;
//            diff = (diff * col)>>3;

//            lcd_putint12(diff);
           
           // lcd_gotoxy(6,2);
            //lcd_puthex((ktywert & 0xFF00)>>8);
           // lcd_puthex(ktywert & 0x00FF);
           // lcd_putc(' ');
           // lcd_putint12(adc3wert - ADC_OFFSET);
           // lcd_putint12(ktywert);
            /*
            lcd_gotoxy(12,2);
            //lcd_putint12(adcwert);
            uint16_t temperatur3 =  adc3wert*10/4; // *256/1024, unkalibriert
            lcd_putint(temperatur3/10);
            lcd_putc('.');
            lcd_putint1(temperatur3%10);
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
            
            payload[10] = adc2wert & 0x00FF;
            payload[11] = (adc2wert & 0xFF00)>>8;
 
//            payload[12] = adc3wert & 0x00FF;
//            payload[13] = (adc3wert & 0xFF00)>>8;
            payload[12] = (ktywert/KTY_FAKTOR) & 0x00FF;
            payload[13] = ((ktywert/KTY_FAKTOR) & 0xFF00)>>8;
            
            if (wl_spi_status & (1<<6))
            {
               wl_spi_status &= ~(1<<6);
               
               wl_module_tx_config(0);
               //lcd_putc('b');
               
               wl_module_send(payload,wl_module_PAYLOAD);
               //lcd_putc('c');
               
               uint8_t tx_status = wl_module_get_status();
               
               lcd_gotoxy(0,1);
               //lcd_putc(' ');
               lcd_puthex(tx_status);
               lcd_putc(' ');
               lcd_puthex(sendcounter);
               
               maincounter++;
               PTX=0;

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
            //wl_status=0;
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
      
      // ***
       // ***
      
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
