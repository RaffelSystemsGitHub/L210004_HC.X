
//Filename: 		Static Lumbar HC Code
//Author:   		Caleb Schultz-Stachnik
//Date:   			2019/10/27
//File Version:		G
//Company:			Raffel Systems
//MCU: 				PIC16F1937

//Changelog: 
//REV A: Reworked with a software debounce
//REV B: Added all-zone-off -> massage-off feature, improved pin mapping for layout, PRCLOSE -> HOME
//REV C: Added independent heat+massage timers
//REV D: Improved UART signal integrity, fixed mode btn state change, added LED timeout
//REV E: Fix to allow hotswapping HC's, allows UART comm after disconnecting HC from JCT
//REV F: Fix for timeout -> 30 min, HR+PR not interrupt massage, AB deflate after timeout
//REV G: Enabled WDT to prevent softlocking when both HC and JCT expect an RX, fixed round BTN flag errors, fixed disconnecting HC while JCT has active function


//Includes
#include <stdio.h>
#include <htc.h>
#include <pic.h>
//#include <pic16f1937.h>
#include <stdbool.h>


__CONFIG(FOSC_INTOSC & WDTE_SWDTEN & PWRTE_OFF & MCLRE_OFF & CP_ON & BOREN_OFF & CLKOUTEN_OFF);
__CONFIG(WRT_OFF & STVREN_OFF & BORV_LO  & LVP_OFF);

//The following is the sample UART communication code. This is included in the 
//handcontrol and the junction, with no variation between them. It is HIGHLY 
//recommended that this remains unchanged, and any changes occur in main() by 
//writing to the corresponding control register. 
#ifndef _SERIAL_H_
#define _SERIAL_H_

#define BAUD 9600      
#define FOSC 4000000L
#define NINE 0    /* Use 9bit communication? FALSE=8bit */

#define DIVIDER ((int)(FOSC/(16UL * BAUD) -1))
#define HIGH_SPEED 1

#if NINE == 1
#define NINE_BITS 0x40
#else
#define NINE_BITS 0
#endif

#if HIGH_SPEED == 1
#define SPEED 0x4
#else
#define SPEED 0
#endif

#if defined(_16F87) || defined(_16F88)
	#define RX_PIN TRISB2
	#define TX_PIN TRISB5
#else
	#define RX_PIN TRISC7
	#define TX_PIN TRISC6
#endif

/* Serial initialization */
#define init_comms()\
	RX_PIN = 1;	\
	TX_PIN = 1;		  \
	SPBRG = DIVIDER;     	\
	RCSTA = (NINE_BITS|0x90);	\
	TXSTA = (SPEED|NINE_BITS|0x20)

void putch(unsigned char);
unsigned char getch(void);
unsigned char getche(void);

#endif

//define Utility
#define     ARR_LENGTH      5
#define     NELEMS(x)       (sizeof(x)/sizeof((x)[0]))

//define UART Commands
#define		MASSAGE_ON		'A'
#define		MASSAGE_OFF		'B'
#define		PR_OPEN			'C'
#define		PR_CLOSE		'D'
#define		PR_HOLD			'E'
#define		HR_OPEN			'F'
#define		HR_CLOSE		'G'
#define		HR_HOLD			'H'
#define		LUM_OPEN		'I'
#define		LUM_CLOSE		'J'
#define		LUM_HOLD		'K'
#define		Z0_ON			'L'
#define		Z0_OFF			'M'
#define		Z1_ON			'N'
#define		Z1_OFF			'O'
#define		Z2_ON			'P'
#define		Z2_OFF			'Q'
#define		Z3_ON			'R'
#define		Z3_OFF			'S'
#define		M_ALTERNATE		'T'
#define		M_WAVE			'U'
#define		M_PULSE			'V'
#define		HEAT_ON			'W'
#define		HEAT_OFF		'X'
#define     ACT_OFF         'Y'
#define     OP_GO           '+'
#define     SYNC            '?'
#define     SYNC_ACK        '!'
#define     ACK             '~'

//I/O Defines

//#define		ZONE0_BTN		RD5
//#define		ZONE1_BTN		RD3
//#define		ZONE2_BTN		RA6
//#define		ZONE3_BTN		RC5
//#define		ZONE0_LED		LATD4
//#define		ZONE1_LED		LATD2
//#define		ZONE2_LED		LATA7
//#define		ZONE3_LED		LATC4

#define		POWER_BTN		RD1
#define		POWER_LED		LATD0
//#define		MODE_BTN		RC3 
#define		HEAT_LED		LATA3
#define		HEAT_BTN		RB4
//#define		MODE3_LED		LATC0
//#define		MODE2_LED		LATC1
//#define		MODE1_LED		LATC2

#define		LUM_OPEN_BTN	RE0
#define		PR_OPEN_BTN     RA5
#define		HR_OPEN_BTN 	RA4
#define		LUM_CLOSE_BTN	RB1
#define		PR_CLOSE_BTN	RB2
#define		HR_CLOSE_BTN	RB3

//Timer setup for 0.05 seconds
#define		PERIOD			50000							// period in uS
#define		XTAL			4000000							// crystal frequency - 4MHz
#define		_XTAL_FREQ		4000000
#define		IPERIOD			(4 * 1000000 / XTAL)			// Period of instruction clock in uSeconds
#define		SCALE			2								// Timer 0 prescaler
#define		T0_TICKS 		256								// Number of counts for interrupt
#define		TICK_PERIOD		(SCALE * IPERIOD)				// Period (uSec) of one increment of timer 0
#define		RELOADS			((PERIOD/T0_TICKS)/TICK_PERIOD)
#define		SEC_RELOAD		200

//Global Constants
#define		DEBOUNCE		200	//random selected, need to examine scope for more accurate count
#define     LONG_TOUT       20*60*29.5
#define     OFF_COUNT       20*7*20
#define     ARR_LENGTH      5
#define     NELEMS(x)       (sizeof(x)/sizeof((x)[0]))
#define     WDT_RESET       0b00011001
#define     WDT_ACT         0b00100001

//Global Variables
volatile unsigned char tx_buffer;
volatile unsigned char last_tx[ARR_LENGTH] = {0};
volatile unsigned char rx_char[ARR_LENGTH] = {0};
volatile int  time_out = SEC_RELOAD;
volatile int  massage_timeout = 0;
volatile int  heat_timeout = 0;
volatile int  mode_state = 1;
volatile bool lum_btn_op  = false;
volatile bool lum_op_sent = false;
volatile bool lum_btn_cl  = false;
volatile bool lum_cl_sent = false;
volatile bool pr_btn_op   = false;
volatile bool pr_op_sent  = false;
volatile bool pr_btn_cl   = false;
volatile bool pr_cl_sent  = false;
volatile bool hr_btn_op   = false;
volatile bool hr_op_sent  = false;
volatile bool hr_btn_cl   = false;
volatile bool hr_cl_sent  = false;
//Toggle bools
volatile bool pwr_on    = true;			//power toggle flags
volatile bool pwr_off   = false;		//set false so first run will power on
volatile bool heat_on   = true;			//heat toggle flags
volatile bool heat_off  = false;			
volatile bool z0_on     = true;			//zone toggle flags
volatile bool z0_off    = false;
volatile bool z1_on     = true;
volatile bool z1_off    = true;
volatile bool z2_on     = false;
volatile bool z2_off    = false;
volatile bool z3_on     = true;
volatile bool z3_off    = false;

volatile bool pwr_btn_flg = false;
volatile bool pwr_sent = false;
volatile bool pwr_flg = false;
volatile bool heat_sent = false;
volatile bool z0_sent = false;
volatile bool z1_sent = false;
volatile bool z2_sent = false;
volatile bool z3_sent = false;


char tx_ferr;		//used to clear framing errors
volatile bool tx_flag = false;

//Function Prototypes

void 
putch(unsigned char byte) 
{
    long count = 0;
	//GIE = 0;	//disable interrupts during tx
	/* output one byte */
	while(!TXIF){	/* set when register is empty */
	//	continue;
        count++;
        if(count >= 10000000){
            break;
        }
    }
	TXREG = byte;
	//GIE = 1;	//enable interrupts after tx
}

unsigned char 
getch() {
    long count = 0;
	//GIE = 0;	//disable interrupts during rx
	/* retrieve one byte */
	while(!RCIF){	/* set when register is not empty */
	//	continue;
        count++;
        if(count >= 10000000){
            break;
        }
    }
	//GIE = 1;	//enable interrupts after rx	
	return RCREG;	
}

unsigned char
getche(void)
{
	GIE = 0;	//disable interrupts during rx+echo
	unsigned char c;
	putch(c = getch());
	GIE = 1;	//enable interrupts after rx+echo
	return c;
}
void hc_led_on();
void hc_led_off();
void connection_est();
bool syn_synack_ack();


//Main Code
void main(void){
    

	//Internal Oscillator Setup
	OSCCON = 0b01101000; 	//Internal 4 MHz
    
    //WDT Setup
    WDTCON = WDT_RESET;    //4s interval for soft-reset
    __delay_ms(3000);
    
	// I/O Defines
	// 1 = input, 0 = output
	
	TRISA = 0b01110111;
	//		  |||||||\-- UNUSED
	//		  ||||||\--- UNUSED
	//		  |||||\---- UNUSED
	//		  ||||\----- HEAT_LED 
	//		  |||\------ HR_CLOSE_BTN
	//		  ||\------- PR_CLOSE_BTN
	//		  |\-------- ZONE2_BTN
	//		  \--------- ZONE2_LED
	ANSELA = 0b00000000;
	
	TRISB = 0b11111111;
	//		  |||||||\-- UNUSED
	//		  ||||||\--- LUM_OPEN_BTN
	//		  |||||\---- PR_OPEN_BTN
	//		  ||||\----- HR_OPEN_BTN
	//		  |||\------ HEAT_BTN
	//		  ||\------- UNUSED
	//		  |\-------- PGC
	//		  \--------- PGD
	ANSELB = 0b00000000;
	
	TRISC = 0b10101000;
	//		  |||||||\-- MODE1_LED
	//		  ||||||\--- MODE2_LED
	//		  |||||\---- MODE3_LED
	//		  ||||\----- MODE_BTN
	//		  |||\------ ZONE3_LED
	//		  ||\------- ZONE3_BTN
	//		  |\-------- TX
	//		  \--------- RX
	//ANSELC = 0b00000000; //does not exist on PIC16F1933
	
	TRISD = 0b11101010;
	//		  |||||||\-- POWER_LED
	//		  ||||||\--- POWER_BTN
	//		  |||||\---- ZONE1_LED
	//		  ||||\----- ZONE1_BTN
	//		  |||\------ ZONE0_LED
	//		  ||\------- ZONE0_BTN
	//		  |\-------- UNUSED
	//		  \--------- UNUSED
	ANSELD = 0b0000000;
    
    TRISE = 0b0001;
    //        |||\-- LUM_CLOSE_BTN
    //        \----- UNUSED
	ANSELE = 0b0000;
    
	//Oscillator Control Register Definition
	OSCCON = 0b01101000;//INTERNAL 4MHz
	
	init_comms();		//run header func for serial comm init 
	
	//Interrupt Definition
	//INTCON = 0b00100000;
	//		   |||||||\-- IOC Flag 						IOCIF
	//		   ||||||\--- External Flag					INTF
	//		   |||||\---- TMR0 Flag						TMR0IF
	//		   ||||\----- IOC Enable					IOCIE
	//		   |||\------ External Interrupt Enable		INTE
	//		   ||\------- TMR0 Interrupt Enable			TMR0IE
	//		   |\-------- Peripheral Interrupt Enable	PEIE
	//		   \--------- Global Interrupt Enable 		GIE
	
	PIE1 = 0b00000000;
	//		 |||||||\--- TMR1IE
	//		 ||||||\---- TMR2IE
	//		 |||||\----- CCP1IE
	//		 ||||\------ SSPIE
	//		 |||\------- TXIE Transmit interrupt, may not be necessary for HC
	//		 ||\-------- RCIE Receive interrupt, may not be necessary
	//		 |\--------- ADIE
	//		 \---------- TMR1GIE
	
	//Timer0 configuration	
	//use for software debounce, polling pins in TMR0IF ISR
	OPTION_REG =  0b00001000;
	//			  	|||||||\-- not WPUEN, weak pull up enable
	//			 	||||||\--- INTEDG, interrupt edge select
	//			 	|||||\---- TMR0CS, timer0 clock source select, 1 T0CKI, 0 Internal Clock
	//				||||\----- TMR0SE, timer0 source edge select, 1 high>low, 0 low>high
	//				|||\------ PSA, prescaler assignment, TMR0 1 prescaler on, TMR0 0 prescaler off
	//				||\------- Prescale bit 2
	//				|\-------- Prescale bit 1
	//				\--------- Prescale bit 0 //111 = 256, double each binary increment
	WPUB = 0b00000000;
	//Serial Port Definition
	//TXSTA = 0b00100000;
	//		  |||||||\-- TX9D  - 9th data bit/address
	//		  ||||||\--- TRMT  - Transmit Shift Register Status 1 = empty, 0 = full
	//		  |||||\---- BRGH  - Baud Rate High Select, 1 = high, 0 = low
	//		  ||||\----- SENDB - Send Break Character 1 = send on next tx, 0 = sync complete
	//		  |||\------ SYNC  - Synchronous select 1 = sync, 0 = async
	//		  ||\------- TXEN  - Transmission enable, 1 = enable, 0 = disable
	//		  |\-------- TX9   - 9bit receive enable, don't need
	//		  \--------- CSRC  - Clock source select, 0 for async

	//RCSTA = 0b10000000;
	//		  |||||||\-- RX9D  - 9th bit of received data
	//		  ||||||\--- OERR  - Overrun error bit
	//		  |||||\---- FERR  - Framing error bit
	//		  ||||\----- ADDEN - Address detect
	//		  |||\------ CREN  - Continuous receive, 0 for tx
	//		  ||\------- SREN  - single receive enable, async = 0
	//		  |\-------- RX9   - 9 bit receive 
	//		  \--------- SPEN  - Serial Port Enable
	
	//Baud Rate Definition 
	BAUDCON = 0b00000000;
	SPBRGL  = 0x0C; 		//=> 51 = 1200 baud
	SPBRGH  = 0b00000000;
	OPTION_REG = 0b00001110;//  prescaler disabled, t0 prescale = 128
	
	TMR0IE = 1;
	GIE    = 1;	//interrupt enable
	
	//Software Debounce Variables		
	int z0_de    = DEBOUNCE;			
	int z1_de    = DEBOUNCE;		
	int z2_de    = DEBOUNCE;
	int z3_de    = DEBOUNCE;
	int heat_de  = DEBOUNCE;
	int mode_de  = DEBOUNCE;
	int power_de = DEBOUNCE;	
	
	int safety   = 0;
    
	LATA = 0xFF;                    //clear port outputs
    LATB = 0xFF;
    LATC = 0xFF;
    LATD = 0xFF;
    LATE = 0xFF;
    tx_buffer = 'a';                //stop extraneous tx    
    
    
//    MODE1_LED = 1;                  //mode LED turns on intermittently
//    MODE2_LED = 1;                  //drive pins high to prevent
//    MODE3_LED = 1;
    
	__delay_ms(250);                //let tx init complete
    
    bool connect_ack = true;
    while(connect_ack){             //run at power up to show connection made
        connect_ack = !syn_synack_ack();
    }
    
    connection_est();               //LED indication
    
    int x = 0;
    for(unsigned int i = 0; i < NELEMS(last_tx); ++i){
        putch(MASSAGE_OFF);
        last_tx[i] = MASSAGE_OFF; //tx set amount of commands
        __delay_us(5);          //maybe unnecessary 
        CLRWDT();
    }
    for(unsigned int i = 0; i < NELEMS(rx_char); ++i){
        rx_char[i] = getch();   //rx echo from JCT
        CLRWDT();
    }
    for(unsigned int i = 0; i < NELEMS(last_tx); ++i){
        if(last_tx[i] == rx_char[i]){
            x++;                //verify command was correctly rx'ed
        }
        CLRWDT();
    }
    if(x == ARR_LENGTH){        //decrease ARR_LENGTH for faster signal tx, increase for better signal integrity
//                    echo = false;           //exit while
        putch(OP_GO);           //send ACK to JCT to execute command
        x = 0;
    }
    
    for(unsigned int i = 0; i < NELEMS(last_tx); ++i){
        putch(ACT_OFF);
        last_tx[i] = ACT_OFF; //tx set amount of commands
        __delay_us(5);          //maybe unnecessary 
        CLRWDT();
    }
    for(unsigned int i = 0; i < NELEMS(rx_char); ++i){
        rx_char[i] = getch();   //rx echo from JCT
        CLRWDT();
    }
    for(unsigned int i = 0; i < NELEMS(last_tx); ++i){
        if(last_tx[i] == rx_char[i]){
            x++;                //verify command was correctly rx'ed
        }
        CLRWDT();
    }
    if(x == ARR_LENGTH){        //decrease ARR_LENGTH for faster signal tx, increase for better signal integrity
//                    echo = false;           //exit while
        putch(OP_GO);           //send ACK to JCT to execute command
        x = 0;
        CLRWDT();
    }
    
	while(1){			
		if(OERR || FERR){			//Error handling
			CREN = 0;
            SPEN = 0;
			CREN = 1;
            SPEN = 1;
			tx_ferr = RCSTA;
		}

//Most of the button press and tx depends on the MCU cycling through while(1) 
//sufficiently quickly so that there will not be time to press and read a button
//press between cycles. Behavior when 2 buttons are press simultaneously is unknown.
//Measures have been taken to minimize unusual behavior with 2 simultaneous presses. 
//This is mainly to prevent multiple tx of the same command to the junction. -CSS

//This version of firmware includes a software debounce. A hardware debounce is
//not necessary for the circuit . -CSS
					

//-=-=-=-**NOTICE**-=-=-=-=-
//zone0 corresponds to zone1 on the controller keypad
//this may cause some confusion so I am attempting to 
//clear any misunderstanding here. - CSS
		//zone0 enables.
//		if(ZONE0_BTN){				
//			if(!(--z0_de) && !pwr_on){
//				if(z0_on && !z0_sent){
//					tx_buffer = Z0_ON;
//					tx_flag = true;
//					ZONE0_LED = 0;
//					z0_on = false;
//                    z0_off = true;
//                    massage_timeout = LONG_TOUT;
//				}
//				else if(z0_off){
//					tx_buffer = Z0_OFF;
//                    z0_sent = true;
//					tx_flag = true;
//					ZONE0_LED = 1;
//					z0_off = false;
//                    z0_on = true;
//                    massage_timeout = LONG_TOUT;
//				}                
//			}
//            if((last_tx[0] != Z0_ON) && (z0_on == false)){  //these statements are a check to ensure that if the button is held 
////				z0_off = true;                              //the correct flags will still be set. Previously flags would be 
//                                                            //incorrectly set when more than 1 button was pressed. This is the fix -CSS
//			}
//			if((last_tx[0] != Z0_OFF) && (z0_off == false)){
////				z0_on = true;
//			}
//		}
//		else{
//			z0_de = DEBOUNCE;
//            z0_sent = false;
//			if(last_tx[0] == Z0_ON){
////				z0_off = true;
//			}
//			if(last_tx[0] == Z0_OFF){
////				z0_on = true;
//			}
//		}
//		//zone1 enables.
//		if(ZONE1_BTN){				
//			if(!(--z1_de) && !pwr_on){
//				if(z1_on && !z1_sent){
//					tx_buffer = Z1_ON;
//					tx_flag = true;
//					ZONE1_LED = 0;
//					z1_on = false;
//                    z1_off = true;
//                    massage_timeout = LONG_TOUT;
//				}
//				else if(z1_off){
//					tx_buffer = Z1_OFF;
//					tx_flag = true;
//					ZONE1_LED = 1;
//					z1_off = false;
//                    z1_on = true;
//                    z1_sent = true;
//                    massage_timeout = LONG_TOUT;
//				}
//			}
//            if((last_tx[0] != Z1_ON) && (z1_on == false)){
////				z1_off = true;
//			}
//			if((last_tx[0] != Z1_OFF) && (z1_off == false)){
////				z1_on = true;
//			}
//		}
//		else{
//			z1_de = DEBOUNCE;
//            z1_sent = false;
//			if(last_tx[0] == Z1_ON){
////				z1_off = true;
//			}
//			if(last_tx[0] == Z1_OFF){
////				z1_on = true;
//			}
//		}	
//		//zone2 enables
//		if(ZONE2_BTN){	
//            if(!(--z2_de) && !pwr_on){
//                if(z2_on && !z2_sent){
//                    tx_buffer = Z2_ON;	
//                    tx_flag = true;
//                    ZONE2_LED = 0;
//                    z2_on = false;
//                    z2_off = true;
//                    massage_timeout = LONG_TOUT;
//                }
//                else if(z2_off){
//                    tx_buffer = Z2_OFF;
//                    tx_flag = true;
//                    ZONE2_LED = 1;
//                    z2_off = false;
//                    z2_on = true;
//                    z2_sent = true;
//                    massage_timeout = LONG_TOUT;
//                }
//            }
//            if((last_tx[0] != Z2_ON) && (z2_on == false)){
////                z2_off = true;
//            }
//            if((last_tx[0] != Z2_OFF) && (z2_off == false)){
////                z2_on = true;
//            }            
//		}
//		else{
//			z2_de = DEBOUNCE;
//            z2_sent = false;
//			if(last_tx[0] == Z2_ON){
////				z2_off = true;
//			}
//			if(last_tx[0] == Z2_OFF){
////				z2_on = true;
//			}
//		}	
//		//zone3 enables.
//		if(ZONE3_BTN){				
//			if(!(--z3_de) && !pwr_on){
//				if(z3_on && !z3_sent){
//					tx_buffer = Z3_ON;
//					tx_flag = true;
//					ZONE3_LED = 0;
//					z3_on = false;
//                    z3_off = true;
//                    massage_timeout = LONG_TOUT;
//				}
//				else if(z3_off){
//					tx_buffer = Z3_OFF;
//					tx_flag = true;
//					ZONE3_LED = 1;
//					z3_off = false;
//                    z3_on = true;
//                    z3_sent = true;
//                    massage_timeout = LONG_TOUT;
//				}
//			}
//            if((last_tx[0] != Z3_ON) && (z3_on == false)){
////				z3_off = true;
//			}
//			if((last_tx[0] != Z3_OFF) && (z3_off == false)){
////				z3_on = true;
//			}
//		}
//		else{
//			z3_de = DEBOUNCE;
//            z3_sent = false;
//			if(last_tx[0] == Z3_ON){
////				z3_off = true;
//			}
//			if(last_tx[0] == Z3_OFF){
////				z3_on = true;
//			}
//		}
//		//massage mode handling
//		if(MODE_BTN){				
//			if(!(--mode_de) && !pwr_on){            //bugged, if btn is held, will cycle modes, ok operation for now            
//                mode_state++;
//                if(mode_state > 3){                 //cycle through massage modes, reset after 3rd
//                    mode_state = 1;
//                }
//                switch(mode_state){
//                    case 1:
//                        tx_buffer = M_ALTERNATE;    //cycle through modes. Will remember mode after OFF command
//                        tx_flag = true;
//                        MODE1_LED = 0;
//                        MODE2_LED = 1;
//                        MODE3_LED = 1;
//                        massage_timeout = LONG_TOUT;
//                        break;
//                    case 2:
//                        tx_buffer = M_PULSE;
//                        tx_flag = true;
//                        MODE1_LED = 1;
//                        MODE2_LED = 0;
//                        MODE3_LED = 1;
//                        massage_timeout = LONG_TOUT;
//                        break;
//                    case 3:
//                        tx_buffer = M_WAVE;
//                        tx_flag = true;
//                        MODE1_LED = 1;
//                        MODE2_LED = 1;
//                        MODE3_LED = 0;
//                        massage_timeout = LONG_TOUT;
//                        break;
//                    default:
//                        mode_state = 0;      
//                }
//			}
//		}
//		else{
//			mode_de = DEBOUNCE;
//            
//		}				
		//heat enable
		if(HEAT_BTN){
            if(POWER_BTN){
            }
            else{
                if(!(--heat_de)){
                    if(heat_on && !heat_sent){
                        tx_buffer = HEAT_ON;
                        tx_flag = true;
                        HEAT_LED = 0;
                        heat_on = false;
                        heat_off = true;
                        heat_timeout = LONG_TOUT;
                    }
                    else if(heat_off){
                        tx_buffer = HEAT_OFF;
                        tx_flag = true;
                        HEAT_LED = 1;
                        heat_off = false;
                        heat_on = true;
                        heat_sent = true;
                    }
                }
            }
            if((last_tx[0] != HEAT_ON) && (heat_on == false)){
//				heat_off = true;
			}
			if((last_tx[0] != HEAT_OFF) && (heat_off == false)){
//				heat_on = true;
			}
		}
		else{
			heat_de = DEBOUNCE;
            heat_sent = false;
			if(last_tx[0] == HEAT_ON){
//				heat_off = true;
			}
			if(last_tx[0] == HEAT_OFF){
//				heat_on = true;
			}
		}
        
        if(POWER_BTN){						//enter only when button is pressed                    
            if(!(--power_de)){              //decrement debounce until zero, then execute btn press
                if(pwr_on && !pwr_sent){					//flag prevents multiple tx if button is held high
                    tx_buffer = MASSAGE_ON;	
                    tx_flag = true;			//allow transmission
                    hc_led_on();
                    pwr_on = false;			//set toggle off
                    pwr_off = true;
                    massage_timeout = LONG_TOUT;
//                    if(mode_state == 1){
//                        MODE1_LED = 0;
//                        MODE2_LED = 1;
//                        MODE3_LED = 1;
//                    }
//                    else if(mode_state == 2){
//                        MODE1_LED = 1;
//                        MODE2_LED = 0;
//                        MODE3_LED = 1;
//                    }
//                    else if(mode_state == 3){
//                        MODE1_LED = 1;
//                        MODE2_LED = 1;
//                        MODE3_LED = 0;
//                    }
                }
                else if(pwr_off){
                    tx_buffer = MASSAGE_OFF;
                    tx_flag = true;			//allow transmission
                    hc_led_off();
                    HEAT_LED = 1;
                    heat_on  = true;
                    heat_off = false;
                    pwr_off = false;		//set toggle off
                    pwr_on = true;
                    pwr_sent = true;
                    mode_state = 1;
                }
            }
		}
		else{
			power_de = DEBOUNCE;
            pwr_sent = false;
			if((last_tx[0]==MASSAGE_OFF)||(last_tx[0]==LUM_CLOSE)){//only allow another ON tx if last tx was not ON tx
				//pwr_on = true;
			}
			else if(last_tx[0] == MASSAGE_ON){	//only allow another off tx if last tx was not off tx
				//pwr_off = true;
			}			
  		}			
//		if((POWER_LED == 0)&&(ZONE0_LED == 1)&&(ZONE1_LED == 1)&&(ZONE2_LED == 1)&&(ZONE3_LED == 1)){
//			tx_buffer = MASSAGE_OFF;//check if all zones are off while pump is still on, 
//			tx_flag = true;			//turn off pump if all zones are off
//            hc_led_off();
//            HEAT_LED = 1;
//            heat_on  = true;
//            heat_off = false;
//            pwr_off = false;		//set toggle off
//            pwr_on = true;
//		}
//These control the relay operations. High/Low reads are handled in the TMR0 ISR
//with a flag indicating the state. the xx_op_sent will limit the amount of tx to
//a single tx per button press. This also operates on the assumption that while(1)
//is sufficiently fast to prevent the last_tx variable from being corrupted. 
//No debounce is needed because they are polled in the TMR0 overflow ISR. -CSS
		//LUMBAR OPEN
		if(lum_btn_op){					//flag set in TMR0 isr
			if(last_tx[0] == LUM_OPEN){	//check last transmission
				lum_op_sent = true;		//flag set when tx occurs to prevent multiple tx of same instruction
			}
			else{
				lum_op_sent = false;	//reset flag on successful tx
			}
			if(!lum_op_sent){
				tx_buffer = LUM_OPEN;	//only tx once 
                pwr_on = true;
				tx_flag = true;
                WDTCON = WDT_ACT;      //set WDT to 16s for actuator control
                SWDTEN = 0;
                hc_led_off();
                mode_state = 1;
			}
		}
		else{
			if(lum_op_sent){			//only send if open cmd was sent
				tx_buffer = LUM_HOLD;	//this will close the relay. Imperative that this is always sent
				tx_flag = true;	
                WDTCON = WDT_RESET;    //4s interval for soft-reset
                SWDTEN = 1;
				lum_op_sent = false;
                //mode_state = 1;
			}
		}
		//LUMBAR CLOSE
		if(lum_btn_cl){
			if(last_tx[0] == LUM_CLOSE){
				lum_cl_sent = true;
			}
			else{
				lum_cl_sent = false;
			}
			if(!lum_cl_sent){
				tx_buffer = LUM_CLOSE;
                pwr_on = true;
				tx_flag = true;                
//                WDTCON = WDT_ACT;      //set WDT to 16s for actuator control
                SWDTEN = 0;
                hc_led_off();
                mode_state = 1;
			}
		}
		else{
			if(lum_cl_sent){
				tx_buffer = LUM_HOLD;
				tx_flag = true;
//                WDTCON = WDT_RESET;    //4s interval for soft-reset
                SWDTEN = 1;
				lum_cl_sent = false;
			}
		}
		//HR OPEN
		if(hr_btn_op){
			if(last_tx[0] == HR_OPEN){
				hr_op_sent = true;
			}
			else{
				hr_op_sent = false;
			}
			if(!hr_op_sent){
				tx_buffer = HR_OPEN;
				tx_flag = true;              
//                WDTCON = WDT_ACT;      //set WDT to 16s for actuator control
                SWDTEN = 0;
			}
		}
		else{
			if(hr_op_sent){
				tx_buffer = HR_HOLD;
				tx_flag = true;
//                WDTCON = WDT_RESET;    //4s interval for soft-reset
                SWDTEN = 1;
				hr_op_sent = false;
			}
		}
		//HR CLOSE
		if(hr_btn_cl){
			if(last_tx[0] == HR_CLOSE){
				hr_cl_sent = true;
			}
			else{
				hr_cl_sent = false;
   			}
			if(!hr_cl_sent){
				tx_buffer = HR_CLOSE;
				tx_flag = true;                             
//                WDTCON = WDT_ACT;      //set WDT to 16s for actuator control
                SWDTEN = 0;
			}
		}
		else{
			if(hr_cl_sent){
				tx_buffer = HR_HOLD;
				tx_flag = true;
//                WDTCON = WDT_RESET;    //4s interval for soft-reset
                SWDTEN = 1;
				hr_cl_sent = false;
			}
		}
		//PR OPEN
		if(pr_btn_op){
			if(last_tx[0] == PR_OPEN){
				pr_op_sent = true;
			}
			else{
				pr_op_sent = false;
			}
			if(!pr_op_sent){
				tx_buffer = PR_OPEN;
				tx_flag = true;                                             
//                WDTCON = WDT_ACT;      //set WDT to 16s for actuator control
                SWDTEN = 0;
			}
		}
		else{
			if(pr_op_sent){
				tx_buffer = PR_HOLD;
				tx_flag = true;                
//                WDTCON = WDT_RESET;    //4s interval for soft-reset
                SWDTEN = 1;
				pr_op_sent = false;
			}
		}
		//PR CLOSE
		if(pr_btn_cl){
			if(last_tx[0] == PR_CLOSE){
				pr_cl_sent = true;
			}
			else{
				pr_cl_sent = false;
			}
			if(!pr_cl_sent){
				tx_buffer = PR_CLOSE;
				tx_flag = true;                                                             
//                WDTCON = WDT_ACT;      //set WDT to 16s for actuator control
                SWDTEN = 0;
			}
		}
		else{
			if(pr_cl_sent){
				tx_buffer = PR_HOLD;
				tx_flag = true;
//                WDTCON = WDT_RESET;    //4s interval for soft-reset
                SWDTEN = 1;
				pr_cl_sent = false;
			}
		}
        
        if(safety < OFF_COUNT){     //repeatedly sends commands to stop actuators. Hot fix to prevent motor from running constantly
            ++safety;               /*safety cannot be excessively small. It must be sufficiently small to be unnoticed if an actuator
                                     *does stay on, but too small will cause signal degradation. This should be linked to ARR_LENGTH in
                                     *some way. Currently, both arbitrarily defined. 
                                     */
        }
        else{
            safety = 0;
            if(LUM_OPEN_BTN || LUM_CLOSE_BTN || PR_OPEN_BTN || HR_OPEN_BTN || PR_CLOSE_BTN || HR_CLOSE_BTN){                
            }
            else{
                tx_buffer = ACT_OFF;
                tx_flag = true;
            }
        }
        
		//TRANSMISSION tx should only occur here
		//theory is that writing in only one place will reduce tx errors - CSS
		//also by having a single buffer holding tx info, last pressed btn takes priority - CSS
		if(tx_flag){            
            bool echo = true;
            do{
                int c = 0;
                for(unsigned int i = 0; i < NELEMS(last_tx); ++i){
                    putch(tx_buffer);
                    last_tx[i] = tx_buffer; //tx set amount of commands
                    __delay_us(5);          //maybe unnecessary 
                }
                for(unsigned int i = 0; i < NELEMS(rx_char); ++i){
                    rx_char[i] = getch();   //rx echo from JCT
                    CLRWDT();
                }
                for(unsigned int i = 0; i < NELEMS(last_tx); ++i){
                    if(last_tx[i] == rx_char[i]){
                        c++;                //verify command was correctly rx'ed
                    }
                }
                if(c == ARR_LENGTH){        //decrease ARR_LENGTH for faster signal tx, increase for better signal integrity
                    echo = false;           //exit while
                    putch(OP_GO);           //send ACK to JCT to execute command
                }                
            }while(echo);
            tx_flag = false;      
		}
	}//end while(1)
}//end main

static void interrupt isr(void){
	if(TMR0IF){ //timer0 set to poll buttons every 10 microsec, control with global flags
//        if(POWER_BTN){
//            if(LUM_CLOSE_BTN || PR_OPEN_BTN || HR_OPEN_BTN || PR_CLOSE_BTN || HR_CLOSE_BTN ||
//                    ZONE0_BTN || ZONE1_BTN || ZONE2_BTN || ZONE3_BTN || HEAT_BTN || MODE_BTN || LUM_OPEN_BTN){
//			}
//            else{
//                pwr_btn_flg = true;
//            }
//        }
//        else{
//            pwr_btn_flg = false;
//        }
        
		if(LUM_OPEN_BTN){
			if(LUM_CLOSE_BTN || PR_OPEN_BTN || HR_OPEN_BTN || PR_CLOSE_BTN || HR_CLOSE_BTN || HEAT_BTN || POWER_BTN){
			}
			else{
				lum_btn_op = true;
			}
		}
		else{
			lum_btn_op = false;
		}
        
		if(LUM_CLOSE_BTN){
			if(LUM_OPEN_BTN || PR_OPEN_BTN || HR_OPEN_BTN || PR_CLOSE_BTN || HR_CLOSE_BTN || HEAT_BTN || POWER_BTN){
			}
			else{
				lum_btn_cl = true;
			}
		}
		else{
			lum_btn_cl = false;
		}
        
		if(PR_OPEN_BTN){
			if(PR_CLOSE_BTN || HR_OPEN_BTN || HR_CLOSE_BTN || LUM_OPEN_BTN || LUM_CLOSE_BTN || HEAT_BTN || POWER_BTN){
			}
			else{
				pr_btn_op = true;
			}
		}
		else{
			pr_btn_op = false;
		}
        
		if(PR_CLOSE_BTN){
			if(PR_OPEN_BTN || HR_OPEN_BTN || HR_CLOSE_BTN || LUM_OPEN_BTN || LUM_CLOSE_BTN || HEAT_BTN || POWER_BTN){
			}
			else{
				pr_btn_cl = true;
			}
		}
		else{
			pr_btn_cl = false;
		}
        
		if(HR_OPEN_BTN){
			if(HR_CLOSE_BTN || PR_OPEN_BTN || PR_CLOSE_BTN || LUM_OPEN_BTN || LUM_CLOSE_BTN || HEAT_BTN || POWER_BTN){
			}
			else{
				hr_btn_op = true;
			}
		}
		else{
			hr_btn_op = false;
		}
        
		if(HR_CLOSE_BTN){
			if(HR_OPEN_BTN || PR_OPEN_BTN || PR_CLOSE_BTN || LUM_OPEN_BTN || LUM_CLOSE_BTN || HEAT_BTN || POWER_BTN){
			}
			else{
				hr_btn_cl = true;
			}
		}
		else{
			hr_btn_cl = false;
		}
        
        if(!(--time_out)){
            time_out = SEC_RELOAD;
            if(massage_timeout){
                if(!(--massage_timeout)){
                    hc_led_off();
                    pwr_on = true;
                    mode_state = 1;
                }
            }
            if(heat_timeout){
                if(!(--heat_timeout)){
                    HEAT_LED = 1;
                    heat_on = true;
                    heat_off = false;                          
                }
            }
        }        
	}    
	TMR0IF = 0;
}

void hc_led_on(){
    POWER_LED = 0;			//0V > sink current > LED on
//	ZONE0_LED = 0;			//turn on all LEDs when power btn pressed
	z0_off = true;			//set flags for individual zone control
	z0_on = false;          //change this to function 
//	ZONE1_LED = 0;
	z1_off = true;
	z1_on = false;
//	ZONE2_LED = 0;
	z2_off = true;
	z2_on = false;
//	ZONE3_LED = 0;
	z3_off = true;
	z3_on = false;	
}

void hc_led_off(){
    POWER_LED = 1;			//5V > no current > LED off
//	MODE1_LED = 1;          //change this to function
//	MODE2_LED = 1;
//	MODE3_LED = 1;
//	ZONE0_LED = 1;
	z0_on  = true;
	z0_off = false;
//	ZONE1_LED = 1;
	z1_on  = true;
	z1_off = false;
//	ZONE2_LED = 1;
	z2_on  = true;
	z2_off = false;
//	ZONE3_LED = 1;
	z3_on  = true;
	z3_off = false;
}

bool syn_synack_ack(){
    bool echo = true;
    do{
        putch(SYNC);
        char c = getch();
        CLRWDT();
        if(c == SYNC_ACK){
            putch(ACK);
            echo = false;                    
        }
    }while(echo);
    
    return true;
        
}
        

void connection_est(){
    for(int i = 0; i < 3; ++i){
        hc_led_on();
        __delay_ms(150);
        hc_led_off();
        __delay_ms(150);
    }    
}