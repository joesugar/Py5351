/*
This is the part of the TrinketI2C code that is usually written in C.
It's taken from the I2C Stick project and updaed to build using the
Arduino IDE.

Copyright (c) 2015 Joseph Consugar
All rights reserved.

TrinketI2C is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of
the License, or (at your option) any later version.

TrinketI2C is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with TrinketI2C. If not, see
<http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <ctype.h>
#include <string.h>

#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/power.h>

#include <util/delay.h>
#include <stdint.h>

#include "cmdline_defs.h"
#include "TrinketI2CC.h"
#include "usbconfig.h"
#include "usbdrv/usbdrv.h"

#define ENABLE_SCL_EXPAND

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

#define WHITE_LED  3
#define YELLOW_LED 1

#ifndef NULL
#define NULL    ((void *)0)
#endif

#define STATUS_IDLE          0
#define STATUS_ADDRESS_ACK   1
#define STATUS_ADDRESS_NAK   2

/* commands from USB, must e.g. match command ids in kernel driver */
#define CMD_ECHO       8
#define CMD_GET_FUNC   9
#define CMD_SET_DELAY  10
#define CMD_GET_STATUS 11

/* I2C commands */
#define CMD_I2C_IO     4
#define CMD_I2C_BEGIN  1        // flag for I2C_IO
#define CMD_I2C_END    2        // flag for I2C_IO

/* linux kernel flags */
#define I2C_M_TEN		    0x10	/* we have a ten bit chip address */
#define I2C_M_RD		    0x01
#define I2C_M_NOSTART		0x4000
#define I2C_M_REV_DIR_ADDR	0x2000
#define I2C_M_IGNORE_NAK	0x1000
#define I2C_M_NO_RD_ACK		0x0800

/* To determine what functionality is present */
#define I2C_FUNC_I2C			            0x00000001
#define I2C_FUNC_10BIT_ADDR		            0x00000002
#define I2C_FUNC_PROTOCOL_MANGLING	        0x00000004 /* I2C_M_{REV_DIR_ADDR,NOSTART,..} */
#define I2C_FUNC_SMBUS_HWPEC_CALC	        0x00000008 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_READ_WORD_DATA_PEC   0x00000800 /* SMBus 2.0 */ 
#define I2C_FUNC_SMBUS_WRITE_WORD_DATA_PEC  0x00001000 /* SMBus 2.0 */ 
#define I2C_FUNC_SMBUS_PROC_CALL_PEC	    0x00002000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_BLOCK_PROC_CALL_PEC  0x00004000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_BLOCK_PROC_CALL	    0x00008000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_QUICK		        0x00010000 
#define I2C_FUNC_SMBUS_READ_BYTE	        0x00020000 
#define I2C_FUNC_SMBUS_WRITE_BYTE	        0x00040000 
#define I2C_FUNC_SMBUS_READ_BYTE_DATA	    0x00080000 
#define I2C_FUNC_SMBUS_WRITE_BYTE_DATA	    0x00100000 
#define I2C_FUNC_SMBUS_READ_WORD_DATA	    0x00200000 
#define I2C_FUNC_SMBUS_WRITE_WORD_DATA	    0x00400000 
#define I2C_FUNC_SMBUS_PROC_CALL	        0x00800000 
#define I2C_FUNC_SMBUS_READ_BLOCK_DATA	    0x01000000 
#define I2C_FUNC_SMBUS_WRITE_BLOCK_DATA     0x02000000 
#define I2C_FUNC_SMBUS_READ_I2C_BLOCK	    0x04000000 /* I2C-like block xfer  */
#define I2C_FUNC_SMBUS_WRITE_I2C_BLOCK	    0x08000000 /* w/ 1-byte reg. addr. */
#define I2C_FUNC_SMBUS_READ_I2C_BLOCK_2	    0x10000000 /* I2C-like block xfer  */
#define I2C_FUNC_SMBUS_WRITE_I2C_BLOCK_2    0x20000000 /* w/ 2-byte reg. addr. */
#define I2C_FUNC_SMBUS_READ_BLOCK_DATA_PEC  0x40000000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_WRITE_BLOCK_DATA_PEC 0x80000000 /* SMBus 2.0 */

#define I2C_FUNC_SMBUS_BYTE I2C_FUNC_SMBUS_READ_BYTE | \
                            I2C_FUNC_SMBUS_WRITE_BYTE
#define I2C_FUNC_SMBUS_BYTE_DATA I2C_FUNC_SMBUS_READ_BYTE_DATA | \
                                 I2C_FUNC_SMBUS_WRITE_BYTE_DATA
#define I2C_FUNC_SMBUS_WORD_DATA I2C_FUNC_SMBUS_READ_WORD_DATA | \
                                 I2C_FUNC_SMBUS_WRITE_WORD_DATA
#define I2C_FUNC_SMBUS_BLOCK_DATA I2C_FUNC_SMBUS_READ_BLOCK_DATA | \
                                  I2C_FUNC_SMBUS_WRITE_BLOCK_DATA
#define I2C_FUNC_SMBUS_I2C_BLOCK I2C_FUNC_SMBUS_READ_I2C_BLOCK | \
                                  I2C_FUNC_SMBUS_WRITE_I2C_BLOCK

#define I2C_FUNC_SMBUS_EMUL I2C_FUNC_SMBUS_QUICK | \
                            I2C_FUNC_SMBUS_BYTE | \
                            I2C_FUNC_SMBUS_BYTE_DATA | \
                            I2C_FUNC_SMBUS_WORD_DATA | \
                            I2C_FUNC_SMBUS_PROC_CALL | \
                            I2C_FUNC_SMBUS_WRITE_BLOCK_DATA | \
                            I2C_FUNC_SMBUS_WRITE_BLOCK_DATA_PEC | \
                            I2C_FUNC_SMBUS_I2C_BLOCK

/* the currently support capability is quite limited */
const unsigned long func PROGMEM = I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;

#define I2C_PORT   PORTB
#define I2C_PIN    PINB
#define I2C_DDR    DDRB
#define I2C_SDA    _BV(0)       // PB0
#define I2C_SCL    _BV(2)       // PB2

#define DEFAULT_DELAY 10        // default 10us (100khz)

static uchar status = STATUS_IDLE;
static unsigned short expected;
static unsigned char saved_cmd;
static unsigned short clock_delay  = DEFAULT_DELAY;
static unsigned short clock_delay2 = DEFAULT_DELAY/2;

struct i2c_cmd {
    unsigned char type;
    unsigned char cmd;
    unsigned short flags;
    unsigned short addr;
    unsigned short len;  
};


#if defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny25__)
/* ------------------------------------------------------------------------- */
/* ------------------------ Oscillator Calibration ------------------------- */
/* ------------------------------------------------------------------------- */
// section copied from EasyLogger
/* Calibrate the RC oscillator to 8.25 MHz. The core clock of 16.5 MHz is
 * derived from the 66 MHz peripheral clock by dividing. Our timing reference
 * is the Start Of Frame signal (a single SE0 bit) available immediately after
 * a USB RESET. We first do a binary search for the OSCCAL value and then
 * optimize this value with a neighboorhod search.
 * This algorithm may also be used to calibrate the RC oscillator directly to
 * 12 MHz (no PLL involved, can therefore be used on almost ALL AVRs), but this
 * is wide outside the spec for the OSCCAL value and the required precision for
 * the 12 MHz clock! Use the RC oscillator calibrated to 12 MHz for
 * experimental purposes only!
 */
void calibrateOscillator(void)
{
    uchar       step = 128;
    uchar       trialValue = 0, optimumValue;
    int         x, optimumDev, targetValue = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);

    /* Do a binary search: 
     */
    do{
        OSCCAL = trialValue + step;
        x = usbMeasureFrameLength();    /* proportional to current real frequency */
        if(x < targetValue)             /* frequency still too low */
            trialValue += step;
        step >>= 1;
    } while(step > 0);
    
    /* We have a precision of +/- 1 for optimum OSCCAL here
     * now do a neighborhood search for optimum value 
     */
    optimumValue = trialValue;
    optimumDev = x; /* this is certainly far away from optimum */
    for (OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++){
        x = usbMeasureFrameLength() - targetValue;
        if(x < 0)
            x = -x;
        if(x < optimumDev){
            optimumDev = x;
            optimumValue = OSCCAL;
        }
    }
    OSCCAL = optimumValue;
}
/*
Note: This calibration algorithm may try OSCCAL values of up to 192 even if
the optimum value is far below 192. It may therefore exceed the allowed clock
frequency of the CPU in low voltage designs!
You may replace this search algorithm with any other algorithm you like if
you have additional constraints such as a maximum CPU clock.
For version 5.x RC oscillators (those with a split range of 2x128 steps, e.g.
ATTiny25, ATTiny45, ATTiny85), it may be useful to search for the optimum in
both regions.
*/
#endif


/*
 * I2C initialization.
 */
static void i2c_init(void) 
{
	/* For DDR, logic 1 = out, logic 0 = in. */
    /* tri-state the sda/scl pins */
    I2C_DDR &= ~I2C_SDA;                // write logic 0 to make it an input.
    I2C_PORT |= I2C_SDA;                // write logic 1 to enable the pullup.
    
    #ifdef ENABLE_SCL_EXPAND
        I2C_DDR &= ~I2C_SCL;            // write logic 0 to make it an input.
        I2C_PORT |= I2C_SCL;            // write logic 1 to enable the pullup.
    #else
        I2C_DDR |= I2C_SCL;             // port is output
    #endif

    /* no bytes to be expected */
    expected = 0;
}

/*
 * Set the I2C data pin
 */
static void i2c_io_set_sda(uchar hi) 
{
    if(hi) 
    {
		/* Tri-state the data line, allowing the pullup to pull it hi. */
        I2C_DDR  &= ~I2C_SDA;           // write logic 0 to make it an input.
        I2C_PORT |=  I2C_SDA;           // write logic 1 to enable the pullup.
    } 
	else 
	{
		/* Make the pin an output and drive it lo. */
        I2C_DDR  |=  I2C_SDA;           // write logic 1 to make it an output.
        I2C_PORT &= ~I2C_SDA;           // write logic 0 to pull it low.
    }
}

/*
 * Get the I2C data pin
 */
static uchar i2c_io_get_sda(void) 
{
	/* Read the input port and mask it with the data mask */
    return(I2C_PIN & I2C_SDA);
}

/*
 * Set the I2C serial clock pin.
 */
static void i2c_io_set_scl(uchar hi) 
{
    #ifdef ENABLE_SCL_EXPAND
        _delay_loop_2(clock_delay2);
        if (hi) 
        {
            /* Tri-state the data line, allowing the pullup to pull it hi. */
            I2C_DDR &= ~I2C_SCL;        // port is input
            I2C_PORT |= I2C_SCL;        // enable pullup

            /* wait until the pin goes hi if it is being pulled low by client */
            while(!(I2C_PIN & I2C_SCL));
        } 
        else 
        {
			/* Make the pin an output and drive it lo. */
            I2C_DDR |= I2C_SCL;         // port is output
            I2C_PORT &= ~I2C_SCL;       // drive it low
        }
        _delay_loop_2(clock_delay);
    #else
        _delay_loop_2(clock_delay2);
        if(hi) 
        {
			/* Drive the pin hi */
            I2C_PORT |=  I2C_SCL;       // port is high
        }            
        else
        {
			/* Drive the pin lo */
            I2C_PORT &= ~I2C_SCL;       // port is low
        }            
        _delay_loop_2(clock_delay);
    #endif
}

/*
 * clock HI, delay, then LO
 */
static void i2c_scl_toggle(void) 
{
    i2c_io_set_scl(1);
    i2c_io_set_scl(0);
}

/*
 * i2c start condition
 */
static void i2c_start(void) 
{
    i2c_io_set_sda(0);
    i2c_io_set_scl(0);
}

/*
 * i2c repeated start condition
 */
static void i2c_repstart(void) 
{
    /* set scl, sda high in case they are not already there. */
    i2c_io_set_sda(1);
    i2c_io_set_scl(1);
  
    i2c_io_set_sda(0);
    i2c_io_set_scl(0);
}

/*
 * i2c stop condition 
 */
void i2c_stop(void) 
{
    i2c_io_set_sda(0);
    i2c_io_set_scl(1);
    i2c_io_set_sda(1);
}

/*
 * Send a byte over the I2C channel.
 */
uchar i2c_put_u08(uchar b) 
{
    char i;

    // Send out each of the bits.
	uchar mask = 0x80;
    for (i = 0; i < 8; ++i) 
    {
        if ( b & mask )  
            i2c_io_set_sda(1);
        else
            i2c_io_set_sda(0);
        i2c_scl_toggle();           // clock HI, delay, then LO
		mask >>= 1;
    }
  
    // Look for the ACK bit.
    i2c_io_set_sda(1);              // leave SDL HI
    i2c_io_set_scl(1);              // clock back up

    b = i2c_io_get_sda();           // get the ACK bit
    i2c_io_set_scl(0);              // not really ??

    //return(b == 0);               // return ACK value
	return (b ^ I2C_SDA);           // if SDA clear return true, otherwise return false.
}

/*
 * Get a byte from the I2C channel. 
 */
uchar i2c_get_u08(uchar last) 
{
    char i;
    uchar c,b = 0;

    // Initialize bits.
    i2c_io_set_sda(1);            // make sure pullups are activated
    i2c_io_set_scl(0);            // clock LOW

    // Read in each of the bits.
    for(i = 0; i < 8; ++i) 
    {
        i2c_io_set_scl(1);        // clock HI
        c = i2c_io_get_sda();
        b <<= 1;
        if (c) 
            b |= 1;
        i2c_io_set_scl(0);        // clock LO
    }

    // Set the ACK/NAK
    if(last) 
        i2c_io_set_sda(1);        // set NAK
    else     
        i2c_io_set_sda(0);        // set ACK

    // Set lines for return.
    i2c_scl_toggle();             // clock pulse
    i2c_io_set_sda(1);            // leave with SDA HI
    return b;                     // return received byte
}

/*
 * Execute the beginning of an I2C transaction.
 */
static uchar i2c_do(struct i2c_cmd *cmd) 
{
    // Initialize.
    uchar addr;

    // Create read/write address from normal 7-bit address.
    addr = ( cmd->addr << 1 );
    if (cmd->flags & I2C_M_RD )
        addr |= 1;

    // Call the appropriate command start routine.
    if(cmd->cmd & CMD_I2C_BEGIN) 
        i2c_start();
    else 
        i2c_repstart();    

    // Send DEVICE address
    if (!i2c_put_u08(addr)) 
    {
        // DEVICE address error.
        status = STATUS_ADDRESS_NAK;
        expected = 0;
        i2c_stop();
    } 
    else 
    {  
        // DEVICE address successfully sent.
        status = STATUS_ADDRESS_ACK;
        expected = cmd->len;
        saved_cmd = cmd->cmd;

        // Check if transfer is already done (or failed) 
        if((cmd->cmd & CMD_I2C_END) && !expected) 
            i2c_stop();
    }

    // More data to be expected?
    return (cmd->len ? 0xff : 0x00);
}

/*
 * Scan an address to see if a device is there.
 * Only sends an address and sets the acknowledgment state.
 */
static void i2c_scan_address(struct i2c_cmd *cmd) 
{
    // Initialize.
    unsigned short addr;

    // Create read/write address from normal 7-bit address.
    addr = ( cmd->addr << 1 );
    if (cmd->flags & I2C_M_RD )
        addr |= 1;

    // Call the start routine.
    i2c_start();   
	status = (i2c_put_u08(addr)) ? STATUS_ADDRESS_ACK : STATUS_ADDRESS_NAK;
	expected = 0;
	i2c_stop();
	return;
}


/*
 * usbFunctionSetup()
 * Called when a class or vendor request is made.
 */
uchar usbFunctionSetup(uchar data[8]) 
{
    static uchar replyBuf[4];
    usbMsgPtr = (usbMsgPtr_t)replyBuf;

    switch(data[1]) 
    {
        case CMD_ECHO:
            // Echo the sent value (for transfer reliability testing)
            replyBuf[0] = data[2];
            replyBuf[1] = data[3];
            return 2;
            break;
        case CMD_GET_FUNC:
            // Get a pointer to the variable containing interface functionality.
            memcpy_P(replyBuf, &func, sizeof(func));
            return sizeof(func);
            break;
        case CMD_SET_DELAY:
            // The delay function used delays 4 system ticks per cycle. 
            // This gives 1/3us at 12Mhz per cycle. The delay function is 
            // called twice per clock edge and thus four times per full cycle.  
            // Thus it is called one time per edge with the full delay  
            // value and one time with the half one. Resulting in 
            // 2 * n * 1/3 + 2 * 1/2 n * 1/3 = n us. 
			// For the RC oscillator operating at 8.25 MHz the delay will
			// be about 50% longer.
            clock_delay = *(unsigned short*)(data+2);
            if(!clock_delay) 
			    clock_delay = 1;
            clock_delay2 = clock_delay/2;
            if(!clock_delay2) 
                clock_delay2 = 1;
            break;
        case CMD_I2C_IO:
        case CMD_I2C_IO + CMD_I2C_BEGIN:
        case CMD_I2C_IO                 + CMD_I2C_END:
        case CMD_I2C_IO + CMD_I2C_BEGIN + CMD_I2C_END:
		    // Transfer data on the I2C bus.
            // These are only allowed as class transfers
            return i2c_do((struct i2c_cmd*)data);
            break;
		case CMD_I2C_BEGIN + CMD_I2C_END:
		    // Probe the address and set the acknowledgment flag.
			// Calling program will check the acknowledgment flag
			// to see if the address is valid.
			// This only allowed as a class transfer.
		    i2c_scan_address((struct i2c_cmd *)data);
            return 0;
		    break;
        case CMD_GET_STATUS:
		    // Retrieve the status flag.
            replyBuf[0] = status;
            return 1;
            break;
        default:
            // Ignore unrecognized functions.
            break;
    }
    return 0;
}


/*
 * usbFunctionWrite()
 * Write data from USB to the I2C bus.
 */
uchar usbFunctionWrite( uchar *data, uchar len )
{
    uchar i, err=0;

    if(status == STATUS_ADDRESS_ACK) 
    {
        // In the ACK state.
        if(len > expected) 
        {
            len = expected;
        }

        // consume bytes
        for (i = 0; i < len; i++) 
        {
            expected--;
            if(!i2c_put_u08(*data++))
	            err = 1;
        }

        // end transfer on last byte
        if((saved_cmd & CMD_I2C_END) && !expected) 
            i2c_stop();

        if(err) 
        {
            //TODO: set status
        }
    } 
    else 
    {
        // Not in ACK state.
        memset(data, 0, len);
    }
    return len;
}


/* 
 * usbFunctionRead
 * Read data from the I2C bus to send back via USB.
 */
uchar usbFunctionRead( uchar *data, uchar len )
{
    uchar i;

    if (status == STATUS_ADDRESS_ACK) 
    {
		// In the ACK state.
		// Don't exceed the expected number of bytes.
        if(len > expected) 
        {
            len = expected;
        }

        // consume bytes
        for(i = 0; i < len; i++) 
        {
            expected--;
            *data = i2c_get_u08(expected == 0);
            data++;
        }

        // end transfer on last byte
        if((saved_cmd & CMD_I2C_END) && !expected) 
            i2c_stop();
    } 
    else 
    {
		// Not in the ACK state.
        memset(data, 0, len);
    }
    return len;
}


/*
 * Calibrate the address each time the interface resets.
 */
void usbEventResetReady(void)
{
	// Store the calibrated value in EEPROM
    calibrateOscillator();
    eeprom_write_byte(0, OSCCAL);
}


/*
 * Main routine.
 */
void usbBegin()
{
    uchar calibrationValue;
	
    /* Calibration value from last time.
     */
    calibrationValue = eeprom_read_byte(0); 
    if (calibrationValue != 0xff)
        OSCCAL = calibrationValue;
    
    /* Clear any pending interrupts.
     */
	cli();

	/* Run at full speed, because Trinket defaults to 8MHz for low voltage 
     * compatibility reasons.
     */
	clock_prescale_set(clock_div_1);
    
    /* Enable the watchdog timer.
     */
    wdt_enable(WDTO_1S);

    /* I2C initialize.
     */
    i2c_init();
    
	/* Fake a disconnect to force the computer to re-enumerate.
     */
	PORTB &= ~(_BV(USB_CFG_DMINUS_BIT) | _BV(USB_CFG_DPLUS_BIT));
	usbDeviceDisconnect();
	_delay_ms(300);
	usbDeviceConnect();

    /* Initialize USB comm.
     */
	usbInit();		
    
    /* Enable interrupts.
     */
	sei();
    
	/* Main event loop.
     */
    for(;;)
	{    
		/* Check to see if it's time to send a USB packet.
         */
		wdt_reset();
        usbPoll();			
    }
}
