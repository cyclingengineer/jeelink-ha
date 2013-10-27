/// @file
/// RFM12B driver implementation
// Acknowledgements & License:
// A large portion of this software is based on Jeelib (see https://github.com/jcw/jeelib) by JC Wippler et al.
// This derivative continues to be published under http://opensource.org/licenses/mit-license.php

#include "JeelinkHaRF12.h"
#include <avr/io.h>
#include <util/crc16.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#if ARDUINO >= 100
#include <Arduino.h> // Arduino 1.0
#else
#include <WProgram.h> // Arduino 0022
#endif

// #define OPTIMIZE_SPI 1  // uncomment this to write to the RFM12B @ 8 Mhz

// pin change interrupts are currently only supported on ATmega328's
// #define PINCHG_IRQ 1    // uncomment this to use pin-change interrupts

// maximum transmit / receive buffer: data bytes
#define RF_MAX   (RF12_MAXDATA)

// pins used for the RFM12B interface - yes, there *is* logic in this madness:
//
//  - leave RFM_IRQ set to the pin which corresponds with INT0, because the
//    current driver code will use attachInterrupt() to hook into that
//  - (new) you can now change RFM_IRQ, if you also enable PINCHG_IRQ - this
//    will switch to pin change interrupts instead of attach/detachInterrupt()
//  - use SS_DDR, SS_PORT, and SS_BIT to define the pin you will be using as
//    select pin for the RFM12B (you're free to set them to anything you like)
//  - please leave SPI_SS, SPI_MOSI, SPI_MISO, and SPI_SCK as is, i.e. pointing
//    to the hardware-supported SPI pins on the ATmega, *including* SPI_SS !

#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)

#define RFM_IRQ     2
#define SS_DDR      DDRB
#define SS_PORT     PORTB
#define SS_BIT      0

#define SPI_SS      53    // PB0, pin 19
#define SPI_MOSI    51    // PB2, pin 21
#define SPI_MISO    50    // PB3, pin 22
#define SPI_SCK     52    // PB1, pin 20

#elif defined(__AVR_ATmega644P__)

#define RFM_IRQ     10
#define SS_DDR      DDRB
#define SS_PORT     PORTB
#define SS_BIT      4

#define SPI_SS      4
#define SPI_MOSI    5
#define SPI_MISO    6
#define SPI_SCK     7

#elif defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny44__)

#define RFM_IRQ     2
#define SS_DDR      DDRB
#define SS_PORT     PORTB
#define SS_BIT      1

#define SPI_SS      1     // PB1, pin 3
#define SPI_MISO    4     // PA6, pin 7
#define SPI_MOSI    5     // PA5, pin 8
#define SPI_SCK     6     // PA4, pin 9

#elif defined(__AVR_ATmega32U4__) //Arduino Leonardo 

#define RFM_IRQ     0	    // PD0, INT0, Digital3 
#define SS_DDR      DDRB
#define SS_PORT     PORTB
#define SS_BIT      6	    // Dig10, PB6

#define SPI_SS      17    // PB0, pin 8, Digital17
#define SPI_MISO    14    // PB3, pin 11, Digital14
#define SPI_MOSI    16    // PB2, pin 10, Digital16
#define SPI_SCK     15    // PB1, pin 9, Digital15

#else

// ATmega168, ATmega328, etc.
#define RFM_IRQ     2
#define SS_DDR      DDRB
#define SS_PORT     PORTB
#define SS_BIT      2     // for PORTB: 2 = d.10, 1 = d.9, 0 = d.8

#define SPI_SS      10    // PB2, pin 16
#define SPI_MOSI    11    // PB3, pin 17
#define SPI_MISO    12    // PB4, pin 18
#define SPI_SCK     13    // PB5, pin 19

#endif 

// RF12 command codes
#define RF_RECEIVER_ON  0x82DD
#define RF_XMITTER_ON   0x823D
#define RF_IDLE_MODE    0x820D
#define RF_SLEEP_MODE   0x8205
#define RF_WAKEUP_MODE  0x8207
#define RF_TXREG_WRITE  0xB800
#define RF_RX_FIFO_READ 0xB000
#define RF_WAKEUP_TIMER 0xE000

// RF12 status bits
#define RF_LBD_BIT      0x0400
#define RF_RSSI_BIT     0x0100

// bits in the node id configuration byte
#define NODE_BAND       0xC0        // frequency band
#define NODE_ACKANY     0x20        // ack on broadcast packets if set
#define NODE_ID         0x1F        // id of this node, as A..Z or 1..31

// transceiver states, these determine what to do with each interrupt
enum {
    TXTAIL, TXDONE, TXIDLE,    
    TXPRE1, TXPRE2, TXPRE3, TXSYN1, TXSYN2,
};

static uint8_t cs_pin = SS_BIT;     // chip select pin

static uint8_t nodeid;              // address of this node
static uint8_t group;               // network group
static volatile uint8_t rxfill;     // number of data bytes in rf12_buf
static volatile int8_t rxstate;     // current transceiver state
static volatile uint8_t rf12_len;   // length of the data

#define RETRIES     8               // stop retrying after 8 times
#define RETRY_MS    1000            // resend packet every second until ack'ed

volatile uint8_t rf12_buf[RF_MAX];  // recv/xmit buf

				    // function to set chip select pin from within sketch
void rf12_set_cs(uint8_t pin)
{
#if defined(__AVR_ATmega32U4__)     //Arduino Leonardo 
  if (pin==10) cs_pin=6; 	    // Dig10, PB6     
  if (pin==9)  cs_pin=5; 	    // Dig9,  PB5	
  if (pin==8)  cs_pin=4; 	    // Dig8,  PB4            
#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__) || defined (__AVR_ATmega328P__) // ATmega168, ATmega328
  if (pin==10) cs_pin = 2; 	    // Dig10, PB2
  if (pin==9) cs_pin = 1;  	    // Dig9,  PB1
  if (pin==8) cs_pin = 0;  	    // Dig8,  PB0
#endif
}


void rf12_spiInit () {
    bitSet(SS_PORT, cs_pin);
    bitSet(SS_DDR, cs_pin);
    digitalWrite(SPI_SS, 1);
    pinMode(SPI_SS, OUTPUT);
    pinMode(SPI_MOSI, OUTPUT);
    pinMode(SPI_MISO, INPUT);
    pinMode(SPI_SCK, OUTPUT);
#ifdef SPCR    
    SPCR = _BV(SPE) | _BV(MSTR);
#if F_CPU > 10000000
    // use clk/2 (2x 1/4th) for sending (and clk/8 for recv, see rf12_xferSlow)
    SPSR |= _BV(SPI2X);
#endif
#else
    // ATtiny
    USICR = bit(USIWM0);
#endif    
    pinMode(RFM_IRQ, INPUT);
    digitalWrite(RFM_IRQ, 1); // pull-up
}

static uint8_t rf12_byte (uint8_t out) {
#ifdef SPDR
    SPDR = out;
    // this loop spins 4 usec with a 2 MHz SPI clock
    while (!(SPSR & _BV(SPIF)))
        ;
    return SPDR;
#else
    // ATtiny
    USIDR = out;
    byte v1 = bit(USIWM0) | bit(USITC);
    byte v2 = bit(USIWM0) | bit(USITC) | bit(USICLK);
#if F_CPU <= 5000000
    // only unroll if resulting clock stays under 2.5 MHz
    USICR = v1; USICR = v2;
    USICR = v1; USICR = v2;
    USICR = v1; USICR = v2;
    USICR = v1; USICR = v2;
    USICR = v1; USICR = v2;
    USICR = v1; USICR = v2;
    USICR = v1; USICR = v2;
    USICR = v1; USICR = v2;
#else
    for (uint8_t i = 0; i < 8; ++i) {
        USICR = v1;
        USICR = v2;
    }
#endif
    return USIDR;
#endif
}

static uint16_t rf12_xferSlow (uint16_t cmd) {
    // slow down to under 2.5 MHz
#if F_CPU > 10000000
    bitSet(SPCR, SPR0);
#endif
    bitClear(SS_PORT, cs_pin);
    uint16_t reply = rf12_byte(cmd >> 8) << 8;
    reply |= rf12_byte(cmd);
    bitSet(SS_PORT, cs_pin);
#if F_CPU > 10000000
    bitClear(SPCR, SPR0);
#endif
    return reply;
}

#if OPTIMIZE_SPI
static void rf12_xfer (uint16_t cmd) {
    // writing can take place at full speed, even 8 MHz works
    bitClear(SS_PORT, cs_pin);
    rf12_byte(cmd >> 8) << 8;
    rf12_byte(cmd);
    bitSet(SS_PORT, cs_pin);
}
#else
#define rf12_xfer rf12_xferSlow
#endif

/// @details
/// This call provides direct access to the RFM12B registers. If you're careful
/// to avoid configuring the wireless module in a way which stops the driver
/// from functioning, this can be used to adjust frequencies, power levels,
/// RSSI threshold, etc. See the RFM12B wireless module documentation.
///
/// This call will briefly disable interrupts to avoid clashes on the SPI bus.
///
/// Returns the 16-bit value returned by SPI. Probably only useful with a 
/// "0x0000" status poll command.
/// @param cmd RF12 command, topmost bits determines which register is affected.
uint16_t rf12_control(uint16_t cmd) {
#ifdef EIMSK
#if PINCHG_IRQ
    #if RFM_IRQ < 8
        bitClear(PCICR, PCIE2);
    #elif RFM_IRQ < 14
        bitClear(PCICR, PCIE0);
    #else
        bitClear(PCICR, PCIE1);
    #endif
#else
    bitClear(EIMSK, INT0);
#endif
   uint16_t r = rf12_xferSlow(cmd);
#if PINCHG_IRQ
    #if RFM_IRQ < 8
        bitSet(PCICR, PCIE2);
    #elif RFM_IRQ < 14
        bitSet(PCICR, PCIE0);
    #else
        bitSet(PCICR, PCIE1);
    #endif
#else
    bitSet(EIMSK, INT0);
#endif
#else
    // ATtiny
    bitClear(GIMSK, INT0);
    uint16_t r = rf12_xferSlow(cmd);
    bitSet(GIMSK, INT0);
#endif
    return r;
}

static void rf12_interrupt() {
    // a transfer of 2x 16 bits @ 2 MHz over SPI takes 2x 8 us inside this ISR
    // correction: now takes 2 + 8 µs, since sending can be done at 8 MHz
    rf12_xfer(0x0000);
    
    uint8_t out;

    if (rxstate < 0) {
        uint8_t pos = rf12_len + rxstate++;
        out = rf12_buf[pos];
    } else
        switch (rxstate++) {
            case TXSYN1: out = 0x2D; break;
            case TXSYN2: out = group; rxstate = -rf12_len; break;
            case TXDONE: rf12_xfer(RF_IDLE_MODE); // fall through
            default:     out = 0xAA; //PRE1/2/3
        }
        
    rf12_xfer(RF_TXREG_WRITE + out);
}

#if PINCHG_IRQ
    #if RFM_IRQ < 8
        ISR(PCINT2_vect) {
            while (!bitRead(PIND, RFM_IRQ))
                rf12_interrupt();
        }
    #elif RFM_IRQ < 14
        ISR(PCINT0_vect) { 
            while (!bitRead(PINB, RFM_IRQ - 8))
                rf12_interrupt();
        }
    #else
        ISR(PCINT1_vect) {
            while (!bitRead(PINC, RFM_IRQ - 14))
                rf12_interrupt();
        }
    #endif
#endif

#include <RF12.h> 
#include <Ports.h> // needed to avoid a linker error :(

/// @details
/// Call this when you have some data to send. If it returns true, then you can
/// use rf12_sendStart() to start the transmission. Else you need to wait and
/// retry this call at a later moment.
///
/// Don't call this function if you have nothing to send, because rf12_canSend()
/// will stop reception when it returns true. IOW, calling the function
/// indicates your intention to send something, and once it returns true, you
/// should follow through and call rf12_sendStart() to actually initiate a send.
/// See [this weblog post](http://jeelabs.org/2010/05/20/a-subtle-rf12-detail/).
///
uint8_t rf12_canSend () {
    // need interrupts off to avoid a race (and enable the RFM12B, thx Jorg!)
    // see http://openenergymonitor.org/emon/node/1051?page=3
    if ((rf12_control(0x0000) & RF_RSSI_BIT) == 0) { // make sure no-one else is Tx'ing
        rf12_control(RF_IDLE_MODE); // stop receiver
        rxstate = TXIDLE;
        return 1;
    }
    return 0;
}

/// @details
/// Switch to transmission mode and send a packet.
///
/// Notes
/// -----
///
/// The rf12_sendStart() function may only be called in specific situations:
///
/// * right after rf12_canSend() returns true - used to send requests out
///
/// Because transmissions may only be started when there is no other reception
/// or transmission taking place.
///
/// Call with 2 arguments, i.e. "rf12_sendStart(data, length)" 
/// followed by a call to rf12_sendWait().
/// @param ptr Pointer to the data to send as packet.
/// @param len Number of data bytes to send. Must be in the range 0 .. 65.
void rf12_sendStart (const void* ptr, uint8_t len) {
    rf12_len = len;
    memcpy((void*) rf12_data, ptr, len);
    rxstate = TXPRE2; // avoid too much sync data
    rf12_xfer(RF_XMITTER_ON); // bytes will be fed via interrupts
}

/// @details
/// Wait until transmission is possible, then start it as soon as possible.
/// @note This uses a (brief) busy loop and will discard any incoming packets.
/// @param hdr The header contains information about the destination of the
///            packet to send, and flags such as whether this should be
///            acknowledged - or if it actually is an acknowledgement.
/// @param ptr Pointer to the data to send as packet.
/// @param len Number of data bytes to send. Must be in the range 0 .. 65.
void rf12_sendNow (const void* ptr, uint8_t len) {
  while (!rf12_canSend());
  rf12_sendStart(ptr, len);
}
  
/// @details
/// Wait for completion of the preceding rf12_sendStart() call, using the
/// specified low-power mode.
/// @note rf12_sendWait() should only be called right after rf12_sendStart().
/// @param mode Power-down mode during wait: 0 = NORMAL, 1 = IDLE, 2 = STANDBY,
///             3 = PWR_DOWN. Values 2 and 3 can cause the millisecond time to 
///             lose a few interrupts. Value 3 can only be used if the ATmega 
///             fuses have been set for fast startup, i.e. 258 CK - the default
///             Arduino fuse settings are not suitable for full power down.
void rf12_sendWait (uint8_t mode) {
    // wait for packet to actually finish sending
    // go into low power mode, as interrupts are going to come in very soon
    while (rxstate != TXDONE)
        if (mode) {
            // power down mode is only possible if the fuses are set to start
            // up in 258 clock cycles, i.e. approx 4 us - else must use standby!
            // modes 2 and higher may lose a few clock timer ticks
            set_sleep_mode(mode == 3 ? SLEEP_MODE_PWR_DOWN :
#ifdef SLEEP_MODE_STANDBY
                           mode == 2 ? SLEEP_MODE_STANDBY :
#endif
                                       SLEEP_MODE_IDLE);
            sleep_mode();
        }
}

/// @details
/// Call this once with the node ID (0-31), frequency band (0-3), and
/// optional group (0-255 for RFM12B, only 212 allowed for RFM12).
/// @param id The ID of this wireless node. ID's should be unique within the
///           netGroup in which this node is operating. The ID range is 0 to 31,
///           but only 1..30 are available for normal use. You can pass a single
///           capital letter as node ID, with 'A' .. 'Z' corresponding to the
///           node ID's 1..26, but this convention is now discouraged. ID 0 is
///           reserved for OOK use, node ID 31 is special because it will pick
///           up packets for any node (in the same netGroup).
/// @param band This determines in which frequency range the wireless module
///             will operate. The following pre-defined constants are available:
///             RF12_433MHZ, RF12_868MHZ, RF12_915MHZ. You should use the one
///             matching the module you have.
/// @param g Net groups are used to separate nodes: only nodes in the same net
///          group can communicate with each other. Valid values are 1 to 212. 
///          This parameter is optional, it defaults to 212 (0xD4) when omitted.
///          This is the only allowed value for RFM12 modules, only RFM12B
///          modules support other group values.
/// @returns the nodeId, to be compatible with rf12_config().
///
/// Programming Tips
/// ----------------
/// Note that rf12_initialize() does not use the EEprom netId and netGroup
/// settings, nor does it change the EEPROM settings. To use the netId and
/// netGroup settings saved in EEPROM use rf12_config() instead of
/// rf12_initialize. The choice whether to use rf12_initialize() or
/// rf12_config() at the top of every sketch is one of personal preference.
/// To set EEPROM settings for use with rf12_config() use the RF12demo sketch.
uint8_t rf12_initialize (uint8_t id, uint8_t band, uint8_t g) {
    nodeid = id;
    group = g;
    
    rf12_spiInit();

    rf12_xfer(0x0000); // intitial SPI transfer added to avoid power-up problem

    rf12_xfer(RF_SLEEP_MODE); // DC (disable clk pin), enable lbd
    
    // wait until RFM12B is out of power-up reset, this takes several *seconds*
    rf12_xfer(RF_TXREG_WRITE); // in case we're still in OOK mode
    while (digitalRead(RFM_IRQ) == 0)
        rf12_xfer(0x0000);
        
    rf12_xfer(0x80C7 | (band << 4)); // EL (ena TX), EF (ena RX FIFO), 12.0pF 
    rf12_xfer(0xA640); // 868MHz 
    rf12_xfer(0xC606); // approx 49.2 Kbps, i.e. 10000/29/(1+6) Kbps
    rf12_xfer(0x94A2); // VDI,FAST,134kHz,0dBm,-91dBm 
    rf12_xfer(0xC2AC); // AL,!ml,DIG,DQD4 
    if (group != 0) {
        rf12_xfer(0xCA83); // FIFO8,2-SYNC,!ff,DR 
        rf12_xfer(0xCE00 | group); // SYNC=2DXX； 
    } else {
        rf12_xfer(0xCA8B); // FIFO8,1-SYNC,!ff,DR 
        rf12_xfer(0xCE2D); // SYNC=2D； 
    }
    rf12_xfer(0xC483); // @PWR,NO RSTRIC,!st,!fi,OE,EN 
    rf12_xfer(0x9850); // !mp,90kHz,MAX OUT 
    rf12_xfer(0xCC77); // OB1，OB0, LPX,！ddy，DDIT，BW0 
    rf12_xfer(0xE000); // NOT USE 
    rf12_xfer(0xC800); // NOT USE 
    rf12_xfer(0xC049); // 1.66MHz,3.1V 

    rxstate = TXIDLE;
#if PINCHG_IRQ
    #if RFM_IRQ < 8
        if ((nodeid & NODE_ID) != 0) {
            bitClear(DDRD, RFM_IRQ);      // input
            bitSet(PORTD, RFM_IRQ);       // pull-up
            bitSet(PCMSK2, RFM_IRQ);      // pin-change
            bitSet(PCICR, PCIE2);         // enable
        } else
            bitClear(PCMSK2, RFM_IRQ);
    #elif RFM_IRQ < 14
        if ((nodeid & NODE_ID) != 0) {
            bitClear(DDRB, RFM_IRQ - 8);  // input
            bitSet(PORTB, RFM_IRQ - 8);   // pull-up
            bitSet(PCMSK0, RFM_IRQ - 8);  // pin-change
            bitSet(PCICR, PCIE0);         // enable
        } else
            bitClear(PCMSK0, RFM_IRQ - 8);
    #else
        if ((nodeid & NODE_ID) != 0) {
            bitClear(DDRC, RFM_IRQ - 14); // input
            bitSet(PORTC, RFM_IRQ - 14);  // pull-up
            bitSet(PCMSK1, RFM_IRQ - 14); // pin-change
            bitSet(PCICR, PCIE1);         // enable
        } else
            bitClear(PCMSK1, RFM_IRQ - 14);
    #endif
#else
    if ((nodeid & NODE_ID) != 0)
        attachInterrupt(0, rf12_interrupt, LOW);
    else
        detachInterrupt(0);
#endif
    
    return nodeid;
}

/// @details
/// This can be used to send out slow bit-by-bit On Off Keying signals to other
/// devices such as remotely controlled power switches operating in the 433,
/// 868, or 915 MHz bands.
///
/// To use this, you need to first call rf12initialize() with a zero node ID
/// and the proper frequency band. Then call rf12onOff() in the exact timing
/// you need for sending out the signal. Once done, either call rf12onOff(0) to
/// turn the transmitter off, or reinitialize the wireless module completely
/// with a call to rf12initialize().
/// @param value Turn the transmitter on (if true) or off (if false).
/// @note The timing of this function is relatively coarse, because SPI
/// transfers are used to enable / disable the transmitter. This will add some
/// jitter to the signal, probably in the order of 10 µsec.
void rf12_onOff (uint8_t value) {
    rf12_xfer(value ? RF_XMITTER_ON : RF_IDLE_MODE);
}

/// @details
/// This calls rf12_initialize() with settings obtained from EEPROM address
/// 0x20 .. 0x3F. These settings can be filled in by the RF12demo sketch in the
/// RFM12B library. If the checksum included in those bytes is not valid, 
/// rf12_initialize() will not be called.
///
/// As side effect, rf12_config() also writes the current configuration to the
/// serial port, ending with a newline.
/// @returns the node ID obtained from EEPROM, or 0 if there was none.
uint8_t rf12_config (uint8_t show) {
    uint16_t crc = ~0;
    for (uint8_t i = 0; i < RF12_EEPROM_SIZE; ++i)
        crc = _crc16_update(crc, eeprom_read_byte(RF12_EEPROM_ADDR + i));
    if (crc != 0)
        return 0;
        
    uint8_t nodeId = 0, group = 0;
    for (uint8_t i = 0; i < RF12_EEPROM_SIZE - 2; ++i) {
        uint8_t b = eeprom_read_byte(RF12_EEPROM_ADDR + i);
        if (i == 0)
            nodeId = b;
        else if (i == 1)
            group = b;
        else if (b == 0)
            break;
        else if (show)
            Serial.print((char) b);
    }
    if (show)
        Serial.println();
    
    rf12_initialize(nodeId, nodeId >> 6, group);
    return nodeId & 0x1F; //RF12_HDR_MASK;
}

/// @details
/// This function can put the radio module to sleep and wake it up again.
/// In sleep mode, the radio will draw only one or two microamps of current.
///
/// This function can also be used as low-power watchdog, by putting the radio
/// to sleep and having it raise an interrupt between about 30 milliseconds
/// and 4 seconds later.
/// @param n If RF12SLEEP (0), put the radio to sleep - no scheduled wakeup. 
///          If RF12WAKEUP (-1), wake the radio up so that the next call to 
///          rf12_recvDone() can restore normal reception. If value is in the
///          range 1 .. 127, then the radio will go to sleep and generate an 
///          interrupt approximately 32*value miliiseconds later.
/// @todo Figure out how to get the "watchdog" mode working reliably.
void rf12_sleep (char n) {
    if (n < 0)
        rf12_control(RF_IDLE_MODE);
    else {
        rf12_control(RF_WAKEUP_TIMER | 0x0500 | n);
        rf12_control(RF_SLEEP_MODE);
        if (n > 0)
            rf12_control(RF_WAKEUP_MODE);
    }
    rxstate = TXIDLE;
}

/// @details
/// This checks the status of the RF12 low-battery detector. It wil be 1 when
/// the supply voltage drops below 3.1V, and 0 otherwise. This can be used to
/// detect an impending power failure, but there are no guarantees that the
/// power still remaining will be sufficient to send or receive further packets.
char rf12_lowbat () {
    return (rf12_control(0x0000) & RF_LBD_BIT) != 0;
}