/*
* This file is part of ddprint - a direct drive 3D printer firmware.
* 
* Copyright 2020 erwin.rieger@ibrieger.de
* 
* ddprint is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* ddprint is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with ddprint.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <Arduino.h>
#include <avr/wdt.h>

#include "pin_states.h"
#include "ddcommands.h"

#include "SdCard/SdSpiCard.h"
#include "massstoragebase.h"

//
// ATMega 2560
//

#define SERIAL_TX_DR_EMPTY() ( (UCSR0A) & (1 << UDRE0) )
#define SERIAL_TX_COMPLETE() ( true )
#define SERIAL_TX_DR_PUTC(c) ( UDR0 = c )

constexpr uint32_t halBaudrate(const uint32_t br) { return ((F_CPU+8*br) / (16*br)) - 1; }

#define HAL_SET_INPUT_PU(pin) pinMode(pin, INPUT_PULLUP)
#define HAL_SET_INPUT_ANALOG(pin) /* */

#define HAL_READ(pin) digitalRead(pin)
#define HAL_READ_ANALOG(pin) analogRead(pin)

#define HAL_SET_OUTPUT(pin)  pinMode(pin, OUTPUT)

// #define SET_OUTPUT_PWM(pin, activeLow)  SET_OUTPUT(pin)

#define HAL_WRITE(pin, v)  digitalWrite(pin, v)
// #define PWM_WRITE(p, v) analogWrite(p, v)

#define HAL_READ_ANALOG(pin) analogRead(pin)

#define CLI()   cli()
#define SEI()   sei()

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~(_BV(bit)))
#endif

#define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
#define CRITICAL_SECTION_END    SREG = _sreg;

#define WDT_ENABLE() wdt_enable(WDTO_4S) /* Timeout 4 seconds */
#define WDT_RESET() wdt_reset()

#define HAL_SYSTEM_RESET() systemHardReset();

#define TIMER_INIT() timerInit()

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  TIMSK1 |= (1<<OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() TIMSK1 &= ~(1<<OCIE1A)

#define ENABLE_STEPPER1_DRIVER_INTERRUPT()  TIMSK1 |= (1<<OCIE1B)
#define DISABLE_STEPPER1_DRIVER_INTERRUPT() TIMSK1 &= ~(1<<OCIE1B)
#define STEPPER1_DRIVER_INTERRUPT_ENABLED() (TIMSK1 & (1<<OCIE1B))

#define HAL_SET_STEPPER_TIMER(timerval) { OCR1A = timerval; }
#define HAL_SET_HOMING_TIMER(timerval) { OCR1A = OCR1B = timerval; }

#define HAL_SPI_INIT() spiInit()

#define HAL_IRQ_INIT() /* */

#define HAL_FS_SPI_SETTINGS SPISettings spiSettingsFS(1000000, MSBFIRST, SPI_MODE1)

#define HAL_FILSENSOR_BEGINTRANS() dDPrintSpi.beginTransaction(spiSettingsFS)

#define HAL_FILSENSOR_READ() dDPrintSpi.transfer(0)

////////////////////////////////////////////////////////////////////////////////////////////////////

void HAL_SETUP_TEMP_ADC();

////////////////////////////////////////////////////////////////////////////////////////////////////

#define PORTADDR(port) ((uint16_t) & (port))

////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Pins
//
template <const uint8_t PIN>
struct AvrPin {

    static volatile uint8_t * pinAddr;
    static volatile uint8_t * portAddr;
    static uint8_t  bitMask;
    static volatile uint8_t * ddrAddr;

    static uint8_t getPin() { return PIN; }

    // timer for pwm: volatile uint8_t *pwmAddr;
    static void setOutput() {
        uint8_t port = digitalPinToPort(PIN);
        pinAddr = portInputRegister(port);
        portAddr = portOutputRegister(port);
        ddrAddr = portModeRegister(port);
        bitMask = digitalPinToBitMask(PIN);

        CRITICAL_SECTION_START
        *ddrAddr = *ddrAddr | bitMask;
        CRITICAL_SECTION_END
    };
    static void setInput(bool pullup) {
        uint8_t port = digitalPinToPort(PIN);
        pinAddr = portInputRegister(port);
        portAddr = portOutputRegister(port);
        ddrAddr = portModeRegister(port);
        bitMask = digitalPinToBitMask(PIN);

        CRITICAL_SECTION_START
        *ddrAddr = *ddrAddr & ~bitMask;
        CRITICAL_SECTION_END
        if (pullup)
            outputHigh(); // enable pullup
        else
            outputLow(); // disable pullup
    };
    FWINLINE static void outputLow() {
        CRITICAL_SECTION_START
        *portAddr = *portAddr & ~bitMask;
        CRITICAL_SECTION_END
    };
    FWINLINE static void outputHigh() {
        CRITICAL_SECTION_START
        *portAddr = *portAddr | bitMask;
        CRITICAL_SECTION_END
    };
    FWINLINE static uint8_t read() {
        uint8_t res = *pinAddr & bitMask;
        return res;
    };
};

template <const uint8_t PIN>
struct DigitalOutput<PIN, ACTIVEHIGHPIN>: AvrPin<PIN> {
    static void initDeActive() { AvrPin<PIN>::setOutput(); deActivate(); }
    FWINLINE static void activate() { AvrPin<PIN>::outputHigh(); };
    FWINLINE static void deActivate() { AvrPin<PIN>::outputLow(); };
    static void saveState() { AvrPin<PIN>::setInput(false); }
};

template <const uint8_t PIN>
struct DigitalOutput<PIN, ACTIVELOWPIN>: AvrPin<PIN>  {
    static void initDeActive() { AvrPin<PIN>::setOutput(); deActivate(); }
    FWINLINE static void activate() { AvrPin<PIN>::outputLow(); };
    FWINLINE static void deActivate() { AvrPin<PIN>::outputHigh(); };
};

////////////////////////////////////////////////////////////////////////////////////////////////////

//
// Macro to declare static storage for AvrPin template members.
//
// #define DigitalOutPin(nr, active) { DigitalOutput< nr, active >; uint8_t DigitalOutput< nr, active > :: bitMask = 0; }
#define DefineIOPinMembers(nr) template<> uint8_t AvrPin< nr > :: bitMask = 0; \
                               template<> volatile uint8_t * AvrPin< nr > :: pinAddr = NULL; \
                               template<> volatile uint8_t * AvrPin< nr > :: portAddr = NULL; \
                               template<> volatile uint8_t * AvrPin< nr > :: ddrAddr = NULL;

////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Optimized output pins for steppers on AVR, assumes we are called from
// ISR context (or when no stepper is running), so no critical section stuff.
// Assumes port-addr is ddr-addr + 1
//
template <const uint16_t PORTADDR, const uint8_t PIN>
struct FastAvrPin {

    static void setOutput() {
        *(volatile uint8_t*)(PORTADDR-1) |= _BV(PIN);
    };
    FWINLINE static void outputLow() {
        *(volatile uint8_t*)PORTADDR &= ~_BV(PIN);
    };
    FWINLINE static void outputHigh() {
        *(volatile uint8_t*)PORTADDR |= _BV(PIN);
    };
};

template <const uint16_t PORTADDR, const uint8_t PIN, typename ACTIVEHIGH>
struct FastDigitalOutput { };

template <const uint16_t PORTADDR, const uint8_t PIN>
struct FastDigitalOutput<PORTADDR, PIN, ACTIVEHIGHPIN>: FastAvrPin<PORTADDR, PIN> {
    static void initDeActive() { FastAvrPin<PORTADDR, PIN>::setOutput(); deActivate(); }
    FWINLINE static void activate() { FastAvrPin<PORTADDR, PIN>::outputHigh(); };
    FWINLINE static void deActivate() { FastAvrPin<PORTADDR, PIN>::outputLow(); };
};

template <const uint16_t PORTADDR, const uint8_t PIN>
struct FastDigitalOutput<PORTADDR, PIN, ACTIVELOWPIN>: FastAvrPin<PORTADDR, PIN> {
    static void initDeActive() { FastAvrPin<PORTADDR, PIN>::setOutput(); deActivate(); }
    FWINLINE static void activate() { FastAvrPin<PORTADDR, PIN>::outputLow(); };
    FWINLINE static void deActivate() { FastAvrPin<PORTADDR, PIN>::outputHigh(); };
};

////////////////////////////////////////////////////////////////////////////////////////////////////

// note: pullup für avr durch high-write auf input: WRITE(X_STOP_PIN,HIGH); 
template <uint8_t PIN, typename ACTIVEHIGH>
struct DigitalInput { };

template <uint8_t PIN>
struct DigitalInput<PIN, ACTIVELOWPIN>: AvrPin<PIN> {
    static void init() { AvrPin<PIN>::setInput(true); }
    FWINLINE static bool active() { return AvrPin<PIN>::read() == LOW; }
    FWINLINE static bool deActive() { return ! active(); }
};

////////////////////////////////////////////////////////////////////////////////////////////////////

template <uint8_t PIN>
struct PWMOutput<PIN, ACTIVEHIGHPIN>: AvrPin<PIN> {
    static void init() { AvrPin<PIN>::setOutput(); }
    FWINLINE static void write(uint8_t v) { analogWrite(PIN, v); }
    static void saveState() { AvrPin<PIN>::setInput(false); }
};

////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Emergency hard reset/reboot
//
void systemHardReset();

////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Initialize the used timers
//
void timerInit();

////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Initialize the spi bus
//
void spiInit();

////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Swap device/Mass storage
//

enum WriteBlockState { WBWait1, WBWait2 };

//
// Mass storage interface, sdcard connected to SPI in this case.
//
class MassStorage: public SdSpiCard, public MassStorageBase {

public:

    bool swapInit();

protected:

    // Number of bytes to shift the block address for non-sdhc cards
    uint8_t blockShift;
    uint8_t readRetry;

    SdSpiAltDriver m_spi;

    bool writeBlock(uint32_t writeBlockNumber, uint8_t *src) {

        static WriteBlockState wbstate = WBWait1;

        switch (wbstate) {

            case WBWait1:

                // Wait while card is busy, no timeout check!
                if (isBusy())  // isBusy() is doing spiStart()/spiStop()
                    return true;

                if (cardCommand(CMD24, writeBlockNumber << blockShift)) { // cardCommand() is doing spiStart()

                    killMessage(RespSDWriteError, SD_CARD_ERROR_CMD24, errorData());
                    // notreached
                }

                if (!writeData(DATA_START_BLOCK, src)) { // writeData() does not spiStart() but spiStop() in case of error

                    killMessage(RespSDWriteError, errorCode(), errorData());
                    // notreached
                }

                spiStop();
                wbstate = WBWait2;
                return true; // continue sub thread

            default: // WBWait2

                // Wait for flash programming to complete, no timeout check!
                if (isBusy()) // isBusy() is doing spiStart()/spiStop()
                    return true;

                if (cardCommand(CMD13, 0) || m_spi.receive()) { // cardCommand() is doing spiStart()

                    killMessage(RespSDWriteError, SD_CARD_ERROR_CMD13, errorData());
                    // notreached
                }

                spiStop();
                wbstate = WBWait1;
                return false; // stop sub thread
        }
    }

    //
    // Wrapper around lowlevel read
    // Returns 0 if read is done and ok
    // Returns <0 if read is done and error (return value is error code)
    // Returns 1 if we want to be called again to continue work
    //
    int readBlockWrapper(uint32_t readBlockNumber, uint8_t *dst) {

        if (! SdSpiCard::readBlock(readBlockNumber, dst)) {

            // Return Sd2Card error code and SPI status byte from sd card.
            // In case of SD_CARD_ERROR_CMD17 this is the return status of
            // CMD17 in Sd2Card::cardCommand().

            if (readRetry++ < 5) {
                // Log error and retry
                // txBuffer.sendSimpleResponse(RespUnsolicitedMsg, RespSDReadError, errorCode(), errorData());
                return 1; // continue
            }

            // return -errorcode
            killMessage(RespSDReadError, errorCode(), errorData());
            // notreached
        }

        readRetry = 0;
        return 0; // done
    }
};

////////////////////////////////////////////////////////////////////////////////////////////////////



