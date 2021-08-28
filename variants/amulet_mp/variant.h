 /*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
  Copyright (c) 2016 Sandeep Mistry All right reserved.
  Copyright (c) 2018, Adafruit Industries (adafruit.com)

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_AMULET_MP_
#define _VARIANT_AMULET_MP_

 /** Master clock frequency */
#define VARIANT_MCK       (64000000ul)

// #define USE_LFXO      // Board uses 32khz crystal for LF
#define USE_LFRC    // Board uses RC for LF
#define USE_DCDC

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// Number of pins defined in PinDescription array
#define PINS_COUNT           (32u)
#define NUM_DIGITAL_PINS     (32u)
#define NUM_ANALOG_INPUTS    (8u)
#define NUM_ANALOG_OUTPUTS   (0u)

#define PIN_RGB_LEDS         (26u)
#define RGB_LED_COUNT        (8)

// Pin used to enable voltage rail for RGB LEDs
#define PIN_RGB_LED_PWR       (5u)
#define RGB_LED_PWR_ON        (0)      // State to enable RGB led voltage rail

// LEDs
#define PIN_LED1             (8)

#define LED_BUILTIN          PIN_LED1

#define LED_BLUE             PIN_LED1

// Don't use this.  Only here to keep compat with some test sketches
#define LED_RED              LED_BLUE

#define LED_STATE_ON         0         // State when LED is litted

/*
 * Buttons
 */

#define PIN_DFU             (27u)
#define PIN_RESET           (20u)

/*
 * Analog pins
 */
#define PIN_A0               (2)
#define PIN_A1               (3)
#define PIN_A2               (4)
#define PIN_A3               (5)
#define PIN_A4               (28)
#define PIN_A5               (29)
#define PIN_A6               (30)
#define PIN_A7               (31)

static const uint8_t A0  = PIN_A0 ;
static const uint8_t A1  = PIN_A1 ;
static const uint8_t A2  = PIN_A2 ;
static const uint8_t A3  = PIN_A3 ;
static const uint8_t A4  = PIN_A4 ;
static const uint8_t A5  = PIN_A5 ;
static const uint8_t A6  = PIN_A6 ;
static const uint8_t A7  = PIN_A7 ;
#define ADC_RESOLUTION    14

// Other pins
#define PIN_AREF           (24)
#define PIN_VBAT           PIN_A7
#define PIN_NFC1           (9)
#define PIN_NFC2           (10)

static const uint8_t AREF = PIN_AREF;

/*
 * Serial interfaces
 */
// Arduino Header D0, D1
#define PIN_SERIAL_RX       (18)
#define PIN_SERIAL_TX       (19)

// Connected to Jlink CDC
// #define PIN_SERIAL2_RX      (8)
// #define PIN_SERIAL2_TX      (6)
/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO         (14)
#define PIN_SPI_MOSI         (13)
#define PIN_SPI_SCK          (12)

static const uint8_t SS   = 18 ;
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (25u)
#define PIN_WIRE_SCL         (26u)

#define FASTLED_NRF52_SUPPRESS_UNTESTED_BOARD_WARNING
#define ARDUINO_GENERIC 

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif
