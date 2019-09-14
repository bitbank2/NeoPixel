//#include <TinyWireM.h>
//#include <USI_TWI_Master.h>

#include <avr/pgmspace.h>
#include <avr/io.h>
//
// Larry's NeoPixel (WS2812B) library
// Copyright (c) 2018 BitBank Software, Inc.
// Project started 3/7/2018
// Written by Larry Bank (bitbank@pobox.com)
//
// NeoPixels (aka WS2812B) are small, independent RGB LED modules containing an integrated controller.
// The controller can produce any 24-bit color by modulating the brightness of the red/green/blue LEDs with PWM.
// The controller uses a 1-wire communication protocol that allows them to be strung together in almost any length.
// The data is sent as 24 bits in GRB order (MSB first). The first module in the chain captures the first 24-bits
// then forwards the successive bits to the next in line. The modules latch/display their data when there is a pause
// in the data stream. This means that for a string of N modules, Nx24 bits must be transmitted every time any of
// the LEDs are to change their color.
//
// Why re-invent the wheel? The challenge is to run a long string of LEDs on microcontrollers with limited
// resources (like the ATTiny AVRs). This code allows you to create long runs of color patterns without needing
// RAM to back each pixel. It accomplishes this by dynamically generating the colors based on a run-length encoding
// scheme. On ATTiny controllers, the CPU is not fast enough to handle dynamicly calculated color gradients, so the
// color delta values have to be explicitly defined in the data stream, but the effect is still good.
//
// The run-length encoded byte stream defines a mode+length followed by the palette color
//  +-+-+-+-+-+-+-+-+
//  |M|M|L|L|L|L|L|L|   MM = 2 bits for mode, LLLLLL = 6 bits for length (1-64) followed by optional palette values
//  +-+-+-+-+-+-+-+-+
// There are 4 "modes" in the RLE data stream:
// MODE_PULSE - defines a run of pixels which pulsate between 2 colors
// MODE_SIMPLE - defines a run of a single palette color, 1 byte palette entry follows
// MODE_GRADIENT - defines a run of a gradient color, 1 byte palette entry, followed by 3 bytes for the RGB deltas
// MODE_END - must be the last byte of the RLE data. Causes the code to start from the beginning
//
#define MODE_MASK 0xc0
#define LENGTH_MASK 0x3f
// A gap of N pixels which slowly pulse between two values
#define MODE_PULSE 0
// A run of N pixels of the same color 
#define MODE_SIMPLE 0x40
// A run of N pixels of a color gradient (start + end color)
#define MODE_GRADIENT 0x80
// The end of a string definition
#define MODE_END 0xc0
#define YELLOW_BUTTON 3
#define ORANGE_BUTTON 2
//#define YELLOW_BUTTON 8
//#define ORANGE_BUTTON 9
//
// Structure for holding the current state of the string of LEDs
//
typedef struct _tagLEDSTRING
{
int iLen; // number of LEDs in the string
byte lastOP; // last operation
byte *pRLEStart; // pointer to start of run-length encoded data for the string
byte *pRLE; // pointer to current spot in the RLE data
byte iCount; // remaining count in the current operation
byte iDir; // direction for pulse (0 = positive, 1 = negative)
unsigned char r, g, b; // current color
signed char dr, dg, db; // color deltas
unsigned char sr, sg, sb; // starting color values (for pulse)
unsigned char er, eg, eb; // ending color values (for pulse)
} LEDSTRING;

#if !defined (__AVR_ATtiny85__)
#include <SPI.h>
#else
#define LED_PORT PORTB // Port of the pin the pixels are connected to
#define LED_DDR DDRB // Port of the pin the pixels are connected to
#define LED_BIT 0 // Bit of the pin the pixels are connected to
#define LONG_DELAY 10
#define SHORT_DELAY 3
#endif

//
// RGB triplets which define the palette
// Up to 256 entries (768 bytes)
// must be stored in flash (PROGMEM)
//
const byte pPalette[] PROGMEM = {0,0,0, 0x1,0,0, 0,0x1,0, 0,0,0x1, 0x1,0x1,0, 0x1,0,0x1, 0,0x1,0x1, 0x1,0x1,0x1,
              0,0,0, 0x8,0,0, 0,0x8,0, 0,0,0x8, 0x8,0x8,0, 0x8,0,0x8, 0,0x8,0x8, 0x8,0x8,0x8,
              0,0,0, 0x10,0,0, 0,0x10,0, 0,0,0x10, 0x10,0x10,0, 0x10,0,0x10, 0,0x10,0x10, 0x10,0x10,0x10};


#ifndef TEST_SIMPLE
//const byte pDemo[] PROGMEM = {
//  MODE_GRADIENT | 31, 0x00, 0x1, 0, 0, MODE_GRADIENT | 31, 0x01, 0xff, 0, 0, MODE_END};
// A demo string to show all the colors of the palette
//const byte pDemo[] PROGMEM = {
//              MODE_GRADIENT | 0x3, 0x01, 0, 0x4, 0, MODE_GRADIENT | 0x3, 0x01, 0xfc, 0x4,0,
//              MODE_GRADIENT | 0x3, 0x01, 0, 0xfc, 0x4, MODE_GRADIENT | 0x3, 0x03, 0x4, 0x4, 0xfc,
//              MODE_GRADIENT | 0x3, 0x04, 0, 0xfc, 0x4, MODE_GRADIENT | 0x3, 0x05, 0xfc, 0x4, 0,
//              MODE_GRADIENT | 0x3, 0x06, 0x4,0,0, MODE_GRADIENT | 0x3, 0x07, 0xfc,0xfc,0xfc,
//              MODE_END};
const byte pPulse1[] PROGMEM = {
              MODE_PULSE | 0x3b, 0x0,0,0x10, 0x1,0,0xff, 0x10,0,0, // start, delta, end color
              MODE_END};              
const byte pPulse2[] PROGMEM = {
              MODE_PULSE | 0x3b, 0x0,0,0x40, 0x1,0,0xff, 0x40,0,0, // start, delta, end color
              MODE_END};              
const byte pPulse3[] PROGMEM = {
              MODE_PULSE | 0x3b, 0x0,0,0x10, 0,0x1,0xff, 0,0x10,0, // start, delta, end color
              MODE_END};              
const byte pPulse4[] PROGMEM = {
              MODE_PULSE | 0x3b, 0x0,0,0x40, 0,0x1,0xff, 0,0x40,0, // start, delta, end color
              MODE_END};              
const byte pPulse5[] PROGMEM = {
              MODE_PULSE | 0x3b, 0x0,0x10,0, 0x1,0xff,0, 0x10,0,0, // start, delta, end color
              MODE_END};              
const byte pPulse6[] PROGMEM = {
              MODE_PULSE | 0x3b, 0x0,0x40,0, 0x1,0xff,0, 0x40,0,0, // start, delta, end color
              MODE_END};              
const byte pPulse7[] PROGMEM = {
              MODE_PULSE | 0x3b, 0x0,0x10,0x10, 0x1,0xff,0xff, 0x10,0,0, // start, delta, end color
              MODE_END};              
const byte pPulse8[] PROGMEM = {
              MODE_PULSE | 0x3b, 0x0,0x40,0x40, 0x1,0xff,0xff, 0x40,0,0, // start, delta, end color
              MODE_END};              
const byte pPulse9[] PROGMEM = {
              MODE_PULSE | 0x3b, 0x0,0x10,0x40, 0x1,0xff,0, 0x10,0,0x10, // start, delta, end color
              MODE_END};              
const byte pPulse10[] PROGMEM = {
              MODE_PULSE | 0x3b, 0x0,0x40,0x40, 0x1,0xff,0, 0x40,0,0x40, // start, delta, end color
              MODE_END};              
const byte pPulse11[] PROGMEM = {
              MODE_PULSE | 0x3b, 0x10,0,0x10, 0xff,0x1,0xff, 0,0x10,0, // start, delta, end color
              MODE_END};              
const byte pPulse12[] PROGMEM = {
              MODE_PULSE | 0x3b, 0x40,0,0x40, 0xff,0x1,0xff, 0,0x40,0, // start, delta, end color
              MODE_END};              
const byte pPulse13[] PROGMEM = {
              MODE_PULSE | 0x3b, 0x10,0x10,0x10, 0xff,0xff,0xff, 0,0,0, // start, delta, end color
              MODE_END};              
const byte pPulse14[] PROGMEM = {
              MODE_PULSE | 0x3b, 0x40,0x40,0x40, 0xff,0xff,0xff, 0,0,0, // start, delta, end color
              MODE_END};              
const byte pCircus1[] PROGMEM = {
              MODE_SIMPLE | 0x2, 0x0, MODE_SIMPLE | 0x2, 0x1, MODE_SIMPLE | 0x2, 0x2, MODE_SIMPLE | 0x2, 0x3,
              MODE_SIMPLE | 0x2, 0x4, MODE_SIMPLE | 0x2, 0x5, MODE_SIMPLE | 0x2, 0x6, MODE_SIMPLE | 0x2, 0x7,
              MODE_END};
const byte pCircus2[] PROGMEM = {
              MODE_SIMPLE | 0x2, 0x8, MODE_SIMPLE | 0x2, 0x9, MODE_SIMPLE | 0x2, 0xa, MODE_SIMPLE | 0x2, 0xb,
              MODE_SIMPLE | 0x2, 0xc, MODE_SIMPLE | 0x2, 0xd, MODE_SIMPLE | 0x2, 0xe, MODE_SIMPLE | 0x2, 0xf,
              MODE_END};
const byte pCircus3[] PROGMEM = {
              MODE_SIMPLE | 0x2, 0x10, MODE_SIMPLE | 0x2, 0x11, MODE_SIMPLE | 0x2, 0x12, MODE_SIMPLE | 0x2, 0x13,
              MODE_SIMPLE | 0x2, 0x14, MODE_SIMPLE | 0x2, 0x15, MODE_SIMPLE | 0x2, 0x16, MODE_SIMPLE | 0x2, 0x17,
              MODE_END};
const byte pSimple1[] PROGMEM = {
              MODE_SIMPLE | 0x3b, 1, MODE_END};
const byte pSimple2[] PROGMEM = {
              MODE_SIMPLE | 0x3b, 9, MODE_END};
const byte pSimple3[] PROGMEM = {
              MODE_SIMPLE | 0x3b, 2, MODE_END};
const byte pSimple4[] PROGMEM = {
              MODE_SIMPLE | 0x3b, 10, MODE_END};
const byte pSimple5[] PROGMEM = {
              MODE_SIMPLE | 0x3b, 3, MODE_END};
const byte pSimple6[] PROGMEM = {
              MODE_SIMPLE | 0x3b, 11, MODE_END};
const byte pSimple7[] PROGMEM = {
              MODE_SIMPLE | 0x3b, 4, MODE_END};
const byte pSimple8[] PROGMEM = {
              MODE_SIMPLE | 0x3b, 12, MODE_END};
const byte pSimple9[] PROGMEM = {
              MODE_SIMPLE | 0x3b, 5, MODE_END};
const byte pSimple10[] PROGMEM = {
              MODE_SIMPLE | 0x3b, 13, MODE_END};
const byte pSimple11[] PROGMEM = {
              MODE_SIMPLE | 0x3b, 6, MODE_END};
const byte pSimple12[] PROGMEM = {
              MODE_SIMPLE | 0x3b, 14, MODE_END};
const byte pSimple13[] PROGMEM = {
              MODE_SIMPLE | 0x3b, 7, MODE_END};
const byte pSimple14[] PROGMEM = {
              MODE_SIMPLE | 0x3b, 15, MODE_END};
const byte pDemo2[] PROGMEM = {
              MODE_GRADIENT | 0x3b, 0x00, 0x4, 0, 0, MODE_GRADIENT | 0x3b, 0x00, 0x4, 0x0,0,
              MODE_GRADIENT | 0x3b, 0x02, 0, 0xfc, 0x4, MODE_GRADIENT | 0x3b, 0x03, 0x4, 0x4, 0xfc,
              MODE_GRADIENT | 0x3b, 0x04, 0, 0xfc, 0x4, MODE_GRADIENT | 0x3b, 0x05, 0xfc, 0x4, 0,
              MODE_GRADIENT | 0x3b, 0x06, 0x4,0,0, MODE_GRADIENT | 0x3b, 0x07, 0xfc,0xfc,0xfc,
              MODE_END};
#else
const byte pDemo[] PROGMEM = {
    MODE_SIMPLE | 0x7, 0x00, MODE_SIMPLE | 0x0, 0x01, MODE_SIMPLE | 0x7, 0x00, MODE_SIMPLE | 0x0, 0x02,
    MODE_SIMPLE | 0x7, 0x00, MODE_SIMPLE | 0x0, 0x03, MODE_SIMPLE | 0x7, 0x00, MODE_SIMPLE | 0x0, 0x04,
    MODE_SIMPLE | 0x7, 0x00, MODE_SIMPLE | 0x0, 0x05, MODE_SIMPLE | 0x7, 0x00, MODE_SIMPLE | 0x0, 0x06,
    MODE_SIMPLE | 0x7, 0x00, MODE_SIMPLE | 0x0, 0x07,
//              MODE_SIMPLE | 0x2, 0x00, MODE_SIMPLE | 0x2, 0x01,
//              MODE_SIMPLE | 0x2, 0x02, MODE_SIMPLE | 0x2, 0x03,
//              MODE_SIMPLE | 0x2, 0x04, MODE_SIMPLE | 0x2, 0x05,
//              MODE_SIMPLE | 0x2, 0x06, MODE_SIMPLE | 0x2, 0x07,
              MODE_END};
#endif

static LEDSTRING mystring;

#if defined (__AVR_ATtiny85__)
//
// This is the "bit-banged" version to transmit 8-bits of data to the WS2812B
// The timing was tested on a 16Mhz ATTiny85 (Digispark) and would need to be adjusted for other speeds
// It is unlikely that this will work properly when operating at less than 8Mhz
// 
void sendByte(byte b)
{
byte i;

 for(i=0; i<8; i++)
 {
   if (b & 0x80) // MSB to LSB
   { // 1-bit
      bitSet(LED_PORT , LED_BIT);
      __builtin_avr_delay_cycles(LONG_DELAY);
      bitClear(LED_PORT , LED_BIT);
      __builtin_avr_delay_cycles(SHORT_DELAY);
   } else
   { // 0-bit
      cli(); // We need to protect this bit from being made wider by an interrupt 
      bitSet(LED_PORT , LED_BIT);
      __builtin_avr_delay_cycles(SHORT_DELAY);
      bitClear(LED_PORT , LED_BIT);
      sei();
      __builtin_avr_delay_cycles(LONG_DELAY);
   }
   b <<= 1; // keep shifting over high bit
 } // for i
} /* sendByte() */
#else
// SPI version
void sendByte(byte b)
{
byte i;
  for (i=0; i<8; i++)
  {
    // shift out each bit (MSB first) and turn it into a SPI bit pattern
    if (b & 0x80) // a 1 bit
    {
      SPI.transfer(0xfc); // a 1
    }
    else
    {
      SPI.transfer(0xe0); // a 0
    }
    b <<= 1;
  }  
} /* sendByte() */
#endif // SPI version

//
// Write 3 bytes of data to the WS2812 (GRB order)
//
void LEDSendColor(byte r, byte g, byte b)
{
  sendByte(g);
  sendByte(r);
  sendByte(b);
} /* LEDSendColor() */
//
// Initialize the LEDSTRING structure with the RLE data and number of LEDs in the string
//
void LEDInit(byte *pRLE, LEDSTRING *pString, int iNumLEDs)
{
byte b, *s;
int i; //, d;

   memset(pString, 0, sizeof(LEDSTRING));
   s = pString->pRLEStart = pRLE;
   pString->iLen = iNumLEDs;
   b = pgm_read_byte(s++); // get first operation
   pString->iCount = (b & LENGTH_MASK); // count of this 'run'
   b &= MODE_MASK;
   pString->lastOP = b;
   switch(b)
   {
     case MODE_PULSE:
        pString->sr = pString->r = pgm_read_byte(s++); // starting values
        pString->sg = pString->g = pgm_read_byte(s++);
        pString->sb = pString->b = pgm_read_byte(s++);
        pString->dr = pgm_read_byte(s++); // get the deltas
        pString->dg = pgm_read_byte(s++);
        pString->db = pgm_read_byte(s++);
        pString->er = pgm_read_byte(s++); // ending values
        pString->eg = pgm_read_byte(s++);
        pString->eb = pgm_read_byte(s++);
        pString->iDir = 0; // start incrementing pulse operations
        break;
     case MODE_SIMPLE:
        i = (int)pgm_read_byte(s++); // get the single palette entry
        i *=3; // offset to this palette entry
        pString->r = pgm_read_byte(pPalette + i++);
        pString->g = pgm_read_byte(pPalette + i++);
        pString->b = pgm_read_byte(pPalette + i++);
        break;
     case MODE_GRADIENT:
        i = (int)pgm_read_byte(s++); // get the starting palette entry
        i *=3; // offset to this palette entry
        pString->r = pgm_read_byte(pPalette + i++);
        pString->g = pgm_read_byte(pPalette + i++);
        pString->b = pgm_read_byte(pPalette + i++);
        pString->dr = pgm_read_byte(s++); // get the deltas
        pString->dg = pgm_read_byte(s++);
        pString->db = pgm_read_byte(s++);
        break;
   }
   pString->pRLE = s; // keep current RLE pointer
} /* LEDInit() */

//
// Step for a pulsing effect
//
void LEDPulse(LEDSTRING *pString)
{
byte bChangeDir = 0;
int r, g, b;

// Put in integers to properly compare for over and underflow
  r = pString->r; g = pString->g; b = pString->b;
 
  if (pString->iDir == 0) // incrementing
  {
    r += pString->dr; // update color
    g += pString->dg;
    b += pString->db;
  }
  else // decrementing
  {
    r -= pString->dr; // update color
    g -= pString->dg;
    b -= pString->db;
  }
    if (pString->sr < pString->er && (r > pString->er || r < pString->sr))
       bChangeDir = 1;
    else if (pString->sr > pString->er && (r < pString->er || r > pString->sr))
       bChangeDir = 1;
    else if (pString->sg < pString->eg && (g > pString->eg || g < pString->sg))
       bChangeDir = 1;
    else if (pString->sg > pString->eg && (g < pString->eg || g > pString->sg))
       bChangeDir = 1;
    else if (pString->sb < pString->eb && b > (pString->eb || b < pString->sb))
       bChangeDir = 1;
    else if (pString->sb > pString->eb && b < (pString->eb || b > pString->sb))
       bChangeDir = 1;
    if (bChangeDir)
    {
      if (pString->iDir == 0)
      {
         r = pString->er; // make sure color values didn't overflow
         g = pString->eg;
         b = pString->eb;
         pString->iDir = 1; // reverse direction
      }
      else
      {
         r = pString->sr; // make sure color values didn't underflow
         g = pString->sg;
         b = pString->sb;
         pString->iDir = 0; // forward direction
      }    
  }
  pString->r = (uint8_t)r;
  pString->g = (uint8_t)g;
  pString->b = (uint8_t)b;
}
//
// Advance the color pattern N steps forward
// For a step value of 1, the code must do its work in < 6-9us in order to not
// trigger an accidental latching of the LED data. AVR MCUs can do solid colors
// but the gradient operation takes too long and causes data glitches
//
void LEDStep(LEDSTRING *pString, int iStepCount)
{
byte b, *s;
int i, iStep; //temp, d;

   s = pString->pRLE; // current run-length data pointer
   for (iStep=0; iStep<iStepCount; iStep++)
   {
      if (pString->iCount > 0) // remaining count of last operation, continue it
      {
         if (pString->lastOP == MODE_GRADIENT)
         {
            pString->r += pString->dr; // update color
            pString->g += pString->dg;
            pString->b += pString->db;
         }
         // Decrement current length count
         pString->iCount--;
         continue;
      } // continuation of last operation
      // Need to step into the next operation
      b = pgm_read_byte(s++); // get next operation
      if (b == MODE_END) // RLE data ended, start at the beginning again
      {
        s = pString->pRLEStart;
        b = pgm_read_byte(s++); // get next operation
      }
      pString->iCount = (b & LENGTH_MASK); // count of this 'run'
      b &= MODE_MASK;
      pString->lastOP = b;
      switch(b)
      {
//        case MODE_GAP: // set RGB and deltas to 0
//           pString->r = pString->g = pString->b = 0;
//           break;
        case MODE_SIMPLE:
           i = (int)pgm_read_byte(s++); // get the single palette entry
           i *=3; // offset to this palette entry
           pString->r = pgm_read_byte(pPalette + i++);
           pString->g = pgm_read_byte(pPalette + i++);
           pString->b = pgm_read_byte(pPalette + i++);
           break;
        case MODE_GRADIENT:
           i = (int)pgm_read_byte(s++); // get the starting palette entry
           i *=3; // offset to this palette entry
           pString->r = pgm_read_byte(pPalette + i++);
           pString->g = pgm_read_byte(pPalette + i++);
           pString->b = pgm_read_byte(pPalette + i++);
           pString->dr = pgm_read_byte(s++); // get the deltas
           pString->dg = pgm_read_byte(s++);
           pString->db = pgm_read_byte(s++);
           break;
      } // switch
   } // for iStep
   pString->pRLE = s; // keep current RLE pointer

} /* LEDStep() */

void LEDStream(byte *pStream, byte r, byte g, byte b)
{
byte *s = pStream;
byte temp, bOff, bOn, i;

   bOn = 1;
   while (bOn) // run until "on" length is zero
   {
        temp = pgm_read_byte(s++);
        bOff = (temp >> 4);
        bOn = (temp & 0xf);
        for (i=0; i<bOff; i++)
        {
            LEDSendColor(0, 0, 0); // transmit "off" pixels   
        }
        for (i=0; i<bOn; i++)
        {
            LEDSendColor(r, g, b); // transmit "on" pixels
        }
   }
} /* LEDStream() */

//
// Display a set of LED streams sequentially
//
void LEDStreamList(byte **pList, byte r, byte g, byte b)
{
byte *s;

   s = *pList++; //pointer to LED instruction list
   while (s)
   {
      LEDStream(s, r, g, b);  
      s = *pList++; // pointer to LED instruction list
   }
} /* LEDStreamList() */
//
// Send the current LED STRING (at its current offset) to the LEDs
// The benefit of this function is that it can dynamically generate the colors fast enough
// to stay below the minimum reset/latch time of the NeoPixels. This allows a long string of pixels
// to be controlled with very little RAM
//
void LEDShow(LEDSTRING *pString)
{
LEDSTRING string; // local copy to not modify original (and faster access with register vars)
int i, iOff;
byte b, *s;

   memcpy(&string, pString, sizeof(LEDSTRING)); // keep local copy
   s = string.pRLE; // current run-length data pointer
   for (i=0; i<string.iLen; i++) // loop through all of the LEDs
   {
       LEDSendColor(string.r, string.g, string.b); // transmit the current color
       // Repeat the LEDStep() logic here to avoid manipulating the values indirectly through pointers
       // The performance hit on AVR is enough to introduce data glitches
          if (string.iCount > 0) // remaining count of last operation, continue it
          {
             if (string.lastOP == MODE_GRADIENT)
             {
                string.r += string.dr; // update color with deltas
                string.g += string.dg;
                string.b += string.db;
             }
             // Decrement current length count
           string.iCount--;
           continue;
           } // continuation of last operation
         // Need to step into the next operation
         b = pgm_read_byte(s++); // get next operation
         if (b == MODE_END) // RLE data ended, start at the beginning again
         {
           s = string.pRLEStart;
           b = pgm_read_byte(s++); // get next operation
         }
         string.iCount = (b & LENGTH_MASK); // count of this 'run'
         b &= MODE_MASK;
         string.lastOP = b;
         switch(b)
         {
//           case MODE_GAP: // set RGB and deltas to 0
//              string.r = string.g = string.b = 0;
//              break;
           case MODE_SIMPLE:
              iOff = (int)pgm_read_byte(s++); // get the single palette entry
              iOff *=3; // offset to this palette entry
              string.r = pgm_read_byte(pPalette + iOff++);
              string.g = pgm_read_byte(pPalette + iOff++);
              string.b = pgm_read_byte(pPalette + iOff++);
              break;
           case MODE_GRADIENT:
              iOff = (int)pgm_read_byte(s++); // get the starting palette entry
              iOff *=3; // offset to this palette entry
              string.r = pgm_read_byte(pPalette + iOff++);
              string.g = pgm_read_byte(pPalette + iOff++);
              string.b = pgm_read_byte(pPalette + iOff++);
              string.dr = pgm_read_byte(s++); // get the deltas
              string.dg = pgm_read_byte(s++);
              string.db = pgm_read_byte(s++);
              break;
         } // switch
   } // for i
   delayMicroseconds(9); // delay to allow all LEDs to latch/display the new data
} /* LEDShow() */

//
// Returns a set bit for each button that has been pressed (change of state only)
//
uint8_t GetButtons(void)
{
static uint8_t oldButton0 = HIGH, oldButton1 = HIGH; // start in unpressed state
uint8_t button0, button1, buttons = 0;
  // Check first button
  button0 = digitalRead(YELLOW_BUTTON);
  button1 = digitalRead(ORANGE_BUTTON);
  if (button0 == LOW && oldButton0 == HIGH) // pressed
     buttons |= 1; // indicate new press
  if (button1 == LOW && oldButton1 == HIGH)
     buttons |= 2;
  oldButton0 = button0; oldButton1 = button1;
  return buttons;
} // GetButtons()

void setup() {
  pinMode(YELLOW_BUTTON, INPUT_PULLUP);
  pinMode(ORANGE_BUTTON, INPUT_PULLUP);
  Serial.begin(9600);
  // put your setup code here, to run once:
#if defined (__AVR_ATtiny85__)
  bitSet(LED_DDR , LED_BIT); // set up the pin as a digital output
#else
  SPI.begin();
  SPI.setClockDivider (SPI_CLOCK_DIV2);
  SPI.setBitOrder (MSBFIRST);
  SPI.setDataMode (SPI_MODE1);   // MOSI normally low.
  delayMicroseconds(100); // steady line before first data
#endif
}
void loop() {
int i, iDelay;
static int iProgram = 0, iType = 0;
byte *pList[3];
byte b, bChange = 0, bColor;
byte bColors[]= {0x10,0x0,0x0, 0x0,0x10,0x0, 0x0,0x0,0x10, 0x10,0x10,0x0, 0x0,0x10,0x10, 0x10,0x0,0x10, 0x10,0x10,0x10, 0x20,0x10,0x10};
const byte *programs[14*3] = {pPulse1, pPulse2, pPulse3,pPulse4, pPulse5, pPulse6,pPulse7, pPulse8, pPulse9,pPulse10, pPulse11, pPulse12,pPulse13,pPulse14,
pCircus1, pCircus2, pCircus3,pCircus1, pCircus2, pCircus3,pCircus1, pCircus2, pCircus3,pCircus1, pCircus2, pCircus3,pCircus1, pCircus2,
pSimple1, pSimple2, pSimple3, pSimple4, pSimple5, pSimple6, pSimple7, pSimple8, pSimple9, pSimple10, pSimple11, pSimple12, pSimple13, pSimple14};

  LEDInit((byte *)programs[(iProgram * 14) + iType], &mystring, 60);
  while (!bChange)
  {
     b = GetButtons();
     if (b & 1)
     {
        iProgram++;
        if (iProgram >= 3) iProgram = 0;
        bChange = 1;
     }
     if (b & 2)
     {
        iType++;
        if (iType >= 14) iType = 0;
        bChange = 1;
     }
     LEDShow(&mystring);
     if (iProgram == 0)
     {
        LEDPulse(&mystring);
        if (iType & 1) // large value swings, short delay
           delay(20);
        else
           delay(80); // small value swings, larger delay
     }
     else
     {
        LEDStep(&mystring, 1);
        delay(100);
     }
  } // while !bChange
  bChange = 0;
} // loop
