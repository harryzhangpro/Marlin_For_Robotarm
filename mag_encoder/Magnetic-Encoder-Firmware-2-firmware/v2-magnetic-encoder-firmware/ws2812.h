/*
 *	WS2812 / NeoPixel implementation for STM32F0x microcontrollers.
 *	Assumes a 48MHz clock rate from the internal oscillator / PLL.
 *
 *  Likely to work with other STM32Fxx, provided clock rate is similar
 *  or timing is adjusted to suit.
 *
 *  Based off of the Adafruit NeoPixel library for Arduino.
 *
 *	This is a VERY basic implementation that relies on bit-banging the signal
 *	to the WS2812 LEDs, and the output waveforms are very rough - in some situations
 *	the signal may result in incorrect behaviour.
 *
 *	A better approach would use DMA to write out values to the LEDs - but this is
 *	what I have at the moment, hopefully it helps someone else get started.
 *
 * Example usage (see included example file for full example):
 *
 * 			#define WS2812_NUM 16
 * 			#define WS2812_PIN GPIO_PIN_5
 * 			#define WS2812_PORT GPIOA
 *
 * 			Ws2812 pixels = Ws2812(WS2812_NUM,WS2812_PIN,WS2812_PORT);
 *
 * 			pixels.begin();
 *
 * 			for(uint8_t i=0; i< pixels.numPixels(); i++) {
 *				pixels.setPixelColor(i, 100,100,100);
 *			}
 *
 *	Written by Chris Barr, 2016.
 */

#ifndef __ws2812
#define __ws2812

#include <Arduino.h>
//#include <stdint.h> 
//#include <stdlib.h>
//#include <stdio.h>
//#include <string.h>

#define MAX_PIXELS 2

uint32_t Wheel(uint8_t WheelPos);

class Ws2812 {
public:
	Ws2812(uint8_t num, uint16_t pin);

	void begin(void);
	void show(void);
	void setPixelColor(uint8_t pixel, uint8_t red, uint8_t green, uint8_t blue);
	void setPixelColor(uint8_t pixel, uint32_t color);
  void setPixelBrightness(uint8_t pixel, uint8_t bright);

	//helper functions to deal with entire strip at once
	void setAllColor(uint8_t red, uint8_t green, uint8_t blue);
	void setAllColor(uint32_t color);

	uint8_t numPixels(void) const;

	static uint32_t Color(uint8_t r, uint8_t g, uint8_t b);


private:

	bool begun;
	uint32_t  pixels[MAX_PIXELS];
  uint8_t   brightness[MAX_PIXELS];
	uint8_t   numLEDs;

	uint16_t pin;

  uint32_t SPIEncodedValue(uint8_t color);

};

#define BIT0 (1 << 0)
#define BIT1 (1 << 1)
#define BIT2 (1 << 2)
#define BIT3 (1 << 3)
#define BIT4 (1 << 4)
#define BIT5 (1 << 5)
#define BIT6 (1 << 6)
#define BIT7 (1 << 7)
#define BIT8 (1 << 8)
#define BIT9 (1 << 9)
#define BIT10 (1 << 10)
#define BIT11 (1 << 11)
#define BIT12 (1 << 12)
#define BIT13 (1 << 13)
#define BIT14 (1 << 14)
#define BIT15 (1 << 15)
#define BIT16 (1 << 16)
#define BIT17 (1 << 17)
#define BIT18 (1 << 18)
#define BIT19 (1 << 19)
#define BIT20 (1 << 20)
#define BIT21 (1 << 21)
#define BIT22 (1 << 22)
#define BIT23 (1 << 23)
#define BIT24 (1 << 24)
#define BIT25 (1 << 25)
#define BIT26 (1 << 26)
#define BIT27 (1 << 27)
#define BIT28 (1 << 28)
#define BIT29 (1 << 29)
#define BIT30 (1 << 30)
#define BIT31 (1 << 31)

#define ALICEBLUE               0xF0F8FF
#define AMETHYST                0x9966CC
#define ANTIQUEWHITE            0xFAEBD7
#define AQUA                    0x00FFFF
#define AQUAMARINE              0x7FFFD4
#define AZURE                   0xF0FFFF
#define BEIGE                   0xF5F5DC
#define BISQUE                  0xFFE4C4
#define BLACK                   0x000000
#define BLANCHEDALMOND          0xFFEBCD
#define BLUE                    0x0000FF
#define BLUEVIOLET              0x8A2BE2
#define BROWN                   0xA52A2A
#define BURLYWOOD               0xDEB887
#define CADETBLUE               0x5F9EA0
#define CHARTREUSE              0x7FFF00
#define CHOCOLATE               0xD2691E
#define CORAL                   0xFF7F50
#define CORNFLOWERBLUE          0x6495ED
#define CORNSILK                0xFFF8DC
#define CRIMSON                 0xDC143C
#define CYAN                    0x00FFFF
#define DARKBLUE                0x00008B
#define DARKCYAN                0x008B8B
#define DARKGOLDENROD           0xB8860B
#define DARKGRAY                0xA9A9A9
#define DARKGREY                0xA9A9A9
#define DARKGREEN               0x006400
#define DARKKHAKI               0xBDB76B
#define DARKMAGENTA             0x8B008B
#define DARKOLIVEGREEN          0x556B2F
#define DARKORANGE              0xFF8C00
#define DARKORCHID              0x9932CC
#define DARKRED                 0x8B0000
#define DARKSALMON              0xE9967A
#define DARKSEAGREEN            0x8FBC8F
#define DARKSLATEBLUE           0x483D8B
#define DARKSLATEGRAY           0x2F4F4F
#define DARKSLATEGREY           0x2F4F4F
#define DARKTURQUOISE           0x00CED1
#define DARKVIOLET              0x9400D3
#define DEEPPINK                0xFF1493
#define DEEPSKYBLUE             0x00BFFF
#define DIMGRAY                 0x696969
#define DIMGREY                 0x696969
#define DODGERBLUE              0x1E90FF
#define FIREBRICK               0xB22222
#define FLORALWHITE             0xFFFAF0
#define FORESTGREEN             0x228B22
#define FUCHSIA                 0xFF00FF
#define GAINSBORO               0xDCDCDC
#define GHOSTWHITE              0xF8F8FF
#define GOLD                    0xFFD700
#define GOLDENROD               0xDAA520
#define GRAY                    0x808080
#define GREY                    0x808080
#define GREEN                   0x008000
#define GREENYELLOW             0xADFF2F
#define HONEYDEW                0xF0FFF0
#define HOTPINK                 0xFF69B4
#define INDIANRED               0xCD5C5C
#define INDIGO                  0x4B0082
#define IVORY                   0xFFFFF0
#define KHAKI                   0xF0E68C
#define LAVENDER                0xE6E6FA
#define LAVENDERBLUSH           0xFFF0F5
#define LAWNGREEN               0x7CFC00
#define LEMONCHIFFON            0xFFFACD
#define LIGHTBLUE               0xADD8E6
#define LIGHTCORAL              0xF08080
#define LIGHTCYAN               0xE0FFFF
#define LIGHTGOLDENRODYELLOW    0xFAFAD2
#define LIGHTGREEN              0x90EE90
#define LIGHTGREY               0xD3D3D3
#define LIGHTPINK               0xFFB6C1
#define LIGHTSALMON             0xFFA07A
#define LIGHTSEAGREEN           0x20B2AA
#define LIGHTSKYBLUE            0x87CEFA
#define LIGHTSLATEGRAY          0x778899
#define LIGHTSLATEGREY          0x778899
#define LIGHTSTEELBLUE          0xB0C4DE
#define LIGHTYELLOW             0xFFFFE0
#define LIME                    0x00FF00
#define LIMEGREEN               0x32CD32
#define LINEN                   0xFAF0E6
#define MAGENTA                 0xFF00FF
#define MAROON                  0x800000
#define MEDIUMAQUAMARINE        0x66CDAA
#define MEDIUMBLUE              0x0000CD
#define MEDIUMORCHID            0xBA55D3
#define MEDIUMPURPLE            0x9370DB
#define MEDIUMSEAGREEN          0x3CB371
#define MEDIUMSLATEBLUE         0x7B68EE
#define MEDIUMSPRINGGREEN       0x00FA9A
#define MEDIUMTURQUOISE         0x48D1CC
#define MEDIUMVIOLETRED         0xC71585
#define MIDNIGHTBLUE            0x191970
#define MINTCREAM               0xF5FFFA
#define MISTYROSE               0xFFE4E1
#define MOCCASIN                0xFFE4B5
#define NAVAJOWHITE             0xFFDEAD
#define NAVY                    0x000080
#define OLDLACE                 0xFDF5E6
#define OLIVE                   0x808000
#define OLIVEDRAB               0x6B8E23
#define ORANGE                  0xFFA500
#define ORANGERED               0xFF4500
#define ORCHID                  0xDA70D6
#define PALEGOLDENROD           0xEEE8AA
#define PALEGREEN               0x98FB98
#define PALETURQUOISE           0xAFEEEE
#define PALEVIOLETRED           0xDB7093
#define PAPAYAWHIP              0xFFEFD5
#define PEACHPUFF               0xFFDAB9
#define PERU                    0xCD853F
#define PINK                    0xFFC0CB
#define PLAID                   0xCC5533
#define PLUM                    0xDDA0DD
#define POWDERBLUE              0xB0E0E6
#define PURPLE                  0x800080
#define RED                     0xFF0000
#define ROSYBROWN               0xBC8F8F
#define ROYALBLUE               0x4169E1
#define SADDLEBROWN             0x8B4513
#define SALMON                  0xFA8072
#define SANDYBROWN              0xF4A460
#define SEAGREEN                0x2E8B57
#define SEASHELL                0xFFF5EE
#define SIENNA                  0xA0522D
#define SILVER                  0xC0C0C0
#define SKYBLUE                 0x87CEEB
#define SLATEBLUE               0x6A5ACD
#define SLATEGRAY               0x708090
#define SLATEGREY               0x708090
#define SNOW                    0xFFFAFA
#define SPRINGGREEN             0x00FF7F
#define STEELBLUE               0x4682B4
#define TAN                     0xD2B48C
#define TEAL                    0x008080
#define THISTLE                 0xD8BFD8
#define TOMATO                  0xFF6347
#define TURQUOISE               0x40E0D0
#define VIOLET                  0xEE82EE
#define WHEAT                   0xF5DEB3
#define WHITE                   0xFFFFFF
#define WHITESMOKE              0xF5F5F5
#define YELLOW                  0xFFFF00
#define YELLOWGREEN             0x9ACD32


#endif
