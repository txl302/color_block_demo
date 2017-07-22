#ifndef _NEOPIXELLEDS_H_
#define _NEOPIXELLEDS_H_

#include "Arduino.h"
#include "Adafruit_NeoPixel.h"

#define PIN 2  // pin connection for the new block
#define NUMPIXELS 150

// struct for indexing specified pixel
// top left corner is the start of first row & column
// all indexes start from 0
struct MatrixIndex {
	uint8_t surface;
	uint8_t row;
	uint8_t column;
};

// struct for surface pattern
struct SurfaceIndex {
	uint8_t row;
	uint8_t column;
};

class NeopixelLeds
{
public:
	NeopixelLeds();  // constructor
	~NeopixelLeds() {};

	// clear all the pixels
	void clear_pixels();

	// rainbow spiral pattern
	void rainbow_spiral_begin();
	void rainbow_spiral_update();

	// rainbow fall pattern
	void rainbow_circular_begin();
	void rainbow_circular_update();

	// color ramp pattern
	void color_ramp_begin();
	void color_ramp_update();

	// color wave pattern
	void color_wave_begin();
	void color_wave_update();

	// countdown pattern
	void countdown_begin();
	void countdown_update();

	int indexing(MatrixIndex index);
	uint32_t Wheel(byte WheelPos);
	Adafruit_NeoPixel pixels;  // make it public for accessing other functions

private:
	int rainbow_spiral_step;  // current step
	int rainbow_spiral_total_step;  // total step
	unsigned long rainbow_spiral_period;  // delay time between each step
	unsigned long rainbow_spiral_timer_last, rainbow_spiral_timer_now;
	SurfaceIndex SPIRAL[25] = {{0,0}, {0,1}, {0,2}, {0,3}, {0,4}
								,{1,4}, {2,4}, {3,4}, {4,4}
								,{4,3}, {4,2}, {4,1}, {4,0}
								,{3,0}, {2,0}, {1,0}
								,{1,1}, {1,2}, {1,3}
								,{2,3}, {3,3}
								,{3,2}, {3,1}
								,{2,1}
								,{2,2}};

	int rainbow_circular_step;
	int rainbow_circular_total_step;
	unsigned long rainbow_circular_period;
	unsigned long rainbow_circular_timer_last, rainbow_circular_timer_now;

	int color_ramp_step;
	unsigned long color_ramp_period;
	unsigned long color_ramp_timer_last, color_ramp_timer_now;
	uint32_t color_ramp_colors[4][5];  // store random colors

	int color_wave_step;
	unsigned long color_wave_period;
	unsigned long color_wave_timer_last, color_wave_timer_now;
	uint32_t color_wave_colors[4][5];

	int counting;
	unsigned long countdown_period;
	unsigned long countdown_timer_last, countdown_timer_now;
	// digits from 0 to 9 on 5x5 matrix
	uint8_t DIGITS[10][5][5] = {
		{{0,1,1,1,0},{0,1,0,1,0},{0,1,0,1,0},{0,1,0,1,0},{0,1,1,1,0}}
		,{{0,0,1,0,0},{0,1,1,0,0},{0,0,1,0,0},{0,0,1,0,0},{0,1,1,1,0}}
		,{{0,1,1,1,0},{0,0,0,1,0},{0,1,1,1,0},{0,1,0,0,0},{0,1,1,1,0}}
		,{{0,1,1,1,0},{0,0,0,1,0},{0,1,1,1,0},{0,0,0,1,0},{0,1,1,1,0}}
		,{{0,1,0,1,0},{0,1,0,1,0},{0,1,1,1,0},{0,0,0,1,0},{0,0,0,1,0}}
		,{{0,1,1,1,0},{0,1,0,0,0},{0,1,1,1,0},{0,0,0,1,0},{0,1,1,1,0}}
		,{{0,1,1,1,0},{0,1,0,0,0},{0,1,1,1,0},{0,1,0,1,0},{0,1,1,1,0}}
		,{{0,1,1,1,0},{0,0,0,1,0},{0,0,0,1,0},{0,0,0,1,0},{0,0,0,1,0}}
		,{{0,1,1,1,0},{0,1,0,1,0},{0,1,1,1,0},{0,1,0,1,0},{0,1,1,1,0}}
		,{{0,1,1,1,0},{0,1,0,1,0},{0,1,1,1,0},{0,0,0,1,0},{0,1,1,1,0}}};

};


#endif

