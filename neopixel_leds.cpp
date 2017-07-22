#include "Arduino.h"
#include "neopixel_leds.h"

// constructor
NeopixelLeds::NeopixelLeds() {
	pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
    pixels.begin();
}

// clear all the pixels, used when switching to another pattern
void NeopixelLeds::clear_pixels() {
    for (int i=0; i<NUMPIXELS; i++) {
        pixels.setPixelColor(i, pixels.Color(0,0,0));
    }
    pixels.show();
}

// all the pixels are rearranged by this function
// input is a index in a 6x5x5 matrix
// output is the sequence in the neopixel serial connection, from 0 to NUMPIXELS-1
// return -1 if there is index error, not used
int NeopixelLeds::indexing(MatrixIndex index) {
	// 6 x 5 x 5 matrix specified:
	// 6 means 6 surfaces of the block
		// starting from the four sides: back, left, front, right
		// then is the bottom and top surfaces
	// for each surfaces there are five rows and five columns
	int serial_index;  // for return
	if (index.surface == 4 || index.surface == 5) {
		// the bottom or top surfaces
		serial_index = 25*index.surface + 5*index.row + index.column;
		return serial_index;
	}
	else {
		// the side surfaces
		// all the led on the side surfaces need to be rotate ccw 90 degrees
		uint8_t row_temp, column_temp;
		row_temp = 4 - index.column;
		column_temp = index.row;
		serial_index  = 25*index.surface + 5*row_temp + column_temp;
		return serial_index;
	}
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t NeopixelLeds::Wheel(byte WheelPos) {
    WheelPos = 255 - WheelPos;
    if(WheelPos < 85)
    {
        return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }
    else if(WheelPos < 170)
    {
        WheelPos -= 85;
        return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
    else
    {
        WheelPos -= 170;
        return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
    }
}

// spiral in pattern
void NeopixelLeds::rainbow_spiral_begin() {
	rainbow_spiral_step = 0;
	rainbow_spiral_total_step = 255;
	rainbow_spiral_period = 3;  // tuned
    rainbow_spiral_timer_last = millis();
}

void NeopixelLeds::rainbow_spiral_update() {
    rainbow_spiral_timer_now = millis();
    if (rainbow_spiral_timer_now - rainbow_spiral_timer_last >= rainbow_spiral_period) {
        MatrixIndex index;
        for (int i=0; i<6; i++) {
            index.surface = i;
            for (int j=0; j<25; j++) {
                index.row = SPIRAL[j].row;
                index.column = SPIRAL[j].column;
                pixels.setPixelColor(indexing(index)
                    , Wheel(((j * 256 / 25) + rainbow_spiral_step) & 255));
            }
        }
        pixels.show();
        // update rainbow spiral pattern
        rainbow_spiral_step = rainbow_spiral_step + 1;
        if (rainbow_spiral_step == rainbow_spiral_total_step) {
            rainbow_spiral_step = 0;
        }

        // update timer
        rainbow_spiral_timer_last = rainbow_spiral_timer_now;
    }
}

// rainbow circular pattern
void NeopixelLeds::rainbow_circular_begin() {
    rainbow_circular_step = 0;
    rainbow_circular_total_step = 255;
    rainbow_circular_period = 10;
    rainbow_circular_timer_last = millis();
}

void NeopixelLeds::rainbow_circular_update() {
    rainbow_circular_timer_now = millis();
    if (rainbow_circular_timer_now - rainbow_circular_timer_last >= rainbow_circular_period) {
        MatrixIndex index;
        for (int i=0; i<4; i++) {
            // 4 side surfaces
            index.surface = i;
            for (int j=0; j<5; j++) {
                // 5 columns, from right to left
                // column sequence in circular direction
                int column_sequence = i*5 + j;
                index.column = 4 - j;
                for (int k=0; k<5; k++) {
                    // 5 rows
                    index.row = k;
                    pixels.setPixelColor(indexing(index)
                        , Wheel(((column_sequence * 256 / 20) + rainbow_circular_step) & 255));
                }
            }
        }
        pixels.show();
        // update rainbow circular pattern
        rainbow_circular_step = rainbow_circular_step + 1;
        if (rainbow_circular_step == rainbow_circular_total_step) {
            rainbow_circular_step = 0;
        }

        // update timer
        rainbow_circular_timer_last = rainbow_circular_timer_now;
    }

}

// color ramp pattern
void NeopixelLeds::color_ramp_begin() {
    color_ramp_step = 0;
    color_ramp_period = 100;
    // initialize column colors
    for (int i=0; i<4; i++) {
        for (int j=0; j<5; j++) {
            color_ramp_colors[i][j] = pixels.Color(0,0,0);
        }
    }
    color_ramp_timer_last = millis();
}

void NeopixelLeds::color_ramp_update() {
    color_ramp_timer_now = millis();
    if (color_ramp_timer_now - color_ramp_timer_last >= color_ramp_period) {
        MatrixIndex index;
        for (int i=0; i<4; i++) {
            // 4 side surfaces
            index.surface = i;
            for (int j=0; j<5; j++) {
                // 5 columns, from right to left
                // column sequence in circular direction
                int column_sequence = i*5 + j;
                int column_state = color_ramp_step + column_sequence;
                column_state = column_state % 5;  // in range of [0,4]
                index.column = 4 - j;
                index.row = column_state;
                if (column_state == 0) {
                    // first pixel on the column, generate a random color
                    color_ramp_colors[i][j] = Wheel(random(255));
                }
                pixels.setPixelColor(indexing(index), color_ramp_colors[i][j]);
                // needs to wipe out last column state pixel
                int last_column_state = column_state - 1;
                if (last_column_state == -1) {
                    last_column_state = 4;
                }
                index.row = last_column_state;
                pixels.setPixelColor(indexing(index), pixels.Color(0,0,0));
            }
        }
        pixels.show();
        // update the color ramp pattern
        color_ramp_step = color_ramp_step + 1;
        if (color_ramp_step == 5) {
            color_ramp_step = 0;
        }

        // update timer
        color_ramp_timer_last = color_ramp_timer_now;
    }

}

// color wave pattern
void NeopixelLeds::color_wave_begin() {
    color_wave_step = 0;
    color_wave_period = 100;
    // initialize column colors
    for (int i=0; i<4; i++) {
        for (int j=0; j<5; j++) {
            color_wave_colors[i][j] = pixels.Color(0,0,0);
        }
    }
    // initialize timer
    color_wave_timer_last = millis();
}

void NeopixelLeds::color_wave_update() {
    color_wave_timer_now = millis();
    if (color_wave_timer_now - color_wave_timer_last >= color_wave_period) {
        MatrixIndex index;
        for (int i=0; i<4; i++) {
            // 4 side surfaces
            index.surface = i;
            for (int j=0; j<5; j++) {
                // 5 columns, from right to left
                // column sequence in circular direction
                int column_sequence = i*5 + j;
                int column_state = color_wave_step + column_sequence;
                column_state = column_state % 10;  // for further processing
                if (column_state >=5) {
                    column_state = 9 - column_state;
                }
                index.column = 4 - j;
                index.row = column_state;
                if (column_state == 0) {
                    // run to first pixel on the column, generate a random color
                    color_wave_colors[i][j] = Wheel(random(255));
                }
                pixels.setPixelColor(indexing(index), color_wave_colors[i][j]);
                // wipe out neighboring pixels
                switch (column_state) {
                    case 0:
                        index.row = 1;
                        pixels.setPixelColor(indexing(index), pixels.Color(0,0,0));
                        break;
                    case 1:
                        index.row = 0;
                        pixels.setPixelColor(indexing(index), pixels.Color(0,0,0));
                        index.row = 2;
                        pixels.setPixelColor(indexing(index), pixels.Color(0,0,0));
                        break;
                    case 2:
                        index.row = 1;
                        pixels.setPixelColor(indexing(index), pixels.Color(0,0,0));
                        index.row = 3;
                        pixels.setPixelColor(indexing(index), pixels.Color(0,0,0));
                        break;
                    case 3:
                        index.row = 2;
                        pixels.setPixelColor(indexing(index), pixels.Color(0,0,0));
                        index.row = 4;
                        pixels.setPixelColor(indexing(index), pixels.Color(0,0,0));
                        break;
                    case 4:
                        index.row = 3;
                        pixels.setPixelColor(indexing(index), pixels.Color(0,0,0));
                        break;
                }
            }
        }
        pixels.show();
        // update the color ramp pattern
        color_wave_step = color_wave_step + 1;
        if (color_wave_step == 10) {
            color_wave_step = 0;
        }

        // update timer
        color_wave_timer_last = color_wave_timer_now;
    }

}

// countdown pattern
void NeopixelLeds::countdown_begin() {
    counting = 9;  // counting from 9 to 0
    countdown_period = 1000;
    countdown_timer_last = millis();
}

void NeopixelLeds::countdown_update() {
    countdown_timer_now = millis();
    if (countdown_timer_now - countdown_timer_last >= countdown_period) {
        MatrixIndex index;
        for (int i=0; i<6; i++) {
            // all 6 surfaces are counting down
            index.surface = i;
            for (int j=0; j<5; j++) {
                // row
                index.row = j;
                for (int k=0; k<5; k++) {
                    // column
                    index.column = k;
                    if (DIGITS[counting][j][k]) {
                        // light up
                        pixels.setPixelColor(indexing(index), pixels.Color(255,255,255));
                    }
                    else {
                        // wipe out
                        pixels.setPixelColor(indexing(index), pixels.Color(0,0,0));
                    }
                }
            }
        }
        pixels.show();
        // update counting
        counting = counting - 1;
        if (counting < 0) {
            counting = 9;
        }

        // update timer
        countdown_timer_last = countdown_timer_now;
    }

}



