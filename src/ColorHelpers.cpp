#include "ColorHelpers.h"

/**
 * @brief Helper function to calculate transition between two colors
 * 
 * @param col_from  8bit color value start
 * @param col_to    8bit color value end 
 * @param perc      percentage in float
 * @return uint8_t  8bit color value at percentage of transition
 */
uint8_t color_value(uint8_t col_from, uint8_t col_to, float_t perc) {
  float_t col;
  if (col_from < col_to) {
    col = col_from + (float) (col_to - col_from) * perc;
  } else {
    col = col_from - (float) (col_from - col_to) * perc;
  }
  return (uint8_t) (col + 0.5);
}

/**
 * @brief Helper function to calculate transition between two colors
 * 
 * @param col_from 
 * @param col_to 
 * @param perc 
 * @return uint32_t 
 */
uint32_t color_value(uint32_t col_from, uint32_t col_to, float_t perc) {
  uint32_t result = 0;
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t cf = (col_from & (0xff << (i*8))) >> (i*8);
    uint8_t ct = (col_to & (0xff << (i*8))) >> (i*8);
    result = result | (color_value(cf, ct, perc) << (i*8));
  }
  return result;
}