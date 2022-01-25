#ifndef COLORHELPERS_H
#define COLORHELPERS_H

#include <Arduino.h>

/**
 * @brief Helper function to calculate a fractional value of perc between col_from and col_to
 * 
 * @param col_from 8bit lower bound value
 * @param col_to 8bit upper bound value
 * @param perc fraction between 0 and 1
 * @return uint8_t 
 */
uint8_t color_value(uint8_t col_from, uint8_t col_to, float_t perc);

/**
 * @brief Helper function to calculate color transitions between two 32bit color values
 * 
 * @param col_from 
 * @param col_to 
 * @param perc 
 * @return uint32_t 
 */
uint32_t color_value(uint32_t col_from, uint32_t col_to, float_t perc);

#endif